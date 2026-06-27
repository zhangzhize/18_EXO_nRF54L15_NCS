#include "ble_sensor_gateway.hpp"

#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bsp_led.h"
#include "pressure_insole.h"
#include "wit_imu.h"

LOG_MODULE_REGISTER(ble_sensor_gateway, LOG_LEVEL_INF);

#define SENSOR_STREAM_BUF_SIZE 256
#define SENSOR_LOG_EVERY_N_FRAMES 1000U
#define SENSOR_READY_MIN_FRAMES 5U
#define SENSOR_STALE_TIMEOUT_MS 500U
#define SENSOR_STATS_EVERY_N_FRAMES 1000U

struct sensor_stream_state
{
  bool in_use;
  bt_addr_le_t addr;
  uint8_t buf[SENSOR_STREAM_BUF_SIZE];
  size_t len;
  uint32_t frames;
  uint32_t last_notify_ms;
  uint32_t max_notify_interval_ms;
  uint16_t max_notify_len;
  uint8_t max_frames_per_notify;
};

struct sensor_rx_state
{
  uint32_t frames;
  uint32_t last_update_ms;
  uint32_t max_interval_ms;
};

static sensor_stream_state sensor_streams[CONFIG_BT_MAX_CONN];
static sensor_rx_state imu_rx_states[EXO_BLE_IMU_COUNT];
static sensor_rx_state insole_rx_states[EXO_BLE_INSOLE_COUNT];
static struct k_spinlock packet_lock;
static uint32_t global_frame_count;

static struct bt_uuid_128 imu_svc_uuid = BT_UUID_INIT_128(
  BT_UUID_128_ENCODE(0x0000FFE5, 0x0000, 0x1000, 0x8000, 0x00805F9A34FB));
static struct bt_uuid_128 imu_notify_uuid = BT_UUID_INIT_128(
  BT_UUID_128_ENCODE(0x0000FFE4, 0x0000, 0x1000, 0x8000, 0x00805F9A34FB));

static bt_addr_le_t imu_macs[] = {
  /* [0] left shank  - F3:BE:1E:29:60:D0 */ {BT_ADDR_LE_RANDOM, {{0xD0, 0x60, 0x29, 0x1E, 0xBE, 0xF3}}},
  /* [1] right shank - E8:16:43:84:A1:68 */ {BT_ADDR_LE_RANDOM, {{0x68, 0xA1, 0x84, 0x43, 0x16, 0xE8}}},
  /* [2] left thigh  - DB:FD:BB:F0:5E:07 */ {BT_ADDR_LE_RANDOM, {{0x07, 0x5E, 0xF0, 0xBB, 0xFD, 0xDB}}},
  /* [3] right thigh - F8:48:DF:38:CE:0A */ {BT_ADDR_LE_RANDOM, {{0x0A, 0xCE, 0x38, 0xDF, 0x48, 0xF8}}},
};

BUILD_ASSERT(ARRAY_SIZE(imu_macs) == EXO_BLE_IMU_COUNT,
             "imu_macs must match EXO_BLE_IMU_COUNT");

static sensor_stream_state *sensor_stream_for_addr(const bt_addr_le_t *addr)
{
  for (size_t i = 0; i < ARRAY_SIZE(sensor_streams); i++)
  {
    if (sensor_streams[i].in_use && bt_addr_le_cmp(&sensor_streams[i].addr, addr) == 0)
    {
      return &sensor_streams[i];
    }
  }

  for (size_t i = 0; i < ARRAY_SIZE(sensor_streams); i++)
  {
    if (!sensor_streams[i].in_use)
    {
      memset(&sensor_streams[i], 0, sizeof(sensor_streams[i]));
      sensor_streams[i].in_use = true;
      bt_addr_le_copy(&sensor_streams[i].addr, addr);
      return &sensor_streams[i];
    }
  }

  return NULL;
}

static void sensor_stream_drop(sensor_stream_state *stream, size_t count)
{
  if (count >= stream->len)
  {
    stream->len = 0;
    return;
  }

  memmove(stream->buf, stream->buf + count, stream->len - count);
  stream->len -= count;
}

static bool sensor_stream_append(sensor_stream_state *stream, const uint8_t *data, size_t len)
{
  if (len > sizeof(stream->buf))
  {
    data += len - sizeof(stream->buf);
    len = sizeof(stream->buf);
    stream->len = 0;
  }

  if (stream->len + len > sizeof(stream->buf))
  {
    LOG_WRN("Sensor stream overflow for %02X, dropping buffered %u bytes",
            stream->addr.a.val[5],
            (unsigned int)stream->len);
    stream->len = 0;
  }

  memcpy(stream->buf + stream->len, data, len);
  stream->len += len;
  return true;
}

static bool sensor_state_ready(const sensor_rx_state *state, uint32_t now_ms)
{
  return state->frames >= SENSOR_READY_MIN_FRAMES &&
         (uint32_t)(now_ms - state->last_update_ms) <= SENSOR_STALE_TIMEOUT_MS;
}

static size_t find_imu_frame_start(const uint8_t *data, size_t len)
{
  for (size_t i = 0; i + 1 < len; i++)
  {
    if (data[i] == WIT_FRAME_HEADER && data[i + 1] == WIT_FRAME_FLAG_ACC_GYRO_ANGLE)
    {
      return i;
    }
  }

  return len;
}

static int imu_slot_from_addr(const bt_addr_le_t *addr)
{
  for (size_t i = 0; i < ARRAY_SIZE(imu_macs); i++)
  {
    if (bt_addr_le_cmp(addr, &imu_macs[i]) == 0)
    {
      return (int)i;
    }
  }

  return -ENOENT;
}

static void update_imu_packet(uint8_t slot, const wit_imu_data_t *imu)
{
  if (slot >= EXO_BLE_IMU_COUNT || imu == NULL)
  {
    return;
  }

  uint32_t now_ms = k_uptime_get_32();
  bool toggle_led;
  uint32_t interval_ms = 0U;
  uint32_t frames;
  uint32_t max_interval_ms;
  bool log_stats;
  k_spinlock_key_t key = k_spin_lock(&packet_lock);

  exo_ble_sensor_data.imu[slot].gyro_radps[0] = imu->wx * (3.14159265f / 180.0f);
  exo_ble_sensor_data.imu[slot].gyro_radps[1] = imu->wy * (3.14159265f / 180.0f);
  exo_ble_sensor_data.imu[slot].gyro_radps[2] = imu->wz * (3.14159265f / 180.0f);

  exo_ble_sensor_data.imu[slot].euler_rad[0] = imu->roll * (3.14159265f / 180.0f);
  exo_ble_sensor_data.imu[slot].euler_rad[1] = imu->pitch * (3.14159265f / 180.0f);
  exo_ble_sensor_data.imu[slot].euler_rad[2] = imu->yaw * (3.14159265f / 180.0f);

  if (imu_rx_states[slot].last_update_ms != 0U)
  {
    interval_ms = now_ms - imu_rx_states[slot].last_update_ms;
    if (interval_ms > imu_rx_states[slot].max_interval_ms)
    {
      imu_rx_states[slot].max_interval_ms = interval_ms;
    }
  }

  imu_rx_states[slot].frames++;
  imu_rx_states[slot].last_update_ms = now_ms;
  frames = imu_rx_states[slot].frames;
  max_interval_ms = imu_rx_states[slot].max_interval_ms;
  log_stats = (frames % SENSOR_STATS_EVERY_N_FRAMES) == 0U;
  if (log_stats)
  {
    imu_rx_states[slot].max_interval_ms = 0U;
  }

  global_frame_count++;
  toggle_led = (global_frame_count % 100U) == 0U;

  k_spin_unlock(&packet_lock, key);

  if (log_stats)
  {
    LOG_INF("[IMU %u] sample frames=%u last_dt=%u ms window_max_dt=%u ms",
            slot,
            frames,
            interval_ms,
            max_interval_ms);
  }

  if (toggle_led)
  {
    bsp_led_toggle();
  }
}

static void log_imu_frame(sensor_stream_state *stream, uint8_t slot, const wit_imu_data_t *parsed)
{
  stream->frames++;
  if ((stream->frames % SENSOR_LOG_EVERY_N_FRAMES) == 0U)
  {
    LOG_INF("[IMU %u %02X:%02X] roll=%.1f pitch=%.1f yaw=%.1f frames=%u",
            slot,
            stream->addr.a.val[5],
            stream->addr.a.val[4],
            static_cast<double>(parsed->roll),
            static_cast<double>(parsed->pitch),
            static_cast<double>(parsed->yaw),
            stream->frames);
    LOG_INF("[IMU %u] notify dt_max=%u ms len_max=%u frames_max=%u",
            slot,
            stream->max_notify_interval_ms,
            stream->max_notify_len,
            stream->max_frames_per_notify);

    stream->max_notify_interval_ms = 0U;
    stream->max_notify_len = 0U;
    stream->max_frames_per_notify = 0U;
  }
}

static void imu_raw_cb(const bt_addr_le_t *addr, const uint8_t *data, size_t len)
{
  int slot = imu_slot_from_addr(addr);
  sensor_stream_state *stream = sensor_stream_for_addr(addr);
  uint32_t now_ms = k_uptime_get_32();
  uint8_t frames_in_notify = 0U;

  if (slot < 0 || !stream || !sensor_stream_append(stream, data, len))
  {
    return;
  }

  if (stream->last_notify_ms != 0U)
  {
    uint32_t notify_interval_ms = now_ms - stream->last_notify_ms;
    if (notify_interval_ms > stream->max_notify_interval_ms)
    {
      stream->max_notify_interval_ms = notify_interval_ms;
    }
  }

  stream->last_notify_ms = now_ms;
  if (len > stream->max_notify_len)
  {
    stream->max_notify_len = (uint16_t)MIN(len, UINT16_MAX);
  }

  while (stream->len >= 2U)
  {
    size_t frame_start = find_imu_frame_start(stream->buf, stream->len);
    if (frame_start > 0U)
    {
      sensor_stream_drop(stream, frame_start);
    }

    if (stream->len < WIT_FRAME_LEN)
    {
      break;
    }

    wit_imu_data_t parsed;
    if (wit_imu_parse(stream->buf, WIT_FRAME_LEN, &parsed))
    {
      update_imu_packet((uint8_t)slot, &parsed);
      if (frames_in_notify < UINT8_MAX)
      {
        frames_in_notify++;
      }

      log_imu_frame(stream, (uint8_t)slot, &parsed);
      sensor_stream_drop(stream, WIT_FRAME_LEN);
    }
    else
    {
      sensor_stream_drop(stream, 1U);
    }
  }

  if (frames_in_notify > stream->max_frames_per_notify)
  {
    stream->max_frames_per_notify = frames_in_notify;
  }
}

/* -- Pressure insole profile --------------------------------- */

static struct bt_uuid_128 insole_svc_uuid = BT_UUID_INIT_128(
  BT_UUID_128_ENCODE(0xFE7B000A, 0xF4BB, 0x41DD, 0xA968, 0xDEB694CE2365));
static struct bt_uuid_128 insole_notify_uuid = BT_UUID_INIT_128(
  BT_UUID_128_ENCODE(0xFE7B000B, 0xF4BB, 0x41DD, 0xA968, 0xDEB694CE2365));
static struct bt_uuid_128 insole_write_uuid = BT_UUID_INIT_128(
  BT_UUID_128_ENCODE(0xFE7B000C, 0xF4BB, 0x41DD, 0xA968, 0xDEB694CE2365));

static bt_addr_le_t insole_macs[] = {
  /* [0] left foot  - CA:E8:5C:D4:08:54 */ {BT_ADDR_LE_RANDOM, {{0x54, 0x08, 0xD4, 0x5C, 0xE8, 0xCA}}},
   /* [1] right foot - F7:67:CA:78:3F:AF */ {BT_ADDR_LE_RANDOM, {{0xAF, 0x3F, 0x78, 0xCA, 0x67, 0xF7}}},
};

static int insole_slot_from_addr(const bt_addr_le_t *addr)
{
  for (size_t i = 0; i < ARRAY_SIZE(insole_macs); i++)
  {
    if (bt_addr_le_cmp(addr, &insole_macs[i]) == 0) { return (int)i; }
  }
  return -ENOENT;
}

static void update_insole_packet(uint8_t slot, const pressure_insole_data_t *parsed)
{
  if (slot >= EXO_BLE_INSOLE_COUNT || parsed == NULL) { return; }

  k_spinlock_key_t key = k_spin_lock(&packet_lock);

  memcpy(exo_ble_sensor_data.insole[slot].channels, parsed->channels,
         sizeof(exo_ble_sensor_data.insole[slot].channels));

  uint32_t now_ms = k_uptime_get_32();
  insole_rx_states[slot].frames++;
  insole_rx_states[slot].last_update_ms = now_ms;

  k_spin_unlock(&packet_lock, key);
}

static void insole_raw_cb(const bt_addr_le_t *addr, const uint8_t *data, size_t len)
{
  int slot = insole_slot_from_addr(addr);
  if (slot < 0) { return; }

  pressure_insole_data_t parsed;
  if (pressure_insole_parse(data, len, &parsed))
  {
    update_insole_packet((uint8_t)slot, &parsed);
  }
}

/* Send start command (0xAA 0x0D 0x55 0x0A) to insole after subscribe */
static void insole_post_subscribe_cb(struct bt_conn *conn, uint16_t write_handle)
{
  if (write_handle == 0) { return; }

  static const uint8_t start_cmd[] = {0xAA, 0x0D, 0x55, 0x0A};

  int err = bt_gatt_write_without_response(conn, write_handle, start_cmd,
                                           sizeof(start_cmd), false);
  if (err)
  {
    LOG_WRN("insole start cmd write failed (err %d)", err);
  }
  else
  {
    LOG_INF("insole start cmd sent");
  }
}

/* -- Device profiles --------------------------------- */

static struct ble_device_profile device_profiles[] = {
  {
    .macs = imu_macs,
    .mac_count = ARRAY_SIZE(imu_macs),
    .svc_uuid = (struct bt_uuid *)&imu_svc_uuid,
    .chrc_uuid = (struct bt_uuid *)&imu_notify_uuid,
    .data_cb = imu_raw_cb,
  },
  {
    .macs = insole_macs,
    .mac_count = ARRAY_SIZE(insole_macs),
    .svc_uuid = (struct bt_uuid *)&insole_svc_uuid,
    .chrc_uuid = (struct bt_uuid *)&insole_notify_uuid,
    .write_chrc_uuid = (struct bt_uuid *)&insole_write_uuid,
    .data_cb = insole_raw_cb,
    .post_subscribe_cb = insole_post_subscribe_cb,
  },
};


const struct ble_device_profile *ble_sensor_gateway_profiles(size_t *count)
{
  if (count)
  {
    *count = ARRAY_SIZE(device_profiles);
  }

  return device_profiles;
}

size_t ble_sensor_gateway_target_count(void)
{
  size_t total = 0U;

  for (size_t i = 0; i < ARRAY_SIZE(device_profiles); i++)
  {
    total += device_profiles[i].mac_count;
  }

  return total;
}

bool ble_sensor_gateway_streams_ready(void)
{
  bool ready = true;
  uint32_t now_ms = k_uptime_get_32();
  k_spinlock_key_t key = k_spin_lock(&packet_lock);

  for (size_t i = 0; i < ARRAY_SIZE(imu_rx_states); i++)
  {
    if (!sensor_state_ready(&imu_rx_states[i], now_ms))
    {
      ready = false;
      break;
    }
  }

  for (size_t i = 0; i < ARRAY_SIZE(insole_rx_states); i++)
  {
    if (!sensor_state_ready(&insole_rx_states[i], now_ms))
    {
      ready = false;
      break;
    }
  }

  k_spin_unlock(&packet_lock, key);
  return ready;
}

void ble_sensor_gateway_copy_packet(exo_ble_sensor_packet_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  k_spinlock_key_t key = k_spin_lock(&packet_lock);

  memcpy(snapshot, &exo_ble_sensor_data, sizeof(*snapshot));

  k_spin_unlock(&packet_lock, key);
}
