#include <zephyr/kernel.h>

#if CONFIG_BT && CONFIG_BT_PERIPHERAL && CONFIG_BT_NUS

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include "bsp_ble.h"
#include "bsp_ble_periph.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bsp_ble_periph, LOG_LEVEL_INF);

/* ── NUS connection tracking ─────────────────────────────────────── */

static struct bt_conn *nus_conns[CONFIG_BT_MAX_CONN];
static uint8_t nus_conn_count;

static bool nus_conn_exists(struct bt_conn *conn)
{
  for (uint8_t i = 0; i < nus_conn_count; i++)
  {
    if (nus_conns[i] == conn) { return true; }
  }
  return false;
}

static void nus_conn_add(struct bt_conn *conn)
{
  if (nus_conn_exists(conn) || nus_conn_count >= CONFIG_BT_MAX_CONN) { return; }
  nus_conns[nus_conn_count++] = bt_conn_ref(conn);
  LOG_INF("NUS conn %u/%u", nus_conn_count, CONFIG_BT_MAX_CONN);
}

static void nus_conn_remove(struct bt_conn *conn)
{
  for (uint8_t i = 0; i < nus_conn_count; i++)
  {
    if (nus_conns[i] != conn) { continue; }
    bt_conn_unref(nus_conns[i]);
    nus_conns[i] = nus_conns[--nus_conn_count];
    nus_conns[nus_conn_count] = NULL;
    LOG_INF("NUS conn %u/%u removed", nus_conn_count, CONFIG_BT_MAX_CONN);
    return;
  }
}

/* ── NUS data receive ────────────────────────────────────────────── */

static ble_rx_cb_t ble_rx_user_cb;
static ble_rx_conn_cb_t ble_rx_conn_user_cb;

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
  nus_conn_add(conn);

  if (ble_rx_conn_user_cb) { ble_rx_conn_user_cb(conn, data, len); }
  else if (ble_rx_user_cb)  { ble_rx_user_cb(data, len); }
}

static struct bt_nus_cb nus_cb = {
  .received = bt_receive_cb,
  .sent = NULL,
  .send_enabled = NULL,
};

void bsp_ble_rx_cb_register(ble_rx_cb_t cb)
{
  ble_rx_user_cb = cb;
}

void bsp_ble_rx_conn_cb_register(ble_rx_conn_cb_t cb)
{
  ble_rx_conn_user_cb = cb;
}

/* ── Advertising ─────────────────────────────────────────────────── */

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static struct k_work adv_work;
static uint8_t active_conn_count;

struct conn_update_context
{
  struct k_work_delayable work;
  struct bt_conn *conn;
#if defined(CONFIG_BT_GATT_CLIENT)
  struct bt_gatt_exchange_params mtu_params;
#endif
};

static struct conn_update_context conn_updates[CONFIG_BT_MAX_CONN];

static const struct bt_data ad[] = {
  BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
  BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
  BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

static void advertising_start(void)
{
  k_work_submit(&adv_work);
}

static void adv_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);

  int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  if (err == -EALREADY || err == -EBUSY)
  {
    return;
  }

  if (err)
  {
    LOG_ERR("Advertising failed to start (err %d)", err);
    return;
  }

  LOG_INF("Advertising started");
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static const char *phy_to_str(uint8_t phy)
{
  switch (phy)
  {
  case BT_GAP_LE_PHY_1M:
    return "1M";
  case BT_GAP_LE_PHY_2M:
    return "2M";
  case BT_GAP_LE_PHY_CODED:
    return "coded";
  default:
    return "unknown";
  }
}
#endif

#if defined(CONFIG_BT_GATT_CLIENT)
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
  ARG_UNUSED(params);

  if (err)
  {
    LOG_WRN("MTU exchange failed (err 0x%02x), using MTU %u", err, bt_gatt_get_mtu(conn));
    return;
  }

  LOG_INF("MTU exchange complete, MTU %u", bt_gatt_get_mtu(conn));
}
#endif

static struct conn_update_context *find_update_context(struct bt_conn *conn)
{
  for (size_t i = 0; i < ARRAY_SIZE(conn_updates); i++)
  {
    if (conn_updates[i].conn == conn)
    {
      return &conn_updates[i];
    }
  }

  return NULL;
}

static struct conn_update_context *alloc_update_context(struct bt_conn *conn)
{
  for (size_t i = 0; i < ARRAY_SIZE(conn_updates); i++)
  {
    if (!conn_updates[i].conn)
    {
      conn_updates[i].conn = bt_conn_ref(conn);
      return &conn_updates[i];
    }
  }

  return NULL;
}

static void release_update_context(struct conn_update_context *ctx)
{
  if (!ctx || !ctx->conn)
  {
    return;
  }

  (void)k_work_cancel_delayable(&ctx->work);
  bt_conn_unref(ctx->conn);
  ctx->conn = NULL;
}

static void request_conn_updates(struct conn_update_context *ctx)
{
  if (!ctx || !ctx->conn)
  {
    return;
  }

  struct bt_conn *conn = ctx->conn;
  struct bt_le_conn_param param = {
    .interval_min = 12,
    .interval_max = 12,
    .latency = 0,
    .timeout = 400,
  };

  int err = bt_conn_le_param_update(conn, &param);
  if (err)
  {
    LOG_WRN("bt_conn_le_param_update() failed (err %d)", err);
  }
  else
  {
    LOG_INF("Conn param update requested: interval=15ms latency=0 timeout=4s");
  }

#if defined(CONFIG_BT_USER_PHY_UPDATE)
  err = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
  if (err)
  {
    LOG_WRN("2M PHY update request failed (err %d)", err);
  }
#endif

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
  err = bt_conn_le_data_len_update(conn, BT_LE_DATA_LEN_PARAM_MAX);
  if (err)
  {
    LOG_WRN("DLE update request failed (err %d)", err);
  }
#endif

#if defined(CONFIG_BT_GATT_CLIENT)
  ctx->mtu_params.func = mtu_exchange_cb;
  err = bt_gatt_exchange_mtu(conn, &ctx->mtu_params);
  if (err)
  {
    if (err == -EALREADY)
    {
      LOG_INF("MTU already exchanged, MTU %u", bt_gatt_get_mtu(conn));
    }
    else
    {
      LOG_WRN("MTU exchange request failed (err %d), using MTU %u", err, bt_gatt_get_mtu(conn));
    }
  }
#endif
}

static void conn_update_work_handler(struct k_work *work)
{
  struct k_work_delayable *delayable = k_work_delayable_from_work(work);
  struct conn_update_context *ctx =
    CONTAINER_OF(delayable, struct conn_update_context, work);

  request_conn_updates(ctx);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
  char addr[BT_ADDR_LE_STR_LEN];

  if (err)
  {
    LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
    return;
  }

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Connected %s", addr);

  struct conn_update_context *ctx = alloc_update_context(conn);
  if (!ctx)
  {
    LOG_ERR("No connection update context available");
    (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    return;
  }

  active_conn_count++;
  k_work_schedule(&ctx->work, K_MSEC(100));

  if (active_conn_count < CONFIG_BT_MAX_CONN)
  {
    advertising_start();
  }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

  if (active_conn_count > 0)
  {
    active_conn_count--;
  }

  nus_conn_remove(conn);
  release_update_context(find_update_context(conn));

  if (active_conn_count < CONFIG_BT_MAX_CONN)
  {
    advertising_start();
  }
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static void phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("PHY updated: %s TX=%s RX=%s",
          addr, phy_to_str(param->tx_phy), phy_to_str(param->rx_phy));
}
#endif

static void param_updated(struct bt_conn *conn, uint16_t interval,
                          uint16_t latency, uint16_t timeout)
{
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Conn param applied: %s interval=%.1fms latency=%d timeout=%dms",
          addr, (double)(interval * 1.25f), latency, timeout * 10);
}

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void data_len_updated(struct bt_conn *conn,
                             struct bt_conn_le_data_len_info *info)
{
  char addr[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Data length updated: %s TX=%u/%uus RX=%u/%uus",
          addr, info->tx_max_len, info->tx_max_time,
          info->rx_max_len, info->rx_max_time);
}
#endif

static void recycled_cb(void)
{
  LOG_INF("Connection object recycled, restarting advertising");
  advertising_start();
}

int bsp_ble_periph_init(void)
{
  static struct bt_conn_cb conn_cb = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled = recycled_cb,
    .le_param_updated = param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    .le_phy_updated = phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
    .le_data_len_updated = data_len_updated,
#endif
  };

  bt_conn_cb_register(&conn_cb);

  int err = bt_nus_init(&nus_cb);
  if (err)
  {
    LOG_ERR("Failed to initialize NUS (err: %d)", err);
    return -1;
  }

  k_work_init(&adv_work, adv_work_handler);
  for (size_t i = 0; i < ARRAY_SIZE(conn_updates); i++)
  {
    k_work_init_delayable(&conn_updates[i].work, conn_update_work_handler);
  }

  advertising_start();

  return 0;
}

/* ── NUS public API ──────────────────────────────────────────────── */

int ble_nus_send_to(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
  if (!conn || !data || len == 0) { return -EINVAL; }

  uint16_t mtu = bt_gatt_get_mtu(conn);
  uint16_t max_payload = (mtu > 3U) ? (mtu - 3U) : 20U;
  uint16_t offset = 0;

  while (offset < len)
  {
    uint16_t chunk = MIN(max_payload, (uint16_t)(len - offset));
    int err = bt_nus_send(conn, &data[offset], chunk);

    if (err)
    {
      LOG_DBG("bt_nus_send failed: %d", err);
      return err;
    }

    offset += chunk;
  }

  return 0;
}

struct bt_conn *ble_nus_get_conn(uint8_t idx)
{
  return (idx < nus_conn_count) ? nus_conns[idx] : NULL;
}

uint8_t ble_nus_get_conn_count(void)
{
  return nus_conn_count;
}

#endif
