#include <zephyr/kernel.h>

#if CONFIG_BT_CENTRAL && CONFIG_BT

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/scan.h>

#include "bsp_ble.h"
#include "bsp_ble_central.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bsp_ble_central, LOG_LEVEL_INF);

#define BLE_CREATE_SCAN_INTERVAL BT_GAP_SCAN_FAST_INTERVAL
#define BLE_CREATE_SCAN_WINDOW BT_GAP_SCAN_FAST_INTERVAL
#define BLE_CREATE_TIMEOUT_MS 20000
#define BLE_POST_CONNECT_SETTLE_MS 300
#define BLE_RETRY_BASE_MS 5000
#define BLE_RETRY_MAX_MS 30000
#define BLE_CONN_INTERVAL_MIN 6 /* 7.5 ms */
#define BLE_CONN_INTERVAL_MAX 6 /* 7.5 ms */
#define BLE_CONN_INTERVAL_FALLBACK 12 /* 15 ms */
#define BLE_CONN_LATENCY 0
#define BLE_CONN_TIMEOUT BT_GAP_MS_TO_CONN_TIMEOUT(4000)
#define BLE_REQUIRED_ATT_MTU CONFIG_BT_L2CAP_TX_MTU
#define BLE_REQUIRED_DATA_LEN CONFIG_BT_BUF_ACL_RX_SIZE
#define BLE_EXCHANGE_ATT_MTU 1

struct central_conn
{
  struct bt_conn *conn;
  bt_addr_le_t addr;
  uint8_t profile_idx;
  uint16_t service_end_handle;
  uint16_t value_handle;
  uint16_t write_handle;
  struct bt_gatt_discover_params discover_params;
  struct bt_gatt_subscribe_params sub_params;
  struct bt_gatt_exchange_params mtu_params;
  bool in_use;
  bool connected;
  bool mtu_exchange_pending;
  bool discovering;
  bool subscribed;
  uint8_t ci_retry_count;
  bool phy_2m_ready;
  bool data_len_ready;
  bool mtu_ready;
  uint16_t interval;
  uint16_t latency;
  uint16_t timeout;
};

struct central_peer_state
{
  bt_addr_le_t addr;
  uint8_t fail_count;
  int64_t retry_after_ms;
  bool in_use;
};

static struct central_conn conns[CONFIG_BT_MAX_CONN];
static struct central_peer_state peer_states[CONFIG_BT_MAX_CONN];
static const struct ble_device_profile *profiles;
static uint8_t profile_count;
static uint8_t target_count;
static bool started;

static void start_next_discovery(void);
static void resume_scan_after_settle(void);

static void resume_scan(void)
{
  int err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);

  if (err && err != -EALREADY)
  {
    LOG_WRN("bt_scan_start failed (err %d)", err);
  }
}

static void scan_resume_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);
  resume_scan();
}

K_WORK_DELAYABLE_DEFINE(scan_resume_work, scan_resume_work_handler);

static uint8_t connected_count(void)
{
  uint8_t count = 0;

  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (conns[i].in_use && conns[i].connected)
    {
      count++;
    }
  }

  return count;
}

static bool all_targets_connected(void)
{
  return target_count > 0 && connected_count() >= target_count;
}

static bool any_discovery_running(void)
{
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (conns[i].in_use && conns[i].discovering)
    {
      return true;
    }
  }

  return false;
}

static void resume_scan_if_needed(void)
{
  if (!all_targets_connected())
  {
    resume_scan();
  }
}

static void resume_scan_after_settle(void)
{
  if (!all_targets_connected())
  {
    (void)k_work_reschedule(&scan_resume_work, K_MSEC(BLE_POST_CONNECT_SETTLE_MS));
  }
}

static struct central_conn *find_conn(const bt_addr_le_t *addr)
{
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (conns[i].in_use && bt_addr_le_cmp(&conns[i].addr, addr) == 0)
    {
      return &conns[i];
    }
  }

  return NULL;
}

static struct central_peer_state *find_peer_state(const bt_addr_le_t *addr)
{
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (peer_states[i].in_use && bt_addr_le_cmp(&peer_states[i].addr, addr) == 0)
    {
      return &peer_states[i];
    }
  }

  return NULL;
}

static void peer_mark_success(const bt_addr_le_t *addr)
{
  struct central_peer_state *state = find_peer_state(addr);

  if (!state)
  {
    return;
  }

  state->fail_count = 0;
  state->retry_after_ms = 0;
}

static void peer_mark_failure(const bt_addr_le_t *addr)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  struct central_peer_state *state = find_peer_state(addr);
  uint32_t backoff_ms;

  if (!state)
  {
    return;
  }

  if (state->fail_count < 6)
  {
    state->fail_count++;
  }

  backoff_ms = BLE_RETRY_BASE_MS << (state->fail_count - 1);
  backoff_ms = MIN(backoff_ms, BLE_RETRY_MAX_MS);
  state->retry_after_ms = k_uptime_get() + backoff_ms;

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
  LOG_WRN("Connection backoff for %s: %u ms after %u failures",
          addr_str,
          backoff_ms,
          state->fail_count);
}

static bool peer_retry_ready(const bt_addr_le_t *addr)
{
  struct central_peer_state *state = find_peer_state(addr);

  if (!state || state->retry_after_ms == 0)
  {
    return true;
  }

  return k_uptime_get() >= state->retry_after_ms;
}

static int peer_retry_remaining_ms(const bt_addr_le_t *addr)
{
  struct central_peer_state *state = find_peer_state(addr);
  int64_t remaining;

  if (!state || state->retry_after_ms == 0)
  {
    return 0;
  }

  remaining = state->retry_after_ms - k_uptime_get();
  return remaining > 0 ? (int)remaining : 0;
}

static struct central_conn *find_conn_by_sub_params(struct bt_gatt_subscribe_params *params)
{
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (&conns[i].sub_params == params)
    {
      return &conns[i];
    }
  }

  return NULL;
}

static struct central_conn *alloc_conn(void)
{
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (!conns[i].in_use)
    {
      memset(&conns[i], 0, sizeof(conns[i]));
      conns[i].in_use = true;
      return &conns[i];
    }
  }

  return NULL;
}

static void clear_conn(struct central_conn *c)
{
  if (!c)
  {
    return;
  }

  if (c->conn)
  {
    bt_conn_unref(c->conn);
  }

  memset(c, 0, sizeof(*c));
}

static const struct ble_device_profile *find_profile(const bt_addr_le_t *addr)
{
  for (int p = 0; p < profile_count; p++)
  {
    for (int i = 0; i < profiles[p].mac_count; i++)
    {
      if (bt_addr_le_cmp(&profiles[p].macs[i], addr) == 0)
      {
        return &profiles[p];
      }
    }
  }

  return NULL;
}

static int find_profile_idx(const bt_addr_le_t *addr)
{
  const struct ble_device_profile *p = find_profile(addr);

  if (!p)
  {
    return -1;
  }

  return (int)(p - profiles);
}

static uint8_t notify_cb(struct bt_conn *conn,
                         struct bt_gatt_subscribe_params *params,
                         const void *data,
                         uint16_t len)
{
  struct central_conn *c = find_conn_by_sub_params(params);

  if (!c)
  {
    return BT_GATT_ITER_STOP;
  }

  if (!data)
  {
    c->subscribed = false;
    c->discovering = false;
    params->value_handle = 0U;
    LOG_INF("Unsubscribed");
    return BT_GATT_ITER_STOP;
  }

  const struct ble_device_profile *prof = &profiles[c->profile_idx];

  if (prof->data_cb)
  {
    prof->data_cb(&c->addr, (const uint8_t *)data, len);
  }

  return BT_GATT_ITER_CONTINUE;
}

#if BLE_EXCHANGE_ATT_MTU
static struct central_conn *find_conn_by_mtu_params(struct bt_gatt_exchange_params *params)
{
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (&conns[i].mtu_params == params)
    {
      return &conns[i];
    }
  }

  return NULL;
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
  struct central_conn *c = find_conn_by_mtu_params(params);

  if (!c)
  {
    return;
  }

  c->mtu_exchange_pending = false;
  c->mtu_ready = bt_gatt_get_mtu(conn) >= BLE_REQUIRED_ATT_MTU;
  if (err)
  {
    LOG_WRN("MTU exchange failed (err 0x%02x), using MTU %u",
            err,
            bt_gatt_get_mtu(conn));
  }
  else
  {
    LOG_INF("MTU exchange complete, MTU %u", bt_gatt_get_mtu(conn));
  }

  start_next_discovery();
}
#endif

static void request_fast_conn_params(struct bt_conn *conn)
{
  int err;
  const struct bt_le_conn_param fast_param = {
    .interval_min = BLE_CONN_INTERVAL_MIN,
    .interval_max = BLE_CONN_INTERVAL_MAX,
    .latency = BLE_CONN_LATENCY,
    .timeout = BLE_CONN_TIMEOUT,
  };

  err = bt_conn_le_param_update(conn, &fast_param);
  if (err && err != -EALREADY)
  {
    LOG_WRN("Connection parameter update request failed (err %d)", err);
  }
}

static void request_high_throughput_params(struct bt_conn *conn, struct central_conn *c)
{
  int err;

  c->phy_2m_ready = false;
  c->data_len_ready = false;
  c->mtu_ready = bt_gatt_get_mtu(conn) >= BLE_REQUIRED_ATT_MTU;

  request_fast_conn_params(conn);

  err = bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
  if (err)
  {
    LOG_WRN("2M PHY update request failed (err %d)", err);
  }

  err = bt_conn_le_data_len_update(conn, BT_LE_DATA_LEN_PARAM_MAX);
  if (err)
  {
    LOG_WRN("DLE update request failed (err %d)", err);
  }

#if BLE_EXCHANGE_ATT_MTU
  c->mtu_exchange_pending = true;
  c->mtu_params.func = mtu_exchange_cb;
  err = bt_gatt_exchange_mtu(conn, &c->mtu_params);
  if (err)
  {
    c->mtu_exchange_pending = false;
    c->mtu_ready = bt_gatt_get_mtu(conn) >= BLE_REQUIRED_ATT_MTU;
    if (err == -EALREADY)
    {
      LOG_INF("MTU already exchanged, MTU %u", bt_gatt_get_mtu(conn));
    }
    else
    {
      LOG_WRN("MTU exchange request failed (err %d), using MTU %u",
            err,
            bt_gatt_get_mtu(conn));
    }
  }
#else
  c->mtu_exchange_pending = false;
  c->mtu_ready = bt_gatt_get_mtu(conn) >= BLE_REQUIRED_ATT_MTU;
  LOG_INF("MTU exchange skipped, MTU %u", bt_gatt_get_mtu(conn));
#endif
}

static bool central_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
  bool can_use_fast = param->interval_min <= BLE_CONN_INTERVAL_MIN &&
                      param->interval_max >= BLE_CONN_INTERVAL_MAX &&
                      param->latency == BLE_CONN_LATENCY;
  bool can_use_fallback = param->interval_min <= BLE_CONN_INTERVAL_FALLBACK &&
                          param->interval_max >= BLE_CONN_INTERVAL_FALLBACK;

  if (can_use_fast)
  {
    param->interval_min = BLE_CONN_INTERVAL_MIN;
    param->interval_max = BLE_CONN_INTERVAL_MAX;
    param->latency = BLE_CONN_LATENCY;
    param->timeout = BLE_CONN_TIMEOUT;
    LOG_INF("Peer params allow fast CI, using 7.5 ms for %02X:%02X",
            bt_conn_get_dst(conn)->a.val[5],
            bt_conn_get_dst(conn)->a.val[4]);
    return true;
  }

  if (can_use_fallback)
  {
    uint16_t peer_min = param->interval_min;
    uint16_t peer_max = param->interval_max;
    uint16_t peer_latency = param->latency;
    uint16_t peer_timeout = param->timeout;

    param->interval_min = BLE_CONN_INTERVAL_FALLBACK;
    param->interval_max = BLE_CONN_INTERVAL_FALLBACK;
    param->latency = BLE_CONN_LATENCY;
    param->timeout = BLE_CONN_TIMEOUT;
    LOG_WRN("Peer params exclude 7.5 ms, try 15 ms lat0 for %02X:%02X peer min=%u max=%u lat=%u to=%u",
            bt_conn_get_dst(conn)->a.val[5],
            bt_conn_get_dst(conn)->a.val[4],
            peer_min,
            peer_max,
            peer_latency,
            peer_timeout);
    return true;
  }

  LOG_WRN("Accept peer params %02X:%02X min=%u max=%u lat=%u to=%u",
          bt_conn_get_dst(conn)->a.val[5],
          bt_conn_get_dst(conn)->a.val[4],
          param->interval_min,
          param->interval_max,
          param->latency,
          param->timeout);
  return true;
}

static void central_param_updated(struct bt_conn *conn,
                                  uint16_t interval,
                                  uint16_t latency,
                                  uint16_t timeout)
{
  struct central_conn *c = find_conn(bt_conn_get_dst(conn));
  uint32_t interval_us = interval * 1250U;

  if (c)
  {
    c->interval = interval;
    c->latency = latency;
    c->timeout = timeout;
  }

  LOG_INF("Conn params %02X:%02X interval=%u.%03u ms lat=%u to=%u ms",
          bt_conn_get_dst(conn)->a.val[5],
          bt_conn_get_dst(conn)->a.val[4],
          interval_us / 1000U,
          interval_us % 1000U,
          latency,
          timeout * 10U);

  /* Re-request fast CI if peer chose a slow one */
  if (c && interval > 20U && c->ci_retry_count < 3U)
  {
    c->ci_retry_count++;
    struct bt_le_conn_param fast = {
      .interval_min = 6, .interval_max = 6,
      .latency = 0, .timeout = BT_GAP_MS_TO_CONN_TIMEOUT(4000),
    };
    int err = bt_conn_le_param_update(conn, &fast);
    if (!err)
    {
      LOG_INF("CI re-request %u/3: 7.5ms", c->ci_retry_count);
    }
  }
}

static void subscribe_complete_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params)
{
  struct central_conn *c = find_conn_by_sub_params(params);

  if (!c)
  {
    return;
  }

  if (err)
  {
    c->discovering = false;
    LOG_WRN("Subscribe failed (ATT err 0x%02x)", err);
    peer_mark_failure(&c->addr);
    (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
  }
  else
  {
    c->subscribed = true;
    c->discovering = false;
    LOG_INF("Subscribed to notifications");

    if (profiles[c->profile_idx].post_subscribe_cb)
    {
      profiles[c->profile_idx].post_subscribe_cb(conn, c->write_handle);
    }

    start_next_discovery();
  }
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
  int err;
  struct central_conn *c = find_conn(bt_conn_get_dst(conn));

  if (!c)
  {
    return BT_GATT_ITER_STOP;
  }

  const struct ble_device_profile *prof = &profiles[c->profile_idx];

  if (!attr)
  {
    LOG_WRN("GATT discovery ended before subscription");
    memset(params, 0, sizeof(*params));
    c->discovering = false;
    if (!c->subscribed)
    {
      peer_mark_failure(&c->addr);
      (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
    resume_scan_if_needed();
    return BT_GATT_ITER_STOP;
  }

  switch (params->type)
  {
  case BT_GATT_DISCOVER_PRIMARY:
  {
    const struct bt_gatt_service_val *svc = attr->user_data;

    c->service_end_handle = svc ? svc->end_handle : BT_ATT_LAST_ATTRIBUTE_HANDLE;

    memset(params, 0, sizeof(*params));
    params->uuid = prof->chrc_uuid;
    params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
    params->func = discover_func;
    params->start_handle = attr->handle + 1;
    params->end_handle = c->service_end_handle;

    LOG_INF("Found service, discovering characteristic");
    err = bt_gatt_discover(conn, params);
    if (err)
    {
      c->discovering = false;
      LOG_ERR("Characteristic discovery failed (err %d)", err);
      peer_mark_failure(&c->addr);
      (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      resume_scan_if_needed();
    }
    break;
  }

  case BT_GATT_DISCOVER_CHARACTERISTIC:
  {
    const struct bt_gatt_chrc *chrc = attr->user_data;

    if (!chrc)
    {
      c->discovering = false;
      LOG_WRN("Characteristic discovery returned no user data");
      peer_mark_failure(&c->addr);
      (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      resume_scan_if_needed();
      break;
    }

    if (!(chrc->properties & BT_GATT_CHRC_NOTIFY))
    {
      c->discovering = false;
      LOG_WRN("Characteristic does not support notify");
      peer_mark_failure(&c->addr);
      (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      resume_scan_if_needed();
      break;
    }

    c->value_handle = chrc->value_handle;
    c->write_handle = prof->write_chrc_uuid ? chrc->value_handle + 2 : 0;

    memset(params, 0, sizeof(*params));
    params->uuid = BT_UUID_GATT_CCC;
    params->type = BT_GATT_DISCOVER_DESCRIPTOR;
    params->func = discover_func;
    params->start_handle = chrc->value_handle + 1;
    params->end_handle = c->service_end_handle;

    LOG_INF("Found notify characteristic, discovering CCC");
    err = bt_gatt_discover(conn, params);
    if (err)
    {
      c->discovering = false;
      LOG_ERR("CCC discovery failed (err %d)", err);
      peer_mark_failure(&c->addr);
      (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      resume_scan_if_needed();
    }
    break;
  }

  case BT_GATT_DISCOVER_DESCRIPTOR:
    memset(&c->sub_params, 0, sizeof(c->sub_params));
    c->sub_params.notify = notify_cb;
    c->sub_params.subscribe = subscribe_complete_cb;
    c->sub_params.value = BT_GATT_CCC_NOTIFY;
    c->sub_params.value_handle = c->value_handle;
    c->sub_params.ccc_handle = attr->handle;
    atomic_set_bit(c->sub_params.flags, BT_GATT_SUBSCRIBE_FLAG_VOLATILE);

    LOG_INF("Found CCC handle %u, subscribing", attr->handle);
    err = bt_gatt_subscribe(conn, &c->sub_params);
    if (err && err != -EALREADY)
    {
      c->discovering = false;
      LOG_ERR("bt_gatt_subscribe failed (err %d)", err);
      peer_mark_failure(&c->addr);
      (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
      resume_scan_if_needed();
    }
    else if (err == -EALREADY)
    {
      c->subscribed = true;
      c->discovering = false;
      start_next_discovery();
    }
    break;

  default:
    break;
  }

  return BT_GATT_ITER_STOP;
}

static void gatt_discover_start(struct bt_conn *conn, struct central_conn *c)
{
  const struct ble_device_profile *prof = &profiles[c->profile_idx];

  c->discovering = true;
  memset(&c->discover_params, 0, sizeof(c->discover_params));
  c->discover_params.uuid = prof->svc_uuid;
  c->discover_params.type = BT_GATT_DISCOVER_PRIMARY;
  c->discover_params.func = discover_func;
  c->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
  c->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;

  int err = bt_gatt_discover(conn, &c->discover_params);
  if (err)
  {
    c->discovering = false;
    LOG_ERR("GATT discovery failed (err %d)", err);
    peer_mark_failure(&c->addr);
    (void)bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    resume_scan_if_needed();
  }
  else
  {
    LOG_INF("GATT discovery started");
  }
}

static void start_next_discovery(void)
{
  if (!all_targets_connected())
  {
    resume_scan_after_settle();
    return;
  }

  if (any_discovery_running())
  {
    return;
  }

  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (conns[i].in_use && conns[i].connected && !conns[i].subscribed &&
        !conns[i].mtu_exchange_pending)
    {
      gatt_discover_start(conns[i].conn, &conns[i]);
      return;
    }
  }

  LOG_INF("All connected sensors subscribed");
}

static void central_connected(struct bt_conn *conn, uint8_t err)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  const bt_addr_le_t *addr = bt_conn_get_dst(conn);
  struct central_conn *c;

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

  c = find_conn(addr);

  if (err)
  {
    LOG_ERR("Connection failed: %s (err %d)", addr_str, err);
    peer_mark_failure(addr);
    clear_conn(c);
    resume_scan();
    return;
  }

  if (!c)
  {
    LOG_WRN("Unknown device %s, disconnecting", addr_str);
    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    return;
  }

  if (!c->conn)
  {
    c->conn = bt_conn_ref(conn);
  }

  c->connected = true;
  peer_mark_success(addr);
  LOG_INF("Connected: %s (%u/%u)", addr_str, connected_count(), target_count);
  request_high_throughput_params(conn, c);
  start_next_discovery();
}

static void central_disconnected(struct bt_conn *conn, uint8_t reason)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  const bt_addr_le_t *addr = bt_conn_get_dst(conn);
  struct central_conn *c = find_conn(addr);

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
  LOG_INF("Disconnected: %s (reason %d)", addr_str, reason);

  clear_conn(c);
  resume_scan();
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

static void central_phy_updated(struct bt_conn *conn,
                                struct bt_conn_le_phy_info *param)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  struct central_conn *c = find_conn(bt_conn_get_dst(conn));

  if (c)
  {
    c->phy_2m_ready = param->tx_phy == BT_GAP_LE_PHY_2M &&
                      param->rx_phy == BT_GAP_LE_PHY_2M;
  }

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
  LOG_INF("PHY updated: %s TX=%s RX=%s",
          addr_str,
          phy_to_str(param->tx_phy),
          phy_to_str(param->rx_phy));
}
#endif

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void central_data_len_updated(struct bt_conn *conn,
                                     struct bt_conn_le_data_len_info *info)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  struct central_conn *c = find_conn(bt_conn_get_dst(conn));

  if (c)
  {
    c->data_len_ready = info->tx_max_len >= BLE_REQUIRED_DATA_LEN &&
                        info->rx_max_len >= BLE_REQUIRED_DATA_LEN;
  }

  bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
  LOG_INF("Data length updated: %s TX=%u/%uus RX=%u/%uus",
          addr_str,
          info->tx_max_len,
          info->tx_max_time,
          info->rx_max_len,
          info->rx_max_time);
}
#endif

static void connect_sensor(const bt_addr_le_t *addr, int8_t rssi)
{
  char addr_str[BT_ADDR_LE_STR_LEN];
  struct central_conn *c;
  struct bt_conn *existing;
  struct bt_conn *conn = NULL;
  int pi;
  int err;

  pi = find_profile_idx(addr);
  if (pi < 0 || find_conn(addr))
  {
    return;
  }

  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

  if (!peer_retry_ready(addr))
  {
    LOG_DBG("Skipping %s in backoff (%d ms remaining)",
            addr_str,
            peer_retry_remaining_ms(addr));
    return;
  }

  existing = bt_conn_lookup_addr_le(BT_ID_DEFAULT, addr);
  if (existing)
  {
    bt_conn_unref(existing);
    return;
  }

  c = alloc_conn();
  if (!c)
  {
    LOG_WRN("Connection table full");
    return;
  }

  bt_addr_le_copy(&c->addr, addr);
  c->profile_idx = (uint8_t)pi;

  LOG_DBG("Sensor found: %s RSSI=%d", addr_str, rssi);

  err = bt_scan_stop();
  if (err && err != -EALREADY)
  {
    LOG_ERR("bt_scan_stop failed (%d)", err);
    clear_conn(c);
    return;
  }

  const struct bt_conn_le_create_param create_param = {
    .options = BT_CONN_LE_OPT_NONE,
    .interval = BLE_CREATE_SCAN_INTERVAL,
    .window = BLE_CREATE_SCAN_WINDOW,
    .interval_coded = 0,
    .window_coded = 0,
    .timeout = BT_GAP_MS_TO_CONN_TIMEOUT(BLE_CREATE_TIMEOUT_MS),
  };
  const struct bt_le_conn_param conn_param = {
    .interval_min = BLE_CONN_INTERVAL_MIN,
    .interval_max = BLE_CONN_INTERVAL_MAX,
    .latency = BLE_CONN_LATENCY,
    .timeout = BLE_CONN_TIMEOUT,
  };

  err = bt_conn_le_create(addr, &create_param, &conn_param, &conn);
  if (err)
  {
    LOG_ERR("Manual connect failed (%d)", err);
    peer_mark_failure(addr);
    clear_conn(c);
    resume_scan();
    return;
  }

  c->conn = conn;
  LOG_INF("Connection initiated: %s", addr_str);
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
                              struct bt_scan_filter_match *filter_match,
                              bool connectable)
{
  ARG_UNUSED(filter_match);

  if (!connectable)
  {
    return;
  }

  connect_sensor(device_info->recv_info->addr, device_info->recv_info->rssi);
}

static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
  char addr_str[BT_ADDR_LE_STR_LEN];

  bt_addr_le_to_str(device_info->recv_info->addr, addr_str, sizeof(addr_str));
  LOG_WRN("Connection attempt failed: %s", addr_str);
}

static void scan_connecting(struct bt_scan_device_info *device_info,
                            struct bt_conn *conn)
{
  ARG_UNUSED(device_info);
  ARG_UNUSED(conn);
}

BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, scan_connecting_error, scan_connecting);

int bsp_ble_central_start(const struct ble_device_profile *p, uint8_t count)
{
  int total = 0;
  int peer_idx = 0;
  int err;

  if (p == NULL || count == 0 || count > CONFIG_BT_MAX_CONN)
  {
    return -EINVAL;
  }

  if (started)
  {
    return -EALREADY;
  }

  for (int i = 0; i < count; i++)
  {
    total += p[i].mac_count;
  }

  if (total == 0 || total > CONFIG_BT_MAX_CONN)
  {
    LOG_ERR("Invalid sensor count %d, CONFIG_BT_MAX_CONN=%d",
            total,
            CONFIG_BT_MAX_CONN);
    return -EINVAL;
  }

#if defined(CONFIG_BT_SCAN_ADDRESS_CNT)
  if (total > CONFIG_BT_SCAN_ADDRESS_CNT)
  {
    LOG_ERR("Invalid sensor count %d, CONFIG_BT_SCAN_ADDRESS_CNT=%d",
            total,
            CONFIG_BT_SCAN_ADDRESS_CNT);
    return -EINVAL;
  }
#endif

  profiles = p;
  profile_count = count;
  target_count = (uint8_t)total;
  memset(peer_states, 0, sizeof(peer_states));
  memset(conns, 0, sizeof(conns));

  static struct bt_conn_cb conn_cb = {
    .connected = central_connected,
    .disconnected = central_disconnected,
    .le_param_req = central_param_req,
    .le_param_updated = central_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
    .le_phy_updated = central_phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
    .le_data_len_updated = central_data_len_updated,
#endif
  };
  bt_conn_cb_register(&conn_cb);

  struct bt_scan_init_param scan_init = {
    .scan_param = NULL,
    .connect_if_match = false,
    .conn_param = NULL,
  };
  bt_scan_init(&scan_init);
  bt_scan_cb_register(&scan_cb);
  bt_scan_filter_remove_all();

  for (int i = 0; i < count; i++)
  {
    for (int j = 0; j < p[i].mac_count; j++)
    {
      err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_ADDR, &p[i].macs[j]);
      if (err)
      {
        LOG_ERR("bt_scan_filter_add failed (err %d)", err);
        return err;
      }

      bt_addr_le_copy(&peer_states[peer_idx].addr, &p[i].macs[j]);
      peer_states[peer_idx].in_use = true;
      peer_idx++;
    }
  }

  err = bt_scan_filter_enable(BT_SCAN_ADDR_FILTER, false);
  if (err)
  {
    LOG_ERR("bt_scan_filter_enable failed (err %d)", err);
    return err;
  }

  err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
  if (err)
  {
    LOG_ERR("bt_scan_start failed (err %d)", err);
    return err;
  }

  LOG_INF("Scanning for %d sensors across %d profiles", total, count);
  started = true;
  return 0;
}

bool bsp_ble_central_all_subscribed(void)
{
  if (target_count == 0) return false;

  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (!conns[i].in_use || !conns[i].connected) continue;
    if (!conns[i].subscribed) return false;
  }

  return connected_count() >= target_count;
}

bool bsp_ble_central_all_links_ready(void)
{
  if (target_count == 0 || connected_count() < target_count)
  {
    return false;
  }

  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++)
  {
    if (!conns[i].in_use || !conns[i].connected)
    {
      continue;
    }

    if (!conns[i].subscribed ||
        !conns[i].phy_2m_ready ||
        !conns[i].data_len_ready ||
        !conns[i].mtu_ready)
    {
      return false;
    }
  }

  return true;
}

#endif /* CONFIG_BT_CENTRAL && CONFIG_BT */
