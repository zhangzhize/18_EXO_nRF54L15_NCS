#if defined(CONFIG_BT) && (CONFIG_BT == 1)

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <bluetooth/services/nus.h>

#include "bsp_ble.h"
#include "app_uart.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bsp_ble, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

/** Advertising */
static struct k_work adv_work;
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};
static void adv_work_handler(struct k_work *work)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Advertising successfully started");
}
static void advertising_start(void)
{
	k_work_submit(&adv_work);
}

/** Connection and Update */
static struct bt_conn *current_conn;
static bool nus_notify_enabled;
static struct k_work_delayable conn_update_work;
static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("ATT MTU updated: TX=%u RX=%u", tx, rx);
}
static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated,
};
static void conn_update_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	struct bt_conn *conn = current_conn;
	int err;

	if (!conn) {
		return;
	}

	/* Request shortest Connection Interval: 7.5 ms = 6 * 1.25 ms. */
	struct bt_le_conn_param param = {
		.interval_min = 6,
		.interval_max = 6,
		.latency = 0,
		.timeout = 400, /* 4 s supervision timeout (units of 10 ms) */
	};

	err = bt_conn_le_param_update(conn, &param);
	if (err) {
		LOG_WRN("bt_conn_le_param_update() failed (err %d)", err);
	} else {
		LOG_INF("Conn param update requested: interval=7.5ms latency=0 timeout=4s");
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	current_conn = bt_conn_ref(conn);
	nus_notify_enabled = false;
	k_work_schedule(&conn_update_work, K_MSEC(100));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

	if (current_conn) {
		k_work_cancel_delayable(&conn_update_work);
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	nus_notify_enabled = false;
}

static void recycled_cb(void)
{
	LOG_INF("Connection object available from previous conn. Disconnect is complete!");
	advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
	.recycled         = recycled_cb,
};

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	app_uart_tx(data, len);
}

static void nus_send_enabled_cb(enum bt_nus_send_status status)
{
	nus_notify_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	LOG_INF("NUS notify %s", nus_notify_enabled ? "enabled" : "disabled");
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = NULL,
	.send_enabled = nus_send_enabled_cb,
};


int bsp_ble_init(void)
{
    int err;
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Failed to enable Bluetooth (err: %d)", err);
		return -1;
	}

    bt_gatt_cb_register(&gatt_callbacks);

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return -1;
	}

	k_work_init(&adv_work, adv_work_handler);
    k_work_init_delayable(&conn_update_work, conn_update_work_handler);

    advertising_start();

    app_uart_rx_cb_register(ble_uart_rx_cb);
    
    return 0;
}

void ble_uart_rx_cb(uint8_t *byte, size_t len)
{
	if (byte == NULL || len == 0) {
		return;
	}
	if (current_conn == NULL || !nus_notify_enabled) {
		return;
	}

	int err = bt_nus_send(current_conn, byte, len);
	if (err) {
		/* Busy path: avoid logs unless debugging. */
		LOG_DBG("bt_nus_send failed: %d", err);
	}
}

#endif /* CONFIG_BT */