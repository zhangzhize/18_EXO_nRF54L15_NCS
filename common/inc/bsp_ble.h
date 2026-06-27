#ifndef BSP_BLE_H
#define BSP_BLE_H

#include <stdint.h>
#include <stddef.h>
#include <errno.h>

struct bt_conn;

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*ble_rx_cb_t)(const uint8_t *data, size_t len);
typedef void (*ble_rx_conn_cb_t)(struct bt_conn *conn, const uint8_t *data, size_t len);

#if CONFIG_BT

/** Initialize the Bluetooth stack. Must be called before any other BLE API. */
int bsp_ble_init(void);

#else

static inline int bsp_ble_init(void)
{
  return -ENOTSUP;
}

#endif

#ifdef __cplusplus
}
#endif

#endif
