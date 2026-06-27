#ifndef BSP_BLE_PERIPH_H
#define BSP_BLE_PERIPH_H

#include <errno.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if CONFIG_BT && CONFIG_BT_PERIPHERAL && CONFIG_BT_NUS

#include "bsp_ble.h"

int bsp_ble_periph_init(void);

/** BLE RX callbacks (NUS data path) */
void bsp_ble_rx_cb_register(ble_rx_cb_t cb);
void bsp_ble_rx_conn_cb_register(ble_rx_conn_cb_t cb);

/** Send data to a specific NUS connection (MTU-chunked). */
int ble_nus_send_to(struct bt_conn *conn, const uint8_t *data, uint16_t len);

/** Query NUS connection list. */
struct bt_conn *ble_nus_get_conn(uint8_t idx);
uint8_t ble_nus_get_conn_count(void);

#else

static inline int bsp_ble_periph_init(void) { return -ENOTSUP; }

#endif

#ifdef __cplusplus
}
#endif

#endif
