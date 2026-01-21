#ifndef BSP_BLE_H
#define BSP_BLE_H

#include "main.h"
#include <stdint.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_BT) && (CONFIG_BT == 1)

int bsp_ble_init(void);
void ble_uart_rx_cb(uint8_t *byte, size_t len);

#else /* CONFIG_BT */

/* BLE 未启用时提供 stub，方便同一套代码按配置裁剪。 */
static inline int bsp_ble_init(void)
{
	return -ENOTSUP;
}

static inline void ble_uart_rx_cb(uint8_t *byte, size_t len)
{
	(void)byte;
	(void)len;
}

#endif /* CONFIG_BT */

#ifdef __cplusplus
}
#endif

#endif /* BSP_BLE_H */