#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stddef.h>
#include <stdint.h>

#define BSP_UART_BUF_SIZE 128

#ifdef __cplusplus
extern "C"
{
#endif

/** Callback type for UART RX: invoked with received data + length. */
typedef void (*uart_rx_cb_t)(uint8_t *data, size_t len);

/**
 * @brief Initialize the UART module (async UART with DMA, RX + TX threads).
 * @return 0 on success, negative error code on failure.
 */
int bsp_uart_init(void);

/**
 * @brief Register callback for received UART data.
 * @param cb  Callback (must not be NULL).
 * @return 0 on success.
 */
int bsp_uart_rx_cb_register(uart_rx_cb_t cb);

/**
 * @brief Send data via UART (non-blocking, copied into TX queue).
 * @return 0 on success, -EINVAL or -ENOMEM on error.
 */
int bsp_uart_tx(const uint8_t *data, size_t len);

/**
 * @brief Disable UART RX and put the peripheral to sleep.
 * @return 0 on success.
 */
int bsp_uart_sleep(void);

/**
 * @brief Wake up the UART from sleep and re-enable RX.
 * @return 0 on success.
 */
int bsp_uart_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_UART_H */
