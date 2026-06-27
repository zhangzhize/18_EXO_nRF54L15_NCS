#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <string.h>

#if IS_ENABLED(CONFIG_NRF_SYS_EVENT)
#include <nrf_sys_event.h>
#endif

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
#include <nrfx_power.h>
#endif

#include "bsp_uart.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bsp_uart, LOG_LEVEL_WRN);

#define UART_TX_THREAD_STACKSIZE 2048
#define UART_TX_THREAD_PRIORITY 6
#define UART_RX_THREAD_STACKSIZE 2048
#define UART_RX_THREAD_PRIORITY 4

#define BUF_SIZE BSP_UART_BUF_SIZE
#define RX_INACTIVE_TIMEOUT_US 200

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart21));

/* Memory slabs for DMA buffers */
#define RX_DRV_NUM 48
#define RX_COPY_NUM 128
#define TX_USER_NUM 8
K_MEM_SLAB_DEFINE(uart_rx_driver_slab, BUF_SIZE, RX_DRV_NUM, 4);
K_MEM_SLAB_DEFINE(uart_rx_copy_slab, BUF_SIZE, RX_COPY_NUM, 4);
K_MEM_SLAB_DEFINE(uart_tx_user_slab, BUF_SIZE, TX_USER_NUM, 4);

/* Queues for TX and RX packets */
struct uart_data_t
{
  uint8_t *data;
  size_t len;
};
K_MSGQ_DEFINE(rx_queue, sizeof(struct uart_data_t), 64, 4);
K_MSGQ_DEFINE(tx_queue, sizeof(struct uart_data_t), 8, 4);

static K_SEM_DEFINE(tx_done, 0, 1);

/* ── RX restart work ─────────────────────────────────────────────── */

static void uart_rx_restart_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(uart_rx_restart_work, uart_rx_restart_work_handler);

static void uart_rx_restart_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);

  /* RX is already stopped by the driver — just re-allocate and enable */

  uint8_t *buf = NULL;
  int err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
  if (err)
  {
    LOG_ERR("RX restart: no driver slab (%d)", err);
    (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(20));
    return;
  }

  err = uart_rx_enable(uart_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT_US);
  if (err)
  {
    LOG_ERR("RX restart: uart_rx_enable failed (%d)", err);
    k_mem_slab_free(&uart_rx_driver_slab, buf);
    (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(20));
    return;
  }

  LOG_DBG("RX restarted");
}

/* ── Async UART callback ─────────────────────────────────────────── */

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
  ARG_UNUSED(user_data);
  int err;

  switch (evt->type)
  {
  case UART_TX_DONE:
    LOG_DBG("TX done %d bytes", evt->data.tx.len);
    k_sem_give(&tx_done);
    break;

  case UART_TX_ABORTED:
    LOG_WRN("TX aborted");
    k_sem_give(&tx_done);
    break;

  case UART_RX_RDY:
  {
    uint8_t *p = &evt->data.rx.buf[evt->data.rx.offset];
    size_t len = evt->data.rx.len;

    if (len > BUF_SIZE)
    {
      break;
    }

    uint8_t *buf_copy = NULL;
    err = k_mem_slab_alloc(&uart_rx_copy_slab, (void **)&buf_copy, K_NO_WAIT);
    if (err)
    {
      LOG_ERR_RATELIMIT_RATE(200, "No RX copy slab (err %d)", err);
      break;
    }

    memcpy(buf_copy, p, len);

    struct uart_data_t packet = {.data = buf_copy, .len = len};

    err = k_msgq_put(&rx_queue, &packet, K_NO_WAIT);
    if (err)
    {
      LOG_ERR_RATELIMIT_RATE(200, "RX queue full (%d), freeing slab", err);
      k_mem_slab_free(&uart_rx_copy_slab, buf_copy);
    }
    break;
  }

  case UART_RX_BUF_REQUEST:
  {
    uint8_t *buf;
    LOG_DBG("RX buffer request");
    err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
    if (err)
    {
      LOG_ERR("RX BUF_REQUEST: no slab (%d)", err);
      break;
    }
    err = uart_rx_buf_rsp(dev, buf, BUF_SIZE);
    if (err)
    {
      LOG_ERR("RX BUF_REQUEST: uart_rx_buf_rsp failed (%d)", err);
      k_mem_slab_free(&uart_rx_driver_slab, buf);
    }
    break;
  }

  case UART_RX_BUF_RELEASED:
    LOG_DBG("RX buffer released");
    k_mem_slab_free(&uart_rx_driver_slab, (void *)evt->data.rx_buf.buf);
    break;

  case UART_RX_DISABLED:
    LOG_DBG("RX disabled");
    (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(10));
    break;

  case UART_RX_STOPPED:
    LOG_DBG("RX stopped, reason=%d", evt->data.rx_stop.reason);
    (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(10));
    break;

  default:
    LOG_WRN("Unknown UART event type %d", evt->type);
    break;
  }
}

/* ── User callback ───────────────────────────────────────────────── */

static uart_rx_cb_t user_callback;

int bsp_uart_rx_cb_register(uart_rx_cb_t cb)
{
  __ASSERT(cb != NULL, "Callback cannot be NULL");
  user_callback = cb;
  return 0;
}

/* ── RX thread ───────────────────────────────────────────────────── */

static void bsp_uart_rx_thread(void *p1, void *p2, void *p3)
{
  struct uart_data_t packet = {0};

  while (1)
  {
    int err = k_msgq_get(&rx_queue, &packet, K_FOREVER);
    if (err)
    {
      LOG_ERR_RATELIMIT_RATE(200, "RX queue get failed");
      continue;
    }

    if (user_callback)
    {
      user_callback(packet.data, packet.len);
    }
    k_mem_slab_free(&uart_rx_copy_slab, packet.data);
  }
}

K_THREAD_DEFINE(bsp_uart_rx_id, UART_RX_THREAD_STACKSIZE, bsp_uart_rx_thread, NULL, NULL, NULL, UART_RX_THREAD_PRIORITY, 0, 0);

/* ── TX ──────────────────────────────────────────────────────────── */

int bsp_uart_tx(const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0)
  {
    LOG_WRN("Invalid TX parameters");
    return -EINVAL;
  }
  if (len > BUF_SIZE)
  {
    LOG_ERR("TX length %d exceeds BUF_SIZE %d", (int)len, BUF_SIZE);
    return -EINVAL;
  }

  uint8_t *buf = NULL;
  int err = k_mem_slab_alloc(&uart_tx_user_slab, (void **)&buf, K_NO_WAIT);
  if (err)
  {
    LOG_ERR("TX slab alloc failed (%d)", err);
    return -ENOMEM;
  }

  memcpy(buf, data, len);

  struct uart_data_t packet = {.data = buf, .len = len};

  err = k_msgq_put(&tx_queue, &packet, K_NO_WAIT);
  if (err)
  {
    LOG_ERR("TX queue full, freeing slab");
    k_mem_slab_free(&uart_tx_user_slab, buf);
    return err;
  }

  return 0;
}

/* ── TX thread ───────────────────────────────────────────────────── */

static void bsp_uart_tx_thread(void *p1, void *p2, void *p3)
{
  while (1)
  {
    struct uart_data_t packet = {0};

    int err = k_msgq_get(&tx_queue, &packet, K_FOREVER);
    if (err)
    {
      LOG_ERR("TX queue get failed");
      continue;
    }

    err = uart_tx(uart_dev, packet.data, packet.len, 0);
    if (err)
    {
      LOG_ERR("uart_tx failed (%d)", err);
      k_mem_slab_free(&uart_tx_user_slab, packet.data);
      continue;
    }

    k_sem_take(&tx_done, K_FOREVER);
    k_mem_slab_free(&uart_tx_user_slab, packet.data);
  }
}

K_THREAD_DEFINE(bsp_uart_tx_id, UART_TX_THREAD_STACKSIZE, bsp_uart_tx_thread, NULL, NULL, NULL, UART_TX_THREAD_PRIORITY, 0, 0);

/* ── Init / sleep / wakeup ───────────────────────────────────────── */

int bsp_uart_init(void)
{
  if (!device_is_ready(uart_dev))
  {
    LOG_ERR("device %s is not ready", uart_dev->name);
    return -ENODEV;
  }

  int err = uart_callback_set(uart_dev, uart_callback, NULL);
  __ASSERT(err == 0, "Failed to set callback");

  uint8_t *buf;
  err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
  __ASSERT(err == 0, "Failed to alloc slab");

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
  nrfx_power_constlat_mode_request();
#endif

#if IS_ENABLED(CONFIG_NRF_SYS_EVENT)
  err = nrf_sys_event_request_global_constlat();
  if (err)
  {
    LOG_WRN("nrf_sys_event_request_global_constlat() failed (%d)", err);
  }
#endif

  err = uart_rx_enable(uart_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT_US);
  __ASSERT(err == 0, "Failed to enable rx");

  return 0;
}

int bsp_uart_sleep(void)
{
  int err = uart_rx_disable(uart_dev);
  if (err)
  {
    LOG_ERR("Failed to disable RX: %d", err);
    return err;
  }

#if !IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
  k_sleep(K_MSEC(10));
  err = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_SUSPEND);
  if (err)
  {
    LOG_ERR("Failed to suspend device: %d", err);
    return err;
  }
#endif

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
  nrfx_power_constlat_mode_free();
#endif

#if IS_ENABLED(CONFIG_NRF_SYS_EVENT)
  (void)nrf_sys_event_release_global_constlat();
#endif

  return 0;
}

int bsp_uart_wakeup(void)
{
#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
  nrfx_power_constlat_mode_request();
#endif

#if IS_ENABLED(CONFIG_NRF_SYS_EVENT)
  (void)nrf_sys_event_request_global_constlat();
#endif

#if !IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
  {
    int err = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_RESUME);
    if (err)
    {
      LOG_ERR("Failed to resume device: %d", err);
      return err;
    }
  }
#endif

  uint8_t *buf;
  int err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
  if (err)
  {
    LOG_ERR("Failed to allocate RX buffer: %d", err);
    return err;
  }

  err = uart_rx_enable(uart_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT_US);
  if (err)
  {
    LOG_ERR("Failed to enable RX: %d", err);
    k_mem_slab_free(&uart_rx_driver_slab, buf);
    return err;
  }
  return 0;
}
