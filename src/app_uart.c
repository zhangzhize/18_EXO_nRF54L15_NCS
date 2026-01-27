#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <string.h>

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
#include <nrfx_power.h>
#endif /* CONFIG_APP_UART_GPIO_CROSS_DOMAIN */

#include "app_uart.h"
#include "bsp_led.h"

#include <zephyr/logging/log.h>
/* Reduce verbosity to avoid RTT/log backend overflow during bursts */
LOG_MODULE_REGISTER(app_uart, LOG_LEVEL_INF);

#define UART_TX_THREAD_STACKSIZE 2048
#define UART_TX_THREAD_PRIORITY  6
#define UART_RX_THREAD_STACKSIZE 2048
#define UART_RX_THREAD_PRIORITY  4

/* Normal UART */
#define UART_INST DT_ALIAS(uart21)
const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart21));
#define RX_INACTIVE_TIMEOUT_US 200

/* uart rx/tx memory pools for DMA */
#define BUF_SIZE        128
#define RX_DRV_NUM      32
#define RX_COPY_NUM     64
#define TX_USER_NUM     8
#define TX_DRV_NUM      8
K_MEM_SLAB_DEFINE(uart_rx_driver_slab, BUF_SIZE, RX_DRV_NUM, 4);
K_MEM_SLAB_DEFINE(uart_rx_copy_slab, BUF_SIZE, RX_COPY_NUM, 4);
K_MEM_SLAB_DEFINE(uart_tx_user_slab, BUF_SIZE, TX_USER_NUM, 4);

/* Queues for TX and RX packet */
struct uart_data_t {
    uint8_t *data;
    size_t len;
};
K_MSGQ_DEFINE(rx_queue, sizeof(struct uart_data_t), 64, 4);
K_MSGQ_DEFINE(tx_queue, sizeof(struct uart_data_t), 8, 4);

/* TX semaphores */
static K_SEM_DEFINE(tx_done, 0, 1);

/* simple debug counters*/
static volatile int tx_user_alloc_attempts = 0;
static volatile int tx_user_alloc_fails = 0;
static volatile int tx_user_free_count = 0;
static volatile int tx_queue_put_fails = 0;
static volatile int tx_queue_put_success = 0;
static volatile int uart_tx_start_count = 0;
static volatile int uart_tx_error_count = 0;
static volatile int uart_tx_done_count = 0;

static volatile int rx_copy_alloc_attempts = 0;
static volatile int rx_copy_alloc_fails = 0;
static volatile int rx_copy_free_count = 0;
static volatile int rx_queue_put_fails = 0;
static volatile int rx_queue_put_success = 0;
static volatile int uart_rx_rdys = 0;
static volatile int uart_rx_buf_request_count = 0;
static volatile int uart_rx_buf_released_count = 0;

static void uart_rx_restart_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(uart_rx_restart_work, uart_rx_restart_work_handler);

static void uart_rx_restart_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* 确保 RX 关闭再开 */
    (void)uart_rx_disable(uart_dev);

    uint8_t *buf = NULL;
    int err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
    if (err) {
        LOG_ERR("RX restart: no driver slab (%d)", err);
        /* 稍后再试 */
        (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(20));
        return;
    }

    err = uart_rx_enable(uart_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT_US);
    if (err) {
        LOG_ERR("RX restart: uart_rx_enable failed (%d)", err);
        k_mem_slab_free(&uart_rx_driver_slab, buf);
        /* 稍后再试 */
        (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(20));
        return;
    }

    LOG_WRN("RX restarted");
}



/* async serial callback */
static void uart_callback(const struct device *dev,
			  struct uart_event *evt,
			  void *user_data)
{
	struct device *uart = user_data;
	int err;

	switch (evt->type) {
	case UART_TX_DONE:
        LOG_INF("TX done %d bytes", evt->data.tx.len);
        uart_tx_done_count++;
        k_sem_give(&tx_done);
		break;

	case UART_TX_ABORTED:
        LOG_WRN("TX aborted");
        k_sem_give(&tx_done);
		break;

	case UART_RX_RDY:
    {
        static int cnt = 0;
        if (++cnt >= 500) {
            cnt = 0;
            bsp_led_toggle();
        }

        uint8_t *p = &(evt->data.rx.buf[evt->data.rx.offset]);
        size_t len = evt->data.rx.len;

        uart_rx_rdys++;

        if (len > BUF_SIZE) {
            break;
        }

        uint8_t *buf_copy = NULL;
        rx_copy_alloc_attempts++;
        int aerr = k_mem_slab_alloc(&uart_rx_copy_slab, (void **)&buf_copy, K_NO_WAIT);
        if (aerr) {
            rx_copy_alloc_fails++;
            LOG_ERR_RATELIMIT_RATE(200, "No available RX copy slab to copy RX data (err %d)", aerr);
            break;
        }

        memcpy(buf_copy, p, len);

        struct uart_data_t packet = {
            .data = buf_copy,
            .len = len,
        };

        err = k_msgq_put(&rx_queue, &packet, K_NO_WAIT);
        if (err) {
            rx_queue_put_fails++;
            LOG_ERR_RATELIMIT_RATE(200, "Failed to put packet to RX queue (%d), freeing slab", err);
            k_mem_slab_free(&uart_rx_copy_slab, buf_copy);
            rx_copy_free_count++;
        } else {
            rx_queue_put_success++;
        }
        break;
    }

	case UART_RX_BUF_REQUEST:
	{
		uint8_t *buf;
        uart_rx_buf_request_count++;
        LOG_INF("RX buffer request");
		err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
		// __ASSERT(err == 0, "Failed to allocate slab\n");
        if (err) {
            LOG_ERR("RX BUF_REQUEST: no slab (%d)", err);
            break;
        }

		err = uart_rx_buf_rsp(uart, buf, BUF_SIZE);
		// __ASSERT(err == 0, "Failed to provide new buffer\n");
        if (err) {
            LOG_ERR("RX BUF_REQUEST: uart_rx_buf_rsp failed (%d)", err);
            k_mem_slab_free(&uart_rx_driver_slab, buf);
            break;
        }

		break;
	}

	case UART_RX_BUF_RELEASED:
    {
        uart_rx_buf_released_count++;
        LOG_INF("RX buffer released");
        k_mem_slab_free(&uart_rx_driver_slab, (void *)evt->data.rx_buf.buf);
        break;
    }

	case UART_RX_DISABLED:
        LOG_WRN("RX disabled");
		break;

	case UART_RX_STOPPED:
        LOG_ERR("RX stopped, reason=%d", evt->data.rx_stop.reason);
        (void)k_work_reschedule(&uart_rx_restart_work, K_MSEC(10));
		break;

    default:
        LOG_WRN("Unknown UART event type %d", evt->type);
        break;
	}
}

static packets_cb_t user_callback = NULL;

int app_uart_rx_cb_register(packets_cb_t cb)
{
    __ASSERT(cb != NULL, "Callback cannot be NULL");
    user_callback = cb;
    return 0;
}

static void app_uart_rx_thread()
{
    struct uart_data_t packet = {0};
    int err;
    while(1) {
        err = k_msgq_get(&rx_queue, &packet, K_FOREVER);
        if (err) {
            LOG_ERR_RATELIMIT_RATE(200, "Failed to get packet from RX queue");
            continue;
        }

        if (user_callback == NULL) {
            k_mem_slab_free(&uart_rx_copy_slab, packet.data);
            rx_copy_free_count++;
            continue;
        }
        user_callback(packet.data, packet.len);
        k_mem_slab_free(&uart_rx_copy_slab, packet.data);
        rx_copy_free_count++;
    }
}

int app_uart_tx(const uint8_t *byte, size_t len)
{
    if (byte == NULL || len == 0) {
        LOG_WRN("Invalid TX parameters");
        return -EINVAL;
    }

    if (len > BUF_SIZE) {
        LOG_ERR("TX length %d exceeds BUF_SIZE %d", (int)len, BUF_SIZE);
        return -EINVAL;
    }

    uint8_t *buf = NULL;
    tx_user_alloc_attempts++;
    int aerr = k_mem_slab_alloc(&uart_tx_user_slab, (void **)&buf, K_NO_WAIT);
    if (aerr) {
        tx_user_alloc_fails++;
        LOG_ERR("Failed to alloc TX slab (%d)", aerr);
        return -ENOMEM;
    }

    memcpy(buf, byte, len);

    struct uart_data_t packet = {
        .data = buf,
        .len = len,
    };

    int err = k_msgq_put(&tx_queue, &packet, K_NO_WAIT);

    if (err) {
        tx_queue_put_fails++;
        LOG_ERR("Failed to put packet to TX queue, freeing slab");
        k_mem_slab_free(&uart_tx_user_slab, buf);
        return err;
    }
    tx_queue_put_success++;

    return 0;
}

int app_uart_tx_malloc(const uint8_t *byte, size_t len)
{
    ARG_UNUSED(byte);
    ARG_UNUSED(len);
    /* Deprecated: keep symbol for compatibility but force slab-only usage. */
    return -ENOTSUP;
}

void app_uart_tx_thread()
{
    while(1) {
        struct uart_data_t packet = {0};
        int err;
        err = k_msgq_get(&tx_queue, &packet, K_FOREVER);

        if (err) {
            LOG_ERR("Failed to get packet from TX queue");
            continue;
        }

        err = uart_tx(uart_dev, packet.data, packet.len, 0);
        if (err) {
            uart_tx_error_count++;
            LOG_ERR("Failed to send tx data");
            k_mem_slab_free(&uart_tx_user_slab, packet.data);
            continue;
        }
        uart_tx_start_count++;

        // wait for TX done
        k_sem_take(&tx_done, K_FOREVER);
        tx_user_free_count++;
        k_mem_slab_free(&uart_tx_user_slab, packet.data);
    }
}


int app_uart_init(void)
{
    int err;
    uint8_t *buf;

    if (!device_is_ready(uart_dev)) {
        LOG_ERR("device %s is not ready; exiting", uart_dev->name);
        return -ENODEV;
    }

    err = uart_callback_set(uart_dev, uart_callback, (void *)uart_dev);
    __ASSERT(err == 0, "Failed to set callback");

    // allocate buffer and start rx (driver slab)
    err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
    __ASSERT(err == 0, "Failed to alloc slab");

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
    nrfx_power_constlat_mode_request();
#endif /* CONFIG_APP_UART_GPIO_CROSS_DOMAIN */

    err = uart_rx_enable(uart_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT_US);
    __ASSERT(err == 0, "Failed to enable rx");

    return 0;
}

int app_uart_sleep(void)
{
    int err;
    err = uart_rx_disable(uart_dev);
    if (err) {
        LOG_ERR("Failed to disable RX: %d", err);
        return err;
    }

#if !IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    // give some time for UART callback
    k_sleep(K_MSEC(10)); 
    err = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_SUSPEND);
    if (err) {
        LOG_ERR("Failed to suspend device: %d", err);
        return err;
    }
#endif /* !CONFIG_PM_DEVICE_RUNTIME */ 

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
    nrfx_power_constlat_mode_free();
#endif /* CONFIG_APP_UART_GPIO_CROSS_DOMAIN */

    return 0;
}

int app_uart_wakeup(void)
{
    uint8_t *buf;
    int err;

#if IS_ENABLED(CONFIG_APP_UART_GPIO_CROSS_DOMAIN)
    nrfx_power_constlat_mode_request();
#endif /* CONFIG_APP_UART_GPIO_CROSS_DOMAIN */

#if !IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    err = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_RESUME);
    if (err) {
        LOG_ERR("Failed to resume device: %d", err);
        return err;
    }
#endif /* !CONFIG_PM_DEVICE_RUNTIME */

    err = k_mem_slab_alloc(&uart_rx_driver_slab, (void **)&buf, K_NO_WAIT);
    if (err) {
        LOG_ERR("Failed to allocate RX buffer: %d", err);
        return err;
    }

    err = uart_rx_enable(uart_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT_US);
    if (err) {
        LOG_ERR("Failed to enable RX: %d", err);
        k_mem_slab_free(&uart_rx_driver_slab, buf);
        return err;
    }
    return 0;
}

void app_uart_print_stats(void)
{
    int tx_alloc_attempts = tx_user_alloc_attempts;
    int tx_alloc_fails = tx_user_alloc_fails;
    int tx_free_count = tx_user_free_count;
    int q_put_fails = tx_queue_put_fails;
    int q_put_success = tx_queue_put_success;
    int tx_start = uart_tx_start_count;
    int tx_err = uart_tx_error_count;
    int tx_done = uart_tx_done_count;
    int q_used = k_msgq_num_used_get(&tx_queue);

    /* RX stats snapshot */
    int rx_alloc_attempts = rx_copy_alloc_attempts;
    int rx_alloc_fails = rx_copy_alloc_fails;
    int rx_free_count = rx_copy_free_count;
    int rx_q_put_fails = rx_queue_put_fails;
    int rx_q_put_ok = rx_queue_put_success;
    int rx_rdys = uart_rx_rdys;
    int rx_buf_req = uart_rx_buf_request_count;
    int rx_buf_rel = uart_rx_buf_released_count;
    int rx_q_used = k_msgq_num_used_get(&rx_queue);

    printk("UART TX stats: alloc=%d fail=%d free=%d q_put_fail=%d q_put_ok=%d tx_start=%d tx_err=%d tx_done=%d q_used=%d\n", tx_alloc_attempts, tx_alloc_fails, tx_free_count, q_put_fails, q_put_success, tx_start, tx_err, tx_done, q_used);

    printk("UART RX stats: rdys=%d alloc=%d alloc_fail=%d free=%d q_put_ok=%d q_put_fail=%d q_used=%d buf_req=%d buf_rel=%d\n", rx_rdys, rx_alloc_attempts, rx_alloc_fails, rx_free_count, rx_q_put_ok, rx_q_put_fails, rx_q_used, rx_buf_req, rx_buf_rel);
}

K_THREAD_DEFINE(app_uart_rx_id, UART_RX_THREAD_STACKSIZE, app_uart_rx_thread, NULL, NULL, NULL, UART_RX_THREAD_PRIORITY, 0, 0);
K_THREAD_DEFINE(app_uart_tx_id, UART_TX_THREAD_STACKSIZE, app_uart_tx_thread, NULL, NULL, NULL, UART_TX_THREAD_PRIORITY, 0, 0);