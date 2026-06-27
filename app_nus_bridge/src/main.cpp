/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <string.h>

#include "main.h"
#include "bsp_ble.h"
#include "bsp_ble_periph.h"
#include "bsp_led.h"
#include "bsp_uart.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define RESPONSE_WINDOW_MS 1000U
#define RESPONSE_WINDOW_TICKS (RESPONSE_WINDOW_MS / (SAMPLE_PERIOD_US / 1000U))
#define BLE_TX_RETRY_DELAY_MS 10U
#define BLE_TX_RETRY_ATTEMPTS 100U
#define BLE_TX_RETRY_QUEUE_LEN 8U

struct ble_tx_retry_item
{
  struct bt_conn *conn;
  uint8_t data[BSP_UART_BUF_SIZE];
  uint16_t len;
  uint8_t attempts_left;
};

K_MSGQ_DEFINE(ble_tx_retry_queue, sizeof(ble_tx_retry_item), BLE_TX_RETRY_QUEUE_LEN, 4);
static struct k_work_delayable ble_tx_retry_work;

static struct bt_conn *data_sink;
static struct bt_conn *pending_responder;
static uint16_t responder_ticks;

static bool conn_is_active(struct bt_conn *conn)
{
  if (!conn)
  {
    return false;
  }

  for (uint8_t i = 0; i < ble_nus_get_conn_count(); i++)
  {
    if (ble_nus_get_conn(i) == conn)
    {
      return true;
    }
  }

  return false;
}

static bool ble_send_err_retryable(int err)
{
  return err == -EINVAL || err == -ENOMEM || err == -ENOBUFS || err == -EAGAIN || err == -EBUSY;
}

static void ble_tx_retry_schedule(void)
{
  k_work_schedule(&ble_tx_retry_work, K_MSEC(BLE_TX_RETRY_DELAY_MS));
}

static void ble_tx_retry_queue_put(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
  if (!conn || !data || len == 0 || len > BSP_UART_BUF_SIZE)
  {
    return;
  }

  ble_tx_retry_item item = {
    .conn = bt_conn_ref(conn),
    .len = len,
    .attempts_left = BLE_TX_RETRY_ATTEMPTS,
  };
  memcpy(item.data, data, len);

  int err = k_msgq_put(&ble_tx_retry_queue, &item, K_NO_WAIT);
  if (err)
  {
    bt_conn_unref(item.conn);
    return;
  }

  ble_tx_retry_schedule();
}

static void ble_send_or_retry(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
  if (!conn_is_active(conn))
  {
    return;
  }

  int err = ble_nus_send_to(conn, data, len);
  if (err && ble_send_err_retryable(err))
  {
    ble_tx_retry_queue_put(conn, data, len);
  }
}

static void ble_tx_retry_work_handler(struct k_work *work)
{
  ARG_UNUSED(work);

  uint32_t count = k_msgq_num_used_get(&ble_tx_retry_queue);
  bool reschedule = false;

  for (uint32_t i = 0; i < count; i++)
  {
    ble_tx_retry_item item = {0};
    if (k_msgq_get(&ble_tx_retry_queue, &item, K_NO_WAIT))
    {
      break;
    }

    if (!conn_is_active(item.conn))
    {
      bt_conn_unref(item.conn);
      continue;
    }

    int err = ble_nus_send_to(item.conn, item.data, item.len);
    if (!err)
    {
      bt_conn_unref(item.conn);
      continue;
    }

    if (item.attempts_left > 0 && ble_send_err_retryable(err))
    {
      item.attempts_left--;
      if (!k_msgq_put(&ble_tx_retry_queue, &item, K_NO_WAIT))
      {
        reschedule = true;
        continue;
      }
    }

    bt_conn_unref(item.conn);
  }

  if (reschedule)
  {
    ble_tx_retry_schedule();
  }
}

static void ble_send_target(struct bt_conn *conn, const uint8_t *data, size_t len)
{
  while (conn && data && len > 0)
  {
    uint16_t chunk = (uint16_t)MIN(len, (size_t)BSP_UART_BUF_SIZE);
    ble_send_or_retry(conn, data, chunk);
    data += chunk;
    len -= chunk;
  }
}

static void uart_to_nus_cb(uint8_t *data, size_t len)
{
  if (!data || len == 0)
  {
    return;
  }

  if (data_sink)
  {
    ble_send_target(data_sink, data, len);
  }

  if (pending_responder && pending_responder != data_sink)
  {
    ble_send_target(pending_responder, data, len);
  }
}

static void ble_to_uart_cb(const uint8_t *data, size_t len)
{
  while (data && len > 0)
  {
    size_t chunk = MIN(len, (size_t)BSP_UART_BUF_SIZE);
    int err = bsp_uart_tx(data, chunk);

    if (err)
    {
      LOG_WRN("bsp_uart_tx() failed, err %d", err);
      return;
    }

    data += chunk;
    len -= chunk;
  }
}

static void ble_to_uart_conn_cb(struct bt_conn *conn, const uint8_t *data,
                                size_t len)
{
  if (!conn || !data || len == 0)
  {
    return;
  }

  if (len == 2 && data[0] == 'g' && data[1] == 'g')
  {
    data_sink = conn;
    pending_responder = NULL;
    responder_ticks = 0;
    LOG_INF("Data sink: streaming enabled");
    return;
  }

  if (data_sink == conn && len == 2 && data[0] == 'e' && data[1] == 'e')
  {
    data_sink = NULL;
    LOG_INF("Data sink: streaming paused");
    return;
  }

  if (conn != data_sink)
  {
    pending_responder = conn;
    responder_ticks = RESPONSE_WINDOW_TICKS;
  }

  ble_to_uart_cb(data, len);
}

/* main */

static K_SEM_DEFINE(k_timer_sem, 0, 1);
static struct k_timer k_timer;
static void k_timer_expiry(struct k_timer *timer)
{
  ARG_UNUSED(timer);
  k_sem_give(&k_timer_sem);
}

int main(void)
{
  int err;

  bsp_led_init();

  err = bsp_uart_init();
  if (err)
  {
    LOG_ERR("bsp_uart_init() failed, err %d", err);
    return 0;
  }

  err = bsp_ble_init();
  if (err)
  {
    LOG_ERR("bsp_ble_init() failed, err %d", err);
    return 0;
  }

  err = bsp_ble_periph_init();
  if (err)
  {
    LOG_ERR("bsp_ble_periph_init() failed, err %d", err);
    return 0;
  }

  bsp_uart_rx_cb_register(uart_to_nus_cb);
  bsp_ble_rx_conn_cb_register(ble_to_uart_conn_cb);

  k_work_init_delayable(&ble_tx_retry_work, ble_tx_retry_work_handler);

  k_timer_init(&k_timer, k_timer_expiry, NULL);
  k_timer_start(&k_timer, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

  while (1)
  {
    k_sem_take(&k_timer_sem, K_FOREVER);

    if (responder_ticks > 0 && --responder_ticks == 0)
    {
      pending_responder = NULL;
    }

    if (pending_responder && !conn_is_active(pending_responder))
    {
      pending_responder = NULL;
      responder_ticks = 0;
    }

    if (data_sink && !conn_is_active(data_sink))
    {
      LOG_INF("Data sink disconnected, cleared");
      data_sink = NULL;
    }
  }

  return 0;
}
