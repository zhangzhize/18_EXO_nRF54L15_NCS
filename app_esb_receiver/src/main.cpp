/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include "main.h"
#include "bsp_esb.h"
#include "bsp_led.h"
#include "bsp_ads1220.hpp"
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct spi_dt_spec stm32_slave_spec = SPI_DT_SPEC_GET(
  DT_NODELABEL(stm32_slave),
  SPI_WORD_SET(8) | SPI_TRANSFER_MSB);

static exo_sensor_packet_t exo_sensor_data_snapshot;
static struct spi_buf tx_spi_buf = {.buf = &exo_sensor_data_snapshot, .len = sizeof(exo_sensor_packet_t)};
static struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

static int32_t left_uV_0_1;
static int32_t left_uV_2_3;
static int32_t right_uV_0_1;
static int32_t right_uV_2_3;

/** ADS1220 force measurement thread */
#define FORCE_MEAS_THREAD_STACKSIZE 1024
#define FORCE_MEAS_THREAD_PRIORITY 2

static const struct spi_dt_spec ads1220a_spec = SPI_DT_SPEC_GET(
  DT_NODELABEL(ads1220a),
  SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_MODE_CPHA);
const struct gpio_dt_spec ads1220a_drdy_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ads1220a), drdy_gpios);
ADS1220 ads1220a;

static const struct spi_dt_spec ads1220b_spec = SPI_DT_SPEC_GET(
  DT_NODELABEL(ads1220b),
  SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_MODE_CPHA);
const struct gpio_dt_spec ads1220b_drdy_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ads1220b), drdy_gpios);
ADS1220 ads1220b;

void force_meas_thread(void *p1, void *p2, void *p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  bool is_ok_a = ads1220a.begin(&ads1220a_spec, &ads1220a_drdy_spec);
  bool is_ok_b = ads1220b.begin(&ads1220b_spec, &ads1220b_drdy_spec);
  bool current_is_0_1 = true;
  if (is_ok_a && is_ok_b)
  {
    ads1220a.disableDrdyInterrupt();
    ads1220a.setCompareChannels(ADS1220_MUX_0_1);
    ads1220b.setCompareChannels(ADS1220_MUX_0_1);
    k_sem_reset(&ads1220b.data_ready_sem_);
    ads1220a.start();
    ads1220b.start();
    current_is_0_1 = true;
  }
  else
  {
    LOG_ERR("ads1220 begin failed: is_ok_a = %d, is_ok_b = %d", is_ok_a, is_ok_b);
    return;
  }

  int32_t a_raw_data = 0;
  int32_t b_raw_data = 0;
  while (1)
  {
    k_sem_take(&ads1220b.data_ready_sem_, K_FOREVER);

    b_raw_data = ads1220b.getRawData();
    a_raw_data = ads1220a.getRawData();
    if (current_is_0_1)
    {
      left_uV_0_1 = a_raw_data;
      right_uV_0_1 = b_raw_data;
      ads1220a.setCompareChannels(ADS1220_MUX_2_3);
      ads1220b.setCompareChannels(ADS1220_MUX_2_3);
    }
    else
    {
      left_uV_2_3 = a_raw_data;
      right_uV_2_3 = b_raw_data;
      ads1220a.setCompareChannels(ADS1220_MUX_0_1);
      ads1220b.setCompareChannels(ADS1220_MUX_0_1);
      LOG_DBG("left_uV_0_1: %d, left_uV_2_3: %d, right_uV_0_1: %d, right_uV_2_3: %d",
              left_uV_0_1,
              left_uV_2_3,
              right_uV_0_1,
              right_uV_2_3);
    }

    k_sem_reset(&ads1220b.data_ready_sem_);
    ads1220a.start();
    ads1220b.start();

    if (!current_is_0_1)
    {
      k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
      memcpy(&exo_sensor_data_snapshot, &exo_sensor_data, sizeof(exo_sensor_packet_t));
      k_spin_unlock(&exo_data_lock, key);

      int spi_err = spi_write_dt(&stm32_slave_spec, &tx_set);
      if (spi_err)
      {
        LOG_ERR("spi_write_dt() failed, err %d", spi_err);
      }
    }
    current_is_0_1 = !current_is_0_1;
  }
}

K_THREAD_DEFINE(force_meas_thread_id, FORCE_MEAS_THREAD_STACKSIZE, force_meas_thread, NULL, NULL, NULL, FORCE_MEAS_THREAD_PRIORITY, 0, 0);

static void esb_rx_handler(const uint8_t *data, size_t length)
{
  static uint32_t left_cnt = 0;
  static uint32_t right_cnt = 0;
  if (length == 1 + sizeof(foot_sensor_packet_t))
  {
    k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
    if (data[0] == EXO_NODE_ESB_FOOT_LEFT)
    {
      memcpy(&exo_sensor_data.left_foot, data + 1, sizeof(foot_sensor_packet_t));
      left_cnt++;
    }
    else if (data[0] == EXO_NODE_ESB_FOOT_RIGHT)
    {
      memcpy(&exo_sensor_data.right_foot, data + 1, sizeof(foot_sensor_packet_t));
      right_cnt++;
    }
    k_spin_unlock(&exo_data_lock, key);
  }
  LOG_DBG_RATELIMIT_RATE(200, "left: %d, right: %d", left_cnt, right_cnt);
}

/** main */
int main(void)
{
  int err = 0;

  bsp_led_init();

  if (!spi_is_ready_dt(&stm32_slave_spec))
  {
    LOG_ERR("STM32 SPI device is not ready");
    return 0;
  }

  err = clocks_start();
  if (err)
  {
    LOG_ERR("clocks_start() failed, err %d", err);
    return 0;
  }
  err = esb_initialize();
  if (err)
  {
    LOG_ERR("esb_initialize() failed, err %d", err);
    return 0;
  }

  esb_set_rx_callback(esb_rx_handler);

  err = esb_start_rx();
  if (err)
  {
    LOG_ERR("esb_start_rx() failed, err %d", err);
    return 0;
  }

  k_sleep(K_FOREVER);

  return 0;
}
