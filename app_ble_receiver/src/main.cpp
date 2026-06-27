/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "ble_sensor_gateway.hpp"
#include "bsp_ble.h"
#include "bsp_ble_central.h"
#include "bsp_led.h"
#include "main.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ── SPI slave to STM32 ──────────────────────────────────────────── */

static const struct spi_dt_spec stm32_slave_spec = SPI_DT_SPEC_GET(
  DT_NODELABEL(stm32_slave),
  SPI_WORD_SET(8) | SPI_TRANSFER_MSB);

static exo_ble_sensor_packet_t tx_snapshot;
static struct spi_buf tx_spi_buf = {
  .buf = &tx_snapshot,
  .len = sizeof(exo_ble_sensor_packet_t),
};
static struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

/* ── Main ──────────────────────────────────────────────────────────── */

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
  size_t profile_count = 0U;
  const struct ble_device_profile *profiles;

  bsp_led_init();

  if (!spi_is_ready_dt(&stm32_slave_spec))
  {
    LOG_ERR("STM32 SPI device is not ready");
    return 0;
  }

  err = bsp_ble_init();
  if (err)
  {
    LOG_ERR("bsp_ble_init() failed, err %d", err);
    return 0;
  }

  profiles = ble_sensor_gateway_profiles(&profile_count);
  err = bsp_ble_central_start(profiles, (uint8_t)profile_count);
  if (err)
  {
    LOG_ERR("bsp_ble_central_start() failed, err %d", err);
    return 0;
  }

  LOG_INF("BLE Gateway: scanning for %u sensors across %u profiles",
          (unsigned int)ble_sensor_gateway_target_count(),
          (unsigned int)profile_count);

  k_timer_init(&k_timer, k_timer_expiry, NULL);
  k_timer_start(&k_timer, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

  while (1)
  {
    k_sem_take(&k_timer_sem, K_FOREVER);

    if (!(bsp_ble_central_all_links_ready() && ble_sensor_gateway_streams_ready()))
    {
      continue;
    }

    ble_sensor_gateway_copy_packet(&tx_snapshot);

    int spi_err = spi_write_dt(&stm32_slave_spec, &tx_set);
    if (spi_err)
    {
      LOG_ERR_RATELIMIT_RATE(1000, "spi_write_dt() failed, err %d", spi_err);
    }
  }

  return 0;
}
