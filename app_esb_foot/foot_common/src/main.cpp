/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include "main.h"
#include "bsp_adc.h"
#include "bsp_esb.h"
#include "bsp_led.h"
#include "SparkFun_BNO08x_Library.hpp"
#include "SparkFun_ADS122C04_Library.hpp"
#include "pcap01.hpp"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_ERR);

/** bno085 */
const struct i2c_dt_spec bno085_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(bno085));
const struct gpio_dt_spec bno085_intn = GPIO_DT_SPEC_GET(DT_NODELABEL(bno085), intn_gpios);
const struct gpio_dt_spec bno085_nrst = GPIO_DT_SPEC_GET(DT_NODELABEL(bno085), nrst_gpios);
BNO08x bno085;

/** ads122c04 */
const struct i2c_dt_spec ads122c04_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(ads122c04));
const struct gpio_dt_spec ads122c04_drdy = GPIO_DT_SPEC_GET(DT_NODELABEL(ads122c04), drdy_gpios);
SFE_ADS122C04 ads122c04;

/** PCAP01 capacitance sensor */
static const struct spi_dt_spec pcap01_spi = SPI_DT_SPEC_GET(
  DT_NODELABEL(pcap01),
  SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_MODE_CPHA);
static const struct gpio_dt_spec pcap01_intn = GPIO_DT_SPEC_GET(DT_NODELABEL(pcap01), intn_gpios);
PCAP01 pcap01;

#if EXO_THIS_NODE_IS(EXO_NODE_ESB_FOOT_LEFT)
static constexpr float cap2for_coeffs_heel[4] = {0.0004f, -0.0211f, 1.2841f, -14.878f};
static constexpr float cap2for_coeffs_toe[4] = {0.0009f, -0.0868f, 3.4329f, -32.418f};
#else
static constexpr float cap2for_coeffs_heel[4] = {0.0004f, -0.0439f, 2.3719f, -27.584f};
static constexpr float cap2for_coeffs_toe[4] = {0.001f, -0.0981f, 3.7782f, -35.732f};
#endif

/** BNO085 sensor thread */
#define BNO_THREAD_STACKSIZE 2048
#define BNO_THREAD_PRIORITY 5
#define BNO_REPORT_INTERVAL_MS 5
#define BNO_ENABLE_RETRY_COUNT 20
#define BNO_ENABLE_RETRY_DELAY_MS 100

static bool bno085_enable_rotation_vector_with_retry(void)
{
  for (int attempt = 1; attempt <= BNO_ENABLE_RETRY_COUNT; attempt++)
  {
    if (bno085.wasReset())
    {
      LOG_INF("bno085 was reset");
    }

    if (bno085.enableRotationVector(BNO_REPORT_INTERVAL_MS))
    {
      LOG_INF("bno085 rotation vector enabled");
      return true;
    }

    LOG_WRN("bno085 rotation vector enable retry %d/%d",
            attempt,
            BNO_ENABLE_RETRY_COUNT);

    int64_t retry_until = k_uptime_get() + BNO_ENABLE_RETRY_DELAY_MS;
    while (k_uptime_get() < retry_until)
    {
      (void)bno085.getSensorEvent();
      k_msleep(10);
    }
  }

  return false;
}

void bno085_thread(void *p1, void *p2, void *p3)
{
  bool rotation_vector_enabled = false;
  int64_t next_enable_retry_ms = 0;

  LOG_INF("Initializing BNO085...");
  if (!bno085.begin(&bno085_i2c, &bno085_intn, &bno085_nrst))
  {
    LOG_ERR("bno085 begin failed!");
    return;
  }
  rotation_vector_enabled = bno085_enable_rotation_vector_with_retry();
  if (!rotation_vector_enabled)
  {
    LOG_ERR("bno085 rotation vector enable failed after retries");
    next_enable_retry_ms = k_uptime_get() + 1000;
  }
  while (1)
  {
    if (bno085.wasReset())
    {
      LOG_INF("bno085 was reset");
      rotation_vector_enabled = bno085_enable_rotation_vector_with_retry();
      if (!rotation_vector_enabled)
      {
        LOG_ERR("bno085 rotation vector re-enable failed after retries");
        next_enable_retry_ms = k_uptime_get() + 1000;
      }
    }
    if (!rotation_vector_enabled && k_uptime_get() >= next_enable_retry_ms)
    {
      rotation_vector_enabled = bno085_enable_rotation_vector_with_retry();
      next_enable_retry_ms = k_uptime_get() + 1000;
    }
    if (bno085.getSensorEvent())
    {
      if (bno085.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
      {
        rotation_vector_enabled = true;
        float quat_i = bno085.getQuatI();
        float quat_j = bno085.getQuatJ();
        float quat_k = bno085.getQuatK();
        float quat_real = bno085.getQuatReal();

        k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
        foot_sensor_data.quatI = quat_i;
        foot_sensor_data.quatJ = quat_j;
        foot_sensor_data.quatK = quat_k;
        foot_sensor_data.quatReal = quat_real;
        k_spin_unlock(&exo_data_lock, key);

        LOG_INF_RATELIMIT_RATE(1000, "quatI: %f, quatJ: %f, quatK: %f, quatReal: %f", static_cast<double>(quat_i), static_cast<double>(quat_j), static_cast<double>(quat_k), static_cast<double>(quat_real));
      }
    }
  }
}
// K_THREAD_DEFINE(bno_thread_id, BNO_THREAD_STACKSIZE, bno085_thread, NULL, NULL, NULL, BNO_THREAD_PRIORITY, 0, 0);

/** ADS122C04 force sensor thread */
#define ADS_THREAD_STACKSIZE 1024
#define ADS_THREAD_PRIORITY 6

void ads122c04_thread(void *p1, void *p2, void *p3)
{
  LOG_INF("Initializing ADS122C04...");
  if (!ads122c04.begin(&ads122c04_i2c, &ads122c04_drdy))
  {
    LOG_ERR("ads122c04 begin failed!");
    return;
  }
  if (!ads122c04.configureADCmode(ADS122C04_BRIDGE_MODE, ADS122C04_DATA_RATE_175SPS))
  {
    LOG_ERR("ads122c04 configureADCmode failed!");
    return;
  }
  ads122c04.setConversionMode(ADS122C04_CONVERSION_MODE_CONTINUOUS);
  ads122c04.start();

  LOG_INF("ads122c04 Started in Continuous Mode.");

  while (1)
  {
    float force = ads122c04.readBridgeMilliVolts();

    k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
    foot_sensor_data.force = force;
    k_spin_unlock(&exo_data_lock, key);

    LOG_INF_RATELIMIT_RATE(1000, "Bridge volt (uV): %f", static_cast<double>(force * 1000.0f));
  }
}
// K_THREAD_DEFINE(ads_thread_id, ADS_THREAD_STACKSIZE, ads122c04_thread, NULL, NULL, NULL, ADS_THREAD_PRIORITY, 0, 0);

/** PCAP01 capacitance sensor thread */
#define PCAP01_THREAD_STACKSIZE 2048
#define PCAP01_THREAD_PRIORITY 6
#define PCAP01_MEAS_TIMEOUT_MS 80

void pcap01_thread(void *p1, void *p2, void *p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  LOG_INF("Initializing PCAP01...");
  PCAP01_ERROR err = pcap01.begin(&pcap01_spi, &pcap01_intn);
  if (err != PCAP01_ERROR::OK)
  {
    LOG_ERR("pcap01 begin failed, err %d", static_cast<int>(err));
    return;
  }

  LOG_INF("pcap01 started");

  int64_t next_log_ms = k_uptime_get() + 1000;
  int64_t next_timeout_log_ms = 0;
  uint32_t sample_count = 0;
  int64_t max_wait_ms = 0;

  while (1)
  {
    uint32_t raw_count[PCAP01::CHANNEL_COUNT] = {0};
    float raw_cap_pf[PCAP01::CHANNEL_COUNT] = {0.0f};
    int64_t wait_ms = 0;
    uint32_t status = 0;
    if (!pcap01.measureCapacitancesPF(K_MSEC(PCAP01_MEAS_TIMEOUT_MS),
                                      raw_cap_pf,
                                      raw_count,
                                      &wait_ms,
                                      &status))
    {
      int64_t now_ms = k_uptime_get();
      if (now_ms >= next_timeout_log_ms)
      {
        LOG_WRN("pcap01 invalid result raw=%06x/%06x status=%06x bytes=%02x/%02x wait=%lld ms",
                raw_count[0],
                raw_count[1],
                raw_count[2],
                status,
                (status >> 16) & 0xFFU,
                (status >> 8) & 0xFFU,
                status & 0xFFU,
                wait_ms);
        next_timeout_log_ms = now_ms + 1000;
      }
      continue;
    }
    if (wait_ms > max_wait_ms)
    {
      max_wait_ms = wait_ms;
    }

    float cap_pf[PCAP01::CHANNEL_COUNT] = {0.0f};
    for (uint8_t channel = 0; channel < PCAP01::CHANNEL_COUNT; channel++)
    {
      cap_pf[channel] = pcap01.applyLowPassFilter(channel, raw_cap_pf[channel]);
    }
    sample_count++;

    float force_heel_N = cap2for_coeffs_heel[0] * cap_pf[0] * cap_pf[0] * cap_pf[0] + cap2for_coeffs_heel[1] * cap_pf[0] * cap_pf[0] + cap2for_coeffs_heel[2] * cap_pf[0] + cap2for_coeffs_heel[3];
    float force_toe_N = cap2for_coeffs_toe[0] * cap_pf[1] * cap_pf[1] * cap_pf[1] + cap2for_coeffs_toe[1] * cap_pf[1] * cap_pf[1] + cap2for_coeffs_toe[2] * cap_pf[1] + cap2for_coeffs_toe[3];

    k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
    foot_sensor_data.force_N[0] = force_heel_N;
    foot_sensor_data.force_N[1] = force_toe_N;
    k_spin_unlock(&exo_data_lock, key);

    int64_t now_ms = k_uptime_get();
    if (now_ms >= next_log_ms)
    {
      LOG_INF("pcap01 raw=%06x/%06x cap_pf=%.3f/%.3f filt=%.3f/%.3f force=%.2f/%.2f N samples=%u max_wait=%lld ms",
              raw_count[0],
              raw_count[1],
              static_cast<double>(raw_cap_pf[0]),
              static_cast<double>(raw_cap_pf[1]),
              static_cast<double>(cap_pf[0]),
              static_cast<double>(cap_pf[1]),
              static_cast<double>(force_heel_N),
              static_cast<double>(force_toe_N),
              sample_count,
              max_wait_ms);

      next_log_ms = now_ms + 1000;
      sample_count = 0;
      max_wait_ms = 0;
    }
  }
}
K_THREAD_DEFINE(pcap01_thread_id, PCAP01_THREAD_STACKSIZE, pcap01_thread, NULL, NULL, NULL, PCAP01_THREAD_PRIORITY, 0, 0);

/** main */

static K_SEM_DEFINE(k_timer_sem, 0, 1);
static struct k_timer k_timer;
static void k_timer_expiry(struct k_timer *timer)
{
  ARG_UNUSED(timer);
  k_sem_give(&k_timer_sem);
}

int main(void)
{
  int err = 0;

  bsp_led_init();

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

  err = bsp_adc_init();
  if (err)
  {
    LOG_ERR("bsp_adc_init() failed, err %d", err);
    return 0;
  }

  /* Initialize and start the ADC timer */
  k_timer_init(&k_timer, k_timer_expiry, NULL);
  k_timer_start(&k_timer, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

  while (1)
  {
    k_sem_take(&k_timer_sem, K_FOREVER);

    continue;

    k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
    bsp_adc_read_channels(); /* ~166 us */
    memcpy(tx_payload.data + 1, &foot_sensor_data, sizeof(foot_sensor_data));
    k_spin_unlock(&exo_data_lock, key);

    tx_payload.data[0] = EXO_THIS_NODE_ID;
    tx_payload.length = 1 + sizeof(foot_sensor_data);
    tx_payload.noack = false;
    if (ready)
    {
      ready = false;

      esb_flush_tx();
      err = esb_write_payload(&tx_payload);
      if (err)
      {
        LOG_ERR("Payload write failed, err %d", err);
      }
    }
    else
    {
      LOG_WRN("Payload not ready");
    }
  }

  return 0;
}
