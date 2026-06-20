/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "main.h"
#include "bsp_adc.h"
#include "bsp_esb.h"
#include "bsp_ble.h"
#include "vofa.hpp"
#include "bsp_led.h"
#include "bsp_mlx90393.hpp"
#include "bsp_ads1220.hpp"
#include "SparkFun_BNO08x_Library.hpp"
#include "SparkFun_ADS122C04_Library.hpp" 
#include "app_uart.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

struct foot_sensor_packet_t foot_sensor_data =
{
    .mV_heel = 0,
    .mV_toe = 0,
    .force = 0,
    .quatI = 0,
    .quatJ = 0,
    .quatK = 0,
    .quatReal = 0,
};  /** for LEFT_FOOT_NODE and RIGHT_FOOT_NODE */

struct exo_sensor_packet_t exo_sensor_data =
{
    .left_foot = {
        .mV_heel = 0,
        .mV_toe = 0,
        .force = 0,
        .quatI = 0,
        .quatJ = 0,
        .quatK = 0,
        .quatReal = 0,
    },
    .right_foot = {
        .mV_heel = 0,
        .mV_toe = 0,
        .force = 0,
        .quatI = 0,
        .quatJ = 0,
        .quatK = 0,
        .quatReal = 0,
    },
};  /** for STM32H7_NODE */

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
/** mlx90393 */
const struct i2c_dt_spec mlx90393_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(mlx90393));
Mlx90393 mlx90393;

/** bno085 */
const struct i2c_dt_spec bno085_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(bno085));
const struct gpio_dt_spec bno085_intn = GPIO_DT_SPEC_GET(DT_NODELABEL(bno085), intn_gpios);
const struct gpio_dt_spec bno085_nrst = GPIO_DT_SPEC_GET(DT_NODELABEL(bno085), nrst_gpios);
BNO08x bno085;

/** ads122c04 */
const struct i2c_dt_spec ads122c04_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(ads122c04));
const struct gpio_dt_spec ads122c04_drdy = GPIO_DT_SPEC_GET(DT_NODELABEL(ads122c04), drdy_gpios);
SFE_ADS122C04 ads122c04;

#endif

#if (THIS_NODE_ID == STM32H7_NODE_ID)

static const struct spi_dt_spec stm32_slave_spec = SPI_DT_SPEC_GET(
    DT_NODELABEL(stm32_slave),  
    SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 
    0
);

exo_sensor_packet_t exo_sensor_data_snapshot;
struct spi_buf tx_spi_buf = {.buf = &exo_sensor_data_snapshot, .len = sizeof(exo_sensor_packet_t)};
struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

int32_t left_uV_0_1 = 0;
int32_t left_uV_2_3 = 0;
int32_t right_uV_0_1 = 0;
int32_t right_uV_2_3 = 0;

#endif

/** Timer  */
static K_SEM_DEFINE(k_timer_sem, 0, 1);
static struct k_timer k_timer;
static void k_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&k_timer_sem);
}

/** app NRF54L15*/
int main(void)
{
	int err = 0;

    bsp_led_init();

#if (THIS_NODE_ID == BRIDGE_NODE_ID) || (THIS_NODE_ID == STM32H7_NODE_ID)
    err = app_uart_init();
    if (err)
    {
        LOG_ERR("app_uart_init() failed, err %d", err);
        return 0;
    }
#endif

#if (THIS_NODE_ID == BRIDGE_NODE_ID)
    err = bsp_ble_init();
    if (err)
    {
        LOG_ERR("bsp_ble_init() failed, err %d", err);
        return 0;
    }
#endif

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID) || (THIS_NODE_ID == STM32H7_NODE_ID)
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
#endif

#if THIS_NODE_ID == STM32H7_NODE_ID
    err = esb_start_rx();
    if (err)
    {
        LOG_ERR("esb_start_rx() failed, err %d", err);
        return 0;
    }
#endif

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
    // if (!mlx90393.begin(&mlx90393_i2c))
    // {
    //     LOG_ERR("mlx90393 begin failed!");
    //     return 0;
    // }

    bsp_adc_init();

#endif

	/* Initialize and start the ADC timer */
	k_timer_init(&k_timer, k_timer_expiry, NULL);
	k_timer_start(&k_timer, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));  //5ms定时

    while (1)
    {
        k_sem_take(&k_timer_sem, K_FOREVER);

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
        // mlx90393.readMeasurement(&x, &y, &z);

        bsp_adc_read_channels();  /* 166 us */
        LOG_INF("mV_heel (mV): %d, mV_toe (mV): %d", foot_sensor_data.mV_heel, foot_sensor_data.mV_toe);

        tx_payload.data[0] = THIS_NODE_ID;

        uint32_t key = irq_lock();
        memcpy(tx_payload.data + 1, &foot_sensor_data, sizeof(foot_sensor_data));
        irq_unlock(key);

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
            LOG_ERR("Payload not ready");
        }

#elif (THIS_NODE_ID == STM32H7_NODE_ID)
        // app_uart_tx(reinterpret_cast<const uint8_t*>(&exo_sensor_data), sizeof(exo_sensor_data));
        /** 或者 */
        // spi_write_dt(&stm32_slave_spec, &tx_set);  //已在force_meas_thread中发送数据了，这里就不重复发送了
#endif

    } // while (1)

    return 0;
}

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)

#define BNO_THREAD_STACKSIZE 2048
#define BNO_THREAD_PRIORITY  5
#define ADS_THREAD_STACKSIZE 1024
#define ADS_THREAD_PRIORITY  6

#define BNO_REPORT_INTERVAL_MS 5

void bno085_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("Initializing BNO085...");
    if (!bno085.begin(&bno085_i2c, &bno085_intn, &bno085_nrst))
    {
        LOG_ERR("bno085 begin failed!");
        return;
    }
    if (!bno085.enableRotationVector(BNO_REPORT_INTERVAL_MS))
    {
        LOG_ERR("bno085 rotation vector enable failed!");
        // return;
    }
    while (1)
    {
        if (bno085.wasReset())
        {
            LOG_INF("bno085 was reset"); 
            if (!bno085.enableRotationVector(BNO_REPORT_INTERVAL_MS))
            {
                LOG_ERR("bno085 rotation vector enable failed!");
            }
        }
        if (bno085.getSensorEvent())
        {
            if (bno085.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
            {
                foot_sensor_data.quatI = bno085.getQuatI();
                foot_sensor_data.quatJ = bno085.getQuatJ();
                foot_sensor_data.quatK = bno085.getQuatK();
                foot_sensor_data.quatReal = bno085.getQuatReal();
                // float quatRadianAccuracy = bno085.getQuatRadianAccuracy();
                LOG_INF("quatI: %f, quatJ: %f, quatK: %f, quatReal: %f", foot_sensor_data.quatI, foot_sensor_data.quatJ, foot_sensor_data.quatK, foot_sensor_data.quatReal);
            }
        }
    }
}
K_THREAD_DEFINE(bno_thread_id, BNO_THREAD_STACKSIZE, bno085_thread, NULL, NULL, NULL, BNO_THREAD_PRIORITY, 0, 0);

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
        foot_sensor_data.force = ads122c04.readBridgeMilliVolts();
        LOG_INF("Bridge volt (uV): %f", foot_sensor_data.force * 1000.0f);
    }
}
K_THREAD_DEFINE(ads_thread_id, ADS_THREAD_STACKSIZE, ads122c04_thread, NULL, NULL, NULL, ADS_THREAD_PRIORITY, 0, 0);

#endif

#if (THIS_NODE_ID == STM32H7_NODE_ID)

#define FORCE_MEAS_THREAD_STACKSIZE 1024
#define FORCE_MEAS_THREAD_PRIORITY  0

static const struct spi_dt_spec ads1220a_spec = SPI_DT_SPEC_GET(
    DT_NODELABEL(ads1220a), 
    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_MODE_CPHA,
    0
);
const struct gpio_dt_spec ads1220a_drdy_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ads1220a), drdy_gpios);
ADS1220 ads1220a;

static const struct spi_dt_spec ads1220b_spec = SPI_DT_SPEC_GET(
    DT_NODELABEL(ads1220b), 
    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE | SPI_MODE_CPHA,
    0
);
const struct gpio_dt_spec ads1220b_drdy_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(ads1220b), drdy_gpios);
ADS1220 ads1220b;


void force_meas_thread(void *p1, void *p2, void *p3)
{
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
            LOG_ERR("left_uV_0_1: %d, left_uV_2_3: %d, right_uV_0_1: %d, right_uV_2_3: %d", left_uV_0_1, left_uV_2_3, right_uV_0_1, right_uV_2_3);
        }

        ads1220a.start();
        ads1220b.start();
        k_sem_reset(&ads1220b.data_ready_sem_);

        if (!current_is_0_1)
        {
            uint32_t key = irq_lock();
            memcpy(&exo_sensor_data_snapshot, &exo_sensor_data, sizeof(exo_sensor_packet_t));
            irq_unlock(key);

            spi_write_dt(&stm32_slave_spec, &tx_set);
        }
        current_is_0_1 = !current_is_0_1;
    }
}

K_THREAD_DEFINE(force_meas_thread_id, FORCE_MEAS_THREAD_STACKSIZE, force_meas_thread, NULL, NULL, NULL, FORCE_MEAS_THREAD_PRIORITY, 0, 0);

#endif