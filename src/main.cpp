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
#include "SparkFun_BNO08x_Library.hpp"
#include "SparkFun_ADS122C04_Library.hpp"
#include "app_uart.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

struct sensor_data_packet_t sensor_data = 
{
    .data_type = kFoot_ADC,
    .left_foot_adc_data = {0},
    .right_foot_adc_data = {0},
    .left_foot_imu_data = {0},
    .right_foot_imu_data = {0},
};

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
/** mlx90393 */
// const struct i2c_dt_spec mlx90393_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(mlx90393));
// Mlx90393 mlx90393;

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
	int err;
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

    if (!bno085.begin(&bno085_i2c, &bno085_intn, &bno085_nrst))
    {
        LOG_ERR("bno085 begin failed!");
        return 0;
    }
    if (!bno085.enableRotationVector(100))
    {
        LOG_ERR("bno085 rotation vector enable failed!");
    }

    if (!ads122c04.begin(&ads122c04_i2c, &ads122c04_drdy))
    {
        LOG_ERR("ads122c04 begin failed!");
        return 0;
    }
    if (!ads122c04.configureADCmode(ADS122C04_BRIDGE_MODE, ADS122C04_DATA_RATE_90SPS))
    {
        LOG_ERR("ads122c04 configureADCmode failed!");
        return 0;
    }
    k_msleep(1000);

    bsp_adc_init();

#endif

	/* Initialize and start the ADC timer */
	k_timer_init(&k_timer, k_timer_expiry, NULL);
	k_timer_start(&k_timer, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

    while (1)
    {
        k_sem_take(&k_timer_sem, K_FOREVER);

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
        if (bno085.wasReset())
        {
            LOG_INF("bno085 was reset"); 
            if (!bno085.enableRotationVector(100))
            {
                LOG_ERR("bno085 rotation vector enable failed!");
            }
        }
        if (bno085.getSensorEvent())
        {
            if (bno085.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
            {
                float quatI = bno085.getQuatI();
                float quatJ = bno085.getQuatJ();
                float quatK = bno085.getQuatK();
                float quatReal = bno085.getQuatReal();
                float quatRadianAccuracy = bno085.getQuatRadianAccuracy();

                LOG_INF("quatI: %f, quatJ: %f, quatK: %f, quatReal: %f, quatRadianAccuracy: %f", quatI, quatJ, quatK, quatReal, quatRadianAccuracy);
            }
        }

        float milvol = ads122c04.readBridgeVoltage();
        LOG_INF("Bridge voltage: %f", milvol * 1000.0f);
        bsp_adc_read_channels();  /* 166 us */

        tx_payload.data[0] = THIS_NODE_ID;
        memcpy(tx_payload.data + 1, &mV_ain0, sizeof(mV_ain0));
        memcpy(tx_payload.data + 1 + sizeof(mV_ain0), &mV_ain1, sizeof(mV_ain1));
        tx_payload.length = 1 + sizeof(mV_ain0) + sizeof(mV_ain1);
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

#elif (THIS_NODE_ID == STM32H7_NODE_ID)
        if (sensor_data.data_type == kFoot_ADC)
        {
            app_uart_tx(reinterpret_cast<const uint8_t*>(&sensor_data), 1 + 2 * 8);
        }
        else if (sensor_data.data_type == kFoot_ADC_IMU)
        {
            app_uart_tx(reinterpret_cast<const uint8_t*>(&sensor_data), 1 + 2 * 8 + 2 * 12);
        }
#endif

    } // while (1)

    return 0;
}