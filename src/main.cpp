/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "main.h"
#include "bsp_adc.h"
#include "bsp_esb.h"
#include "bsp_ble.h"
#include "bsp_vofa.h"
#include "bsp_led.h"
#include "bsp_mlx90393.hpp"
#include "app_uart.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/** mlx90393 */
// const struct i2c_dt_spec mlx90393_spec = I2C_DT_SPEC_GET(DT_NODELABEL(mlx90393));
// Mlx90393 mlx90393(mlx90393_spec);

/** Timer  */
static K_SEM_DEFINE(k_timer_sem, 0, 1);
static struct k_timer k_timer;
static void k_timer_expiry(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&k_timer_sem);
}

/** app */
int main(void)
{
	int err;
	static int uart_stats_counter = 0;
    bsp_led_init();


    app_uart_init();
    
    LOG_INF("Starting node ID %d.", THIS_NODE_ID);
    bsp_ble_init();
    LOG_INF("BLE initialized.");


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
        LOG_ERR("RX setup failed, err %d", err);
        return 0;
    }
#endif

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
    if (!mlx90393.begin())
    {
        LOG_ERR("mlx90393 begin failed!");
        return 0;
    }
    bsp_adc_init();

#endif

	/* Initialize and start the ADC timer */
	k_timer_init(&k_timer, k_timer_expiry, NULL);
	k_timer_start(&k_timer, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

    /* Main loop */
	while (1)
    {
        k_sem_take(&k_timer_sem, K_FOREVER);

#if (THIS_NODE_ID == LEFT_FOOT_NODE_ID) || (THIS_NODE_ID == RIGHT_FOOT_NODE_ID)
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
        /* per-second UART stats print (200 * 5ms = 1s) */
        uart_stats_counter++;
        if (uart_stats_counter >= 200)
        {
            uart_stats_counter = 0;
            // app_uart_print_stats();
        }
	}

    return 0;
}