#ifndef __MAIN_H
#define __MAIN_H

// #include <zephyr/kernel.h>
// #include <zephyr/types.h>
// #include <zephyr/device.h>
// #include <zephyr/logging/log.h>

// #include <zephyr/sys/atomic.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/uart.h>
// #include <zephyr/irq.h>
// #include <string.h>
// #include <nrf.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEFT_FOOT_NODE_ID       0x02
#define RIGHT_FOOT_NODE_ID      0x03
#define STM32H7_NODE_ID         0x04
#define BRIDGE_NODE_ID          0x05

#define THIS_NODE_ID            STM32H7_NODE_ID

#define SAMPLE_PERIOD_US        5000U

#ifdef __cplusplus
}
#endif

#endif