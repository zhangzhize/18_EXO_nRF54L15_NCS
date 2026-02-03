#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <errno.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LEFT_FOOT_NODE_ID       0x02
#define RIGHT_FOOT_NODE_ID      0x03
#define STM32H7_NODE_ID         0x04
#define BRIDGE_NODE_ID          0x05

#define THIS_NODE_ID            LEFT_FOOT_NODE_ID

#define SAMPLE_PERIOD_US        5000U


enum
{
    kFoot_ADC = 0x01,
    kFoot_ADC_IMU = 0x02,
};
struct sensor_data_packet_t
{
    uint8_t data_type;
    uint8_t left_foot_adc_data[8];
    uint8_t right_foot_adc_data[8];
    uint8_t left_foot_imu_data[12];
    uint8_t right_foot_imu_data[12];
};
BUILD_ASSERT(sizeof(struct sensor_data_packet_t) == 41, "sensor_data_packet_t must be 41 bytes");
extern struct sensor_data_packet_t sensor_data;


#ifdef __cplusplus
}
#endif

#endif