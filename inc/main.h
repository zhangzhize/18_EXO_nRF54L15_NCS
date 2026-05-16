#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <errno.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STM32H7_NODE_ID         0x01
#define LEFT_FOOT_NODE_ID       0x02
#define RIGHT_FOOT_NODE_ID      0x03
#define BRIDGE_NODE_ID          0x04

#define THIS_NODE_ID            RIGHT_FOOT_NODE_ID

#define SAMPLE_PERIOD_US        5000U

typedef struct foot_sensor_packet_t
{
    int32_t mV_heel;
    int32_t mV_toe;
    float mV_pull;
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
} foot_sensor_packet_t;
BUILD_ASSERT(sizeof(struct foot_sensor_packet_t) == 28, "foot_sensor_packet_t must be 28 bytes");
extern struct foot_sensor_packet_t foot_sensor_data;

typedef struct exo_sensor_packet_t
{
    foot_sensor_packet_t left_foot;
    foot_sensor_packet_t right_foot;
} exo_sensor_packet_t;
extern struct exo_sensor_packet_t exo_sensor_data;


#ifdef __cplusplus
}
#endif

#endif