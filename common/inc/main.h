#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "exo_node.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SAMPLE_PERIOD_US 5000U

#define EXO_BLE_IMU_COUNT 4U
#define EXO_BLE_INSOLE_COUNT 2U
#define EXO_BLE_INSOLE_CHANNELS 8U

enum exo_ble_imu_slot
{
  EXO_BLE_IMU_LEFT_SHANK = 0,
  EXO_BLE_IMU_RIGHT_SHANK,
  EXO_BLE_IMU_LEFT_THIGH,
  EXO_BLE_IMU_RIGHT_THIGH,
};

typedef struct __attribute__((packed)) exo_ble_imu_sample_t
{
  float gyro_radps[3];
  float euler_rad[3];
} exo_ble_imu_sample_t;

typedef struct __attribute__((packed)) exo_ble_insole_sample_t
{
  int16_t channels[EXO_BLE_INSOLE_CHANNELS];
} exo_ble_insole_sample_t;

typedef struct __attribute__((packed)) exo_ble_sensor_packet_t
{
  exo_ble_imu_sample_t imu[EXO_BLE_IMU_COUNT];
  exo_ble_insole_sample_t insole[EXO_BLE_INSOLE_COUNT];
} exo_ble_sensor_packet_t;
extern struct exo_ble_sensor_packet_t exo_ble_sensor_data;
BUILD_ASSERT(sizeof(struct exo_ble_sensor_packet_t) == 128, "exo_ble_sensor_packet_t must be 128 bytes");

typedef struct __attribute__((packed)) foot_sensor_packet_t
{
  int32_t mV_heel;
  int32_t mV_toe;
  float force;
  float quatI;
  float quatJ;
  float quatK;
  float quatReal;
  float force_N[2];
} foot_sensor_packet_t;
BUILD_ASSERT(sizeof(struct foot_sensor_packet_t) == 36, "foot_sensor_packet_t must be 36 bytes");
extern struct foot_sensor_packet_t foot_sensor_data;

typedef struct __attribute__((packed)) exo_sensor_packet_t
{
  foot_sensor_packet_t left_foot;
  foot_sensor_packet_t right_foot;
} exo_sensor_packet_t;
BUILD_ASSERT(sizeof(struct exo_sensor_packet_t) == 72, "exo_sensor_packet_t must be 72 bytes");
extern struct exo_sensor_packet_t exo_sensor_data;
extern struct k_spinlock exo_data_lock;

#ifdef __cplusplus
}
#endif

#endif
