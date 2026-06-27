/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "main.h"

/** Global sensor data — shared across BSP modules */
struct k_spinlock exo_data_lock;

struct exo_ble_sensor_packet_t exo_ble_sensor_data = {0};

struct foot_sensor_packet_t foot_sensor_data = {
  .mV_heel = 0,
  .mV_toe = 0,
  .force = 0,
  .quatI = 0,
  .quatJ = 0,
  .quatK = 0,
  .quatReal = 0,
  .force_N = {0},
};

struct exo_sensor_packet_t exo_sensor_data = {
  .left_foot = {
    .mV_heel = 0,
    .mV_toe = 0,
    .force = 0,
    .quatI = 0,
    .quatJ = 0,
    .quatK = 0,
    .quatReal = 0,
    .force_N = {0},
  },
  .right_foot = {
    .mV_heel = 0,
    .mV_toe = 0,
    .force = 0,
    .quatI = 0,
    .quatJ = 0,
    .quatK = 0,
    .quatReal = 0,
    .force_N = {0},
  },
};
