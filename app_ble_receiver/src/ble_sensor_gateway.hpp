#ifndef BLE_SENSOR_GATEWAY_HPP
#define BLE_SENSOR_GATEWAY_HPP

#include <stddef.h>

#include "bsp_ble_central.h"
#include "main.h"

const struct ble_device_profile *ble_sensor_gateway_profiles(size_t *count);
size_t ble_sensor_gateway_target_count(void);
bool ble_sensor_gateway_streams_ready(void);
void ble_sensor_gateway_copy_packet(exo_ble_sensor_packet_t *snapshot);

#endif /* BLE_SENSOR_GATEWAY_HPP */
