#ifndef WIT_IMU_H
#define WIT_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* WitMotion WT901BLE data frame (20 bytes) */
#define WIT_FRAME_LEN 20
#define WIT_FRAME_HEADER 0x55
#define WIT_FRAME_FLAG_ACC_GYRO_ANGLE 0x61

typedef struct __attribute__((packed)) wit_imu_raw_frame_t
{
  uint8_t header; /* 0x55 */
  uint8_t flag; /* 0x61 = acc+gyro+angle */
  int16_t ax; /* X accel, raw */
  int16_t ay; /* Y accel, raw */
  int16_t az; /* Z accel, raw */
  int16_t wx; /* X gyro, raw */
  int16_t wy; /* Y gyro, raw */
  int16_t wz; /* Z gyro, raw */
  int16_t roll; /* X angle, raw */
  int16_t pitch; /* Y angle, raw */
  int16_t yaw; /* Z angle, raw */
} wit_imu_raw_frame_t;

BUILD_ASSERT(sizeof(wit_imu_raw_frame_t) == WIT_FRAME_LEN,
             "wit_imu_raw_frame_t must be 20 bytes");

typedef struct
{
  float ax, ay, az; /* acceleration (m/s²) */
  float wx, wy, wz; /* angular velocity (°/s) */
  float roll, pitch, yaw; /* angle (°) */
} wit_imu_data_t;

/**
 * @brief Parse a raw 20-byte WitMotion frame into engineering units.
 * @return true if frame header/flag valid, false otherwise.
 */
bool wit_imu_parse(const uint8_t *raw, size_t len, wit_imu_data_t *out);

/**
 * @brief Convert a raw int16_t sensor value to float.
 */
static inline float wit_imu_convert(int16_t raw, float range)
{
  return (float)raw / 32768.0f * range;
}

#ifdef __cplusplus
}
#endif

#endif /* WIT_IMU_H */
