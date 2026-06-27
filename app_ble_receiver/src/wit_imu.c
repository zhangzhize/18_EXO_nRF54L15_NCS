#include "wit_imu.h"

bool wit_imu_parse(const uint8_t *raw, size_t len, wit_imu_data_t *out)
{
  if (raw == NULL || out == NULL || len < WIT_FRAME_LEN)
  {
    return false;
  }

  const wit_imu_raw_frame_t *frame = (const wit_imu_raw_frame_t *)raw;

  if (frame->header != WIT_FRAME_HEADER ||
      frame->flag != WIT_FRAME_FLAG_ACC_GYRO_ANGLE)
  {
    return false;
  }

  out->ax = wit_imu_convert(frame->ax, 16.0f * 9.8f);
  out->ay = wit_imu_convert(frame->ay, 16.0f * 9.8f);
  out->az = wit_imu_convert(frame->az, 16.0f * 9.8f);
  out->wx = wit_imu_convert(frame->wx, 2000.0f);
  out->wy = wit_imu_convert(frame->wy, 2000.0f);
  out->wz = wit_imu_convert(frame->wz, 2000.0f);
  out->roll = wit_imu_convert(frame->roll, 180.0f);
  out->pitch = wit_imu_convert(frame->pitch, 180.0f);
  out->yaw = wit_imu_convert(frame->yaw, 180.0f);

  return true;
}
