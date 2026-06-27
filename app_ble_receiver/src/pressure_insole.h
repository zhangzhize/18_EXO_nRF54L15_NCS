#ifndef PRESSURE_INSOLE_H
#define PRESSURE_INSOLE_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define INSOLE_FRAME_LEN 24
#define INSOLE_HEADER 0xAA
#define INSOLE_FOOTER 0x55
#define INSOLE_CHANNEL_COUNT 8

typedef struct __attribute__((packed)) pressure_insole_frame_t
{
  uint8_t header;
  uint8_t len;
  uint32_t id;
  int16_t ch[INSOLE_CHANNEL_COUNT];
  uint8_t checksum;
  uint8_t footer;
} pressure_insole_frame_t;

BUILD_ASSERT(sizeof(pressure_insole_frame_t) == INSOLE_FRAME_LEN,
             "pressure_insole_frame_t must be 24 bytes");

typedef struct
{
  uint32_t id;
  int16_t channels[INSOLE_CHANNEL_COUNT];
} pressure_insole_data_t;

bool pressure_insole_parse(const uint8_t *raw, size_t len, pressure_insole_data_t *out);

#ifdef __cplusplus
}
#endif

#endif /* PRESSURE_INSOLE_H */
