#include "pressure_insole.h"

#include <string.h>

bool pressure_insole_parse(const uint8_t *raw, size_t len, pressure_insole_data_t *out)
{
  if (raw == NULL || out == NULL || len < INSOLE_FRAME_LEN)
  {
    return false;
  }

  if (raw[0] != INSOLE_HEADER || raw[INSOLE_FRAME_LEN - 1] != INSOLE_FOOTER)
  {
    return false;
  }

  uint8_t checksum = 0U;
  for (size_t i = 0; i < INSOLE_FRAME_LEN - 2U; i++)
  {
    checksum += raw[i];
  }

  if (checksum != raw[INSOLE_FRAME_LEN - 2U])
  {
    return false;
  }

  const pressure_insole_frame_t *frame = (const pressure_insole_frame_t *)raw;

  out->id = frame->id;
  memcpy(out->channels, frame->ch, sizeof(out->channels));
  return true;
}
