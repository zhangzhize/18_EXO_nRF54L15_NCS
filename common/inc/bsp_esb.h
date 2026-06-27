#ifndef BSP_ESB_H
#define BSP_ESB_H

#include <errno.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if IS_ENABLED(CONFIG_ESB)

#include <esb.h>

#ifndef RF_CHANNEL
#define RF_CHANNEL 82
#endif

extern volatile bool ready;
extern struct esb_payload rx_payload;
extern struct esb_payload tx_payload;

int clocks_start(void);
int esb_initialize(void);

typedef void (*esb_rx_callback_t)(const uint8_t *data, size_t length);
void esb_set_rx_callback(esb_rx_callback_t callback);

#else

struct esb_payload
{
  uint8_t _unused;
};

extern bool ready;
extern struct esb_payload rx_payload;
extern struct esb_payload tx_payload;

static inline int clocks_start(void)
{
  return -ENOTSUP;
}

static inline int esb_initialize(void)
{
  return -ENOTSUP;
}

#endif /* CONFIG_ESB */

#ifdef __cplusplus
}
#endif

#endif /* BSP_ESB_H */
