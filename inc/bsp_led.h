#ifndef __BSP_LED_H
#define __BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LED0_NODE DT_ALIAS(led0)

void bsp_led_init(void);
void bsp_led_toggle(void);
void bsp_led_on(void);
void bsp_led_off(void);

#ifdef __cplusplus
}
#endif

#endif