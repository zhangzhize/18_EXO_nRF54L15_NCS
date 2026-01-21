#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <zephyr/drivers/adc.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified for ADC channels"
#endif

extern const struct adc_dt_spec adc_ain0;
extern const struct adc_dt_spec adc_ain1;
extern int32_t mV_ain0;
extern int32_t mV_ain1;

#ifndef BSP_ADC_SAMPLING_US
#define BSP_ADC_SAMPLING_US 2000U /* default 2000 us */
#endif

void bsp_adc_init(void);
void bsp_adc_read_channels(void);
int bsp_adc_read_mV(const struct adc_dt_spec *adc, int32_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */