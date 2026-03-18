#include "bsp_adc.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(bsp_adc, LOG_LEVEL_ERR);

/* Keep adc_dt_spec definitions */
const struct adc_dt_spec adc_ain0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
const struct adc_dt_spec adc_ain1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);

void bsp_adc_init(void)
{
    int err;

    if (!device_is_ready(adc_ain0.dev)) {
        LOG_ERR("ADC device not ready");
        return;
    }

    err = adc_channel_setup_dt(&adc_ain0);
    if (err) {
        LOG_ERR("adc_channel_setup_dt() failed: %d", err);
        return;
    }

    if (!device_is_ready(adc_ain1.dev)) {
        LOG_ERR("ADC device not ready");
        return;
    }

    err = adc_channel_setup_dt(&adc_ain1);
    if (err) {
        LOG_ERR("adc_channel_setup_dt() failed: %d", err);
        return;
    }
}

void bsp_adc_read_channels(void)
{
    bsp_adc_read_mV(&adc_ain0, &foot_sensor_data.mV_heel);
    bsp_adc_read_mV(&adc_ain1, &foot_sensor_data.mV_toe);
}

int bsp_adc_read_mV(const struct adc_dt_spec *adc, int32_t *raw)
{
	int err;
	struct adc_sequence seq = {
		.buffer = raw,
		.buffer_size = sizeof(*raw),
		.channels = BIT(adc->channel_id),
		.resolution = adc->resolution,
	};

	err = adc_read_dt(adc, &seq);
	if (err) {
		LOG_ERR("adc_read() failed: %d", err);
		return err;
	}

	err = adc_raw_to_millivolts_dt(adc, raw);
	if (err) {
		LOG_ERR("adc_raw_to_millivolts() failed: %d", err);
		return err;
	}

	return 0;
}