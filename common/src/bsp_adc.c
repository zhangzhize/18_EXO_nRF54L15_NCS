#include "bsp_adc.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(bsp_adc, LOG_LEVEL_ERR);

const struct adc_dt_spec adc_ain0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
const struct adc_dt_spec adc_ain1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);

int bsp_adc_init(void)
{
  int err;

  if (!device_is_ready(adc_ain0.dev))
  {
    LOG_ERR("ADC device not ready");
    return -ENODEV;
  }

  err = adc_channel_setup_dt(&adc_ain0);
  if (err)
  {
    LOG_ERR("adc_channel_setup_dt() failed: %d", err);
    return err;
  }

  if (!device_is_ready(adc_ain1.dev))
  {
    LOG_ERR("ADC device not ready");
    return -ENODEV;
  }

  err = adc_channel_setup_dt(&adc_ain1);
  if (err)
  {
    LOG_ERR("adc_channel_setup_dt() failed: %d", err);
    return err;
  }

  return 0;
}

void bsp_adc_read_channels(void)
{
  int32_t mV_heel = 0;
  int32_t mV_toe = 0;
  int heel_err;
  int toe_err;

  heel_err = bsp_adc_read_mV(&adc_ain0, &mV_heel);
  toe_err = bsp_adc_read_mV(&adc_ain1, &mV_toe);

  k_spinlock_key_t key = k_spin_lock(&exo_data_lock);
  if (heel_err == 0)
  {
    foot_sensor_data.mV_heel = mV_heel;
  }
  if (toe_err == 0)
  {
    foot_sensor_data.mV_toe = mV_toe;
  }
  k_spin_unlock(&exo_data_lock, key);
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
  if (err)
  {
    LOG_ERR("adc_read() failed: %d", err);
    return err;
  }

  err = adc_raw_to_millivolts_dt(adc, raw);
  if (err)
  {
    LOG_ERR("adc_raw_to_millivolts() failed: %d", err);
    return err;
  }

  return 0;
}
