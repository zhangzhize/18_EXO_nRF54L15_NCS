#include "bsp_ads1220.hpp"

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ads1220, LOG_LEVEL_WRN);

void ADS1220::drdy_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ADS1220 *instance = CONTAINER_OF(cb, ADS1220, drdy_cb_data_);
    k_sem_give(&instance->data_ready_sem_);
}

bool ADS1220::begin(const struct spi_dt_spec *spi_spec, const struct gpio_dt_spec *drdy_gpio_spec)
{
    spi_spec_ = spi_spec;
    drdy_gpio_spec_ = drdy_gpio_spec;

    if (spi_spec_ != NULL)
    {
        if (!spi_is_ready_dt(spi_spec_)) //TODO
            return false;
    }

    if (drdy_gpio_spec_ != NULL)
    {
        if (!gpio_is_ready_dt(drdy_gpio_spec_))
            return false;
        if (gpio_pin_configure_dt(drdy_gpio_spec_, GPIO_INPUT) != 0)
            return false;

        k_sem_init(&data_ready_sem_, 0, 1);
        if (gpio_pin_interrupt_configure_dt(drdy_gpio_spec_, GPIO_INT_EDGE_TO_ACTIVE) != 0)
            return false;
        gpio_init_callback(&drdy_cb_data_, drdy_isr_handler, BIT(drdy_gpio_spec_->pin));
        gpio_add_callback(drdy_gpio_spec_->port, &drdy_cb_data_);
    }

    k_msleep(1); // wait for power-on reset to complete (datasheet says we should do this)

    reset();
    start();
    uint8_t ctrl_val = 0;
    bypassPGA(true); // just a test if the ADS1220 is connected
    ctrl_val = readRegister(ADS1220_CONF_REG_0);
    bypassPGA(false);
    setNonBlockingMode(true);
    
    if (ctrl_val == 1)
    {
        // HACK: set some default settings for now
        setVRefSource(ADS1220_VREF_AVDD_AVSS);
        setVRefValue_V(3.3f);
        setGain(ADS1220_GAIN_128);
        setOperatingMode(ADS1220_TURBO_MODE);
        setDataRate(ADS1220_DR_LVL_6);
        setConversionMode(ADS1220_SINGLE_SHOT);
        return true;
    }
    else
    {
        return false;
    }
}

void ADS1220::start()
{
    command(ADS1220_START);
}

void ADS1220::reset()
{
    command(ADS1220_RESET);
    k_msleep(1);
}

void ADS1220::powerDown()
{
    command(ADS1220_PWRDOWN);
}

/* Configuration Register 0 settings */

void ADS1220::setCompareChannels(ads1220Mux mux)
{
    if((mux == ADS1220_MUX_REFPX_REFNX_4) || (mux == ADS1220_MUX_AVDD_M_AVSS_4))
    {
        gain_ = 1;    // under these conditions gain is one by definition 
        ref_measurement_ = true; 
    }
    else            // otherwise read gain from register
    {            
        reg_value_ = readRegister(ADS1220_CONF_REG_0);
        reg_value_ = reg_value_ & 0x0E;
        reg_value_ = reg_value_>>1;
        gain_ = 1 << reg_value_;     
        ref_measurement_ = false;
    }
    reg_value_ = readRegister(ADS1220_CONF_REG_0);
    reg_value_ &= ~0xF1;
    reg_value_ |= mux;
    reg_value_ |= !(do_not_bypass_PGA_if_possible_ & 0x01);
    writeRegister(ADS1220_CONF_REG_0, reg_value_);
    if ((mux >= 0x80) && (mux <= 0xD0))
    {
        if (gain_ > 4)
        {
            gain_ = 4;           // max gain is 4 if single-ended input is chosen or PGA is bypassed
        }
        forcedBypassPGA();
    }
}

void ADS1220::setGain(ads1220Gain enumGain)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_0);
    ads1220Mux mux = (ads1220Mux)(reg_value_ & 0xF0);
    reg_value_ &= ~0x0E;
    reg_value_ |= enumGain;
    writeRegister(ADS1220_CONF_REG_0, reg_value_);

    gain_ = 1 << (enumGain >> 1);
    if ((mux >= 0x80) && (mux <= 0xD0))
    {
        if (gain_ > 4)
        {
            gain_ = 4; // max gain is 4 if single-ended input is chosen or PGA is bypassed
        }
        forcedBypassPGA();
    }
}

uint8_t ADS1220::getGainFactor()
{
    return gain_;
}

void ADS1220::bypassPGA(bool bypass)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_0);
    reg_value_ &= ~0x01;
    reg_value_ |= bypass;
    do_not_bypass_PGA_if_possible_ = !(bypass & 0x01);
    writeRegister(ADS1220_CONF_REG_0, reg_value_);
}

bool ADS1220::isPGABypassed()
{
    reg_value_ = readRegister(ADS1220_CONF_REG_0);
    return reg_value_ & 0x01;
}

/* Configuration Register 1 settings */

void ADS1220::setDataRate(ads1220DataRate rate)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_1);
    reg_value_ &= ~0xE0;
    reg_value_ |= rate;
    writeRegister(ADS1220_CONF_REG_1, reg_value_);
}

void ADS1220::setOperatingMode(ads1220OpMode mode)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_1);
    reg_value_ &= ~0x18;
    reg_value_ |= mode;
    writeRegister(ADS1220_CONF_REG_1, reg_value_);
}

void ADS1220::setConversionMode(ads1220ConvMode mode)
{
    conv_mode_ = mode;
    reg_value_ = readRegister(ADS1220_CONF_REG_1);
    reg_value_ &= ~0x04;
    reg_value_ |= mode;
    writeRegister(ADS1220_CONF_REG_1, reg_value_);
}

void ADS1220::enableTemperatureSensor(bool enable)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_1);
    if (enable)
    {
        reg_value_ |= 0x02;
    }
    else
    {
        reg_value_ &= ~0x02;
    }
    writeRegister(ADS1220_CONF_REG_1, reg_value_);
}

void ADS1220::enableBurnOutCurrentSources(bool enable)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_1);
    if (enable)
    {
        reg_value_ |= 0x01;
    }
    else
    {
        reg_value_ &= ~0x01;
    }
    writeRegister(ADS1220_CONF_REG_1, reg_value_);
}

/* Configuration Register 2 settings */

void ADS1220::setVRefSource(ads1220VRef vRefSource)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_2);
    reg_value_ &= ~0xC0;
    reg_value_ |= vRefSource;
    writeRegister(ADS1220_CONF_REG_2, reg_value_);
}

void ADS1220::setFIRFilter(ads1220FIR fir)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_2);
    reg_value_ &= ~0x30;
    reg_value_ |= fir;
    writeRegister(ADS1220_CONF_REG_2, reg_value_);
}

void ADS1220::setLowSidePowerSwitch(ads1220PSW psw)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_2);
    reg_value_ &= ~0x08;
    reg_value_ |= psw;
    writeRegister(ADS1220_CONF_REG_2, reg_value_);
}

void ADS1220::setIdacCurrent(ads1220IdacCurrent current)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_2);
    reg_value_ &= ~0x07;
    reg_value_ |= current;
    writeRegister(ADS1220_CONF_REG_2, reg_value_);
    // k_usleep(200);
    k_busy_wait(200);
}

/* Configuration Register 3 settings */

void ADS1220::setIdac1Routing(ads1220IdacRouting route)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_3);
    reg_value_ &= ~0xE0;
    reg_value_ |= (route << 5);
    writeRegister(ADS1220_CONF_REG_3, reg_value_);
}

void ADS1220::setIdac2Routing(ads1220IdacRouting route)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_3);
    reg_value_ &= ~0x1C;
    reg_value_ |= (route << 2);
    writeRegister(ADS1220_CONF_REG_3, reg_value_);
}

void ADS1220::setDrdyMode(ads1220DrdyMode mode)
{
    reg_value_ = readRegister(ADS1220_CONF_REG_3);
    reg_value_ &= ~0x02;
    reg_value_ |= mode;
    writeRegister(ADS1220_CONF_REG_3, reg_value_);
}

/* Other settings */
void ADS1220::setVRefValue_V(float vRef)
{
    v_ref_ = vRef;
}

float ADS1220::getVRef_V()
{
    return v_ref_;
}

void ADS1220::setAvddAvssAsVrefAndCalibrate()
{
    bool last_blocking_mode = non_blocking_mode_;
    non_blocking_mode_ = false;
    float avss_voltage = 0.0;
    setVRefSource(ADS1220_VREF_AVDD_AVSS);
    setCompareChannels(ADS1220_MUX_AVDD_M_AVSS_4);
    for (int i = 0; i < 10; i++)
    {
        avss_voltage += getVoltage_mV();
    }
    v_ref_ = avss_voltage * 4.0f / 10000.0f;
    non_blocking_mode_ = last_blocking_mode;
}

void ADS1220::setRefp0Refn0AsVrefAndCalibrate()
{
    bool last_blocking_mode = non_blocking_mode_;
    non_blocking_mode_ = false;
    float ref0_voltage = 0.0;
    setVRefSource(ADS1220_VREF_REFP0_REFN0);
    setCompareChannels(ADS1220_MUX_REFPX_REFNX_4);
    for (int i = 0; i < 10; i++)
    {
        ref0_voltage += getVoltage_mV();
    }
    v_ref_ = ref0_voltage * 4.0f / 10000.0f;
    non_blocking_mode_ = last_blocking_mode;
}

void ADS1220::setRefp1Refn1AsVrefAndCalibrate()
{
    bool last_blocking_mode = non_blocking_mode_;
    non_blocking_mode_ = false;
    float ref1_voltage = 0.0;
    setVRefSource(ADS1220_VREF_REFP1_REFN1);
    setCompareChannels(ADS1220_MUX_REFPX_REFNX_4);
    for (int i = 0; i < 10; i++)
    {
        ref1_voltage += getVoltage_mV();
    }
    v_ref_ = ref1_voltage * 4.0f / 10000.0f;
    non_blocking_mode_ = last_blocking_mode;
}

void ADS1220::setIntVRef()
{
    setVRefSource(ADS1220_VREF_INT);
    v_ref_ = 2.048;
}

void ADS1220::setNonBlockingMode(bool nonBlocking)
{
    non_blocking_mode_ = nonBlocking;
}

bool ADS1220::getNonBlockingMode()
{
    return non_blocking_mode_;
} 

/* Results */
float ADS1220::getVoltage_mV()
{
    int32_t raw_data = getData();
    float result_in_mV = 0.0;
    if (ref_measurement_)
    {
        result_in_mV = (raw_data / ADS1220_RANGE) * 2.048f * 1000.0f / (gain_ * 1.0f);
    }
    else
    {
        result_in_mV = (raw_data / ADS1220_RANGE) * v_ref_ * 1000.0f / (gain_ * 1.0f);
    }
    return result_in_mV;
}

float ADS1220::getVoltage_uV()
{
    return getVoltage_mV() * 1000.0f;
}

int32_t ADS1220::getRawData()
{
    return getData();
}

float ADS1220::getTemperature()
{
    enableTemperatureSensor(true);
    uint32_t raw_result = readResult();
    enableTemperatureSensor(false);

    uint16_t result = static_cast<uint16_t>(raw_result >> 18);
    if (result >> 13)
    {
        result = ~(result - 1) & 0x3777;
        return result * (-0.03125f);
    }

    return result * 0.03125f;
}

void ADS1220::disableDrdyInterrupt()
{
    if (drdy_gpio_spec_ != nullptr && drdy_gpio_spec_->port != nullptr)
    {
        int ret = gpio_pin_interrupt_configure_dt(drdy_gpio_spec_, GPIO_INT_DISABLE);
        if (ret != 0)
        {
            LOG_ERR("Failed to disable DRDY interrupt, error: %d", ret);
        }
        else
        {
            LOG_INF("DRDY interrupt disabled successfully.");
        }
    }
}

/************************************************ 
    private functions
*************************************************/

void ADS1220::forcedBypassPGA()
{
    reg_value_ = readRegister(ADS1220_CONF_REG_0);
    reg_value_ |= 0x01;
    writeRegister(ADS1220_CONF_REG_0, reg_value_);
}

int32_t ADS1220::getData()
{
    uint32_t raw_result = readResult();
    int32_t result = (static_cast<int32_t>(raw_result)) >> 8;

    return result;
}

uint32_t ADS1220::readResult()
{
    uint8_t tx_buf[3] = {0x00, 0x00, 0x00};
    uint8_t rx_buf[3] = {0};
    uint32_t raw_result = 0;

    if (conv_mode_ == ADS1220_SINGLE_SHOT)
    {
        if (!non_blocking_mode_)
        {
            start();
        }
    }

    if (!non_blocking_mode_)
    {
        k_sem_take(&data_ready_sem_, K_MSEC(100));
    }
    
    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 3};
    struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = 3};
    struct spi_buf_set rx_set = {.buffers = &rx_spi_buf, .count = 1};
    
    spi_transceive_dt(spi_spec_, &tx_set, &rx_set);

    raw_result = rx_buf[0];
    raw_result = (raw_result << 8) | rx_buf[1];
    raw_result = (raw_result << 8) | rx_buf[2];
    raw_result = (raw_result << 8);

    return raw_result;
}

uint8_t ADS1220::readRegister(uint8_t reg)
{
    uint8_t tx_buf[2] = {static_cast<uint8_t>(ADS1220_RREG | (reg << 2)), 0x00};
    uint8_t rx_buf[2] = {0};

    reg_value_ = 0;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = 2};
    struct spi_buf_set rx_set = {.buffers = &rx_spi_buf, .count = 1};

    spi_transceive_dt(spi_spec_, &tx_set, &rx_set);

    reg_value_ = rx_buf[1];

    return reg_value_;
}

void ADS1220::writeRegister(uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2] = {static_cast<uint8_t>(ADS1220_WREG | (reg << 2)), val};
    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 2};
    struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

    spi_write_dt(spi_spec_, &tx_set);
}

void ADS1220::command(uint8_t cmd)
{
    uint8_t tx_buf[1] = {cmd};
    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = 1};
    struct spi_buf_set tx_set = {.buffers = &tx_spi_buf, .count = 1};

    spi_write_dt(spi_spec_, &tx_set);
}

