/*
  This is a library written for the TI ADS122C04
  24-Bit 4-Channel 2-kSPS Delta-Sigma ADC With I2C Interface

  It allows you to measure temperature very accurately using a
  Platinum Resistance Thermometer

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!

  Written by: Paul Clark (PaulZC)
  Date: May 4th 2020

  Based on the TI datasheet:
  https://www.ti.com/product/ADS122C04
  https://www.ti.com/lit/ds/symlink/ads122c04.pdf
  Using the example code from the "High Precision Temperature Measurement
  for Heat and Cold Meters Reference Design" (TIDA-01526) for reference:
  http://www.ti.com/tool/TIDA-01526
  http://www.ti.com/lit/zip/tidcee5

  The MIT License (MIT)
  Copyright (c) 2020 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SparkFun_ADS122C04_Library.hpp"
#include "math.h"

SFE_ADS122C04::SFE_ADS122C04(void)
{
    // Constructor
    k_sem_init(&data_ready_sem, 0, 1);
}

// Attempt communication with the device and initialise it
// Return true if successful
bool SFE_ADS122C04::begin(const struct i2c_dt_spec *i2c_dev, const struct gpio_dt_spec *drdy_gpio)
{
    _i2c_dev = i2c_dev; // Grab which I2C device the user wants us to use
    _drdy_gpio = drdy_gpio;
    _wireMode = ADS122C04_RAW_MODE; // Default to using 'safe' settings (disable the IDAC current sources)

    if (_i2c_dev != NULL)
    {
        if (!device_is_ready(_i2c_dev->bus))
            return false;
    }
    if (_drdy_gpio != NULL)
    {
        if (!device_is_ready(_drdy_gpio->port))
            return false;
        if (gpio_pin_configure_dt(_drdy_gpio, GPIO_INPUT) != 0)
        {
            return false;
        }
        if (gpio_pin_interrupt_configure_dt(_drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE) != 0)
        {
            return false;
        }
        gpio_init_callback(&drdy_cb_data, drdy_isr_handler, BIT(_drdy_gpio->pin));
        gpio_add_callback(_drdy_gpio->port, &drdy_cb_data);
    }

    k_msleep(1); // wait for power-on reset to complete (datasheet says we should do this)

    if (isConnected() == false)
    {
        return (false);
    }

    reset(); // reset the ADS122C04 (datasheet says we should do this)

    return (configureADCmode(ADS122C04_RAW_MODE)); // Default to using 'safe' settings (disable the IDAC current sources)
}

// Configure the chip for the selected wire mode
bool SFE_ADS122C04::configureADCmode(uint8_t wire_mode, uint8_t rate)
{
    ADS122C04_initParam initParams; // Storage for the chip parameters

    if (wire_mode == ADS122C04_4WIRE_MODE) // 4-wire mode
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;               // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_8;                     // Set the gain to 8
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // The PGA must be enabled for gains >= 8
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS;         // Use the external REF pins
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA;     // Set the IDAC current to 1mA
        initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3;                // Route IDAC1 to AIN3
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;            // Disable IDAC2
        _wireMode = ADS122C04_4WIRE_MODE;                            // Update the wire mode
    }
    else if (wire_mode == ADS122C04_4WIRE_HI_TEMP) // 4-wire mode for high temperatures (gain = 4)
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;               // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_4;                     // Set the gain to 4
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // Enable the PGA
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS;         // Use the external REF pins
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA;     // Set the IDAC current to 1mA
        initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3;                // Route IDAC1 to AIN3
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;            // Disable IDAC2
        _wireMode = ADS122C04_4WIRE_HI_TEMP;                         // Update the wire mode
    }
    else if (wire_mode == ADS122C04_3WIRE_MODE) // 3-wire mode
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;               // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_8;                     // Set the gain to 8
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // The PGA must be enabled for gains >= 8
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS;         // Use the external REF pins
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_500_UA;      // Set the IDAC current to 0.5mA
        initParams.routeIDAC1 = ADS122C04_IDAC1_AIN2;                // Route IDAC1 to AIN2
        initParams.routeIDAC2 = ADS122C04_IDAC2_AIN3;                // Route IDAC2 to AIN3
        _wireMode = ADS122C04_3WIRE_MODE;                            // Update the wire mode
    }
    else if (wire_mode == ADS122C04_3WIRE_HI_TEMP) // 3-wire mode for high temperatures (gain = 4)
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;               // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_4;                     // Set the gain to 4
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // Enable the PGA
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS;         // Use the external REF pins
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_500_UA;      // Set the IDAC current to 0.5mA
        initParams.routeIDAC1 = ADS122C04_IDAC1_AIN2;                // Route IDAC1 to AIN2
        initParams.routeIDAC2 = ADS122C04_IDAC2_AIN3;                // Route IDAC2 to AIN3
        _wireMode = ADS122C04_3WIRE_HI_TEMP;                         // Update the wire mode
    }
    else if (wire_mode == ADS122C04_2WIRE_MODE) // 2-wire mode
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;               // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_8;                     // Set the gain to 8
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // The PGA must be enabled for gains >= 8
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS;         // Use the external REF pins
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA;     // Set the IDAC current to 1mA
        initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3;                // Route IDAC1 to AIN3
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;            // Disable IDAC2
        _wireMode = ADS122C04_2WIRE_MODE;                            // Update the wire mode
    }
    else if (wire_mode == ADS122C04_2WIRE_HI_TEMP) // 2-wire mode for high temperatures (gain = 4)
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0;               // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_4;                     // Set the gain to 4
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // Enable the PGA
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_EXT_REF_PINS;         // Use the external REF pins
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_1000_UA;     // Set the IDAC current to 1mA
        initParams.routeIDAC1 = ADS122C04_IDAC1_AIN3;                // Route IDAC1 to AIN3
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;            // Disable IDAC2
        _wireMode = ADS122C04_2WIRE_HI_TEMP;                         // Update the wire mode
    }
    else if (wire_mode == ADS122C04_TEMPERATURE_MODE) // Internal temperature mode
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_1;       // Set the gain to 1
        initParams.pgaBypass = ADS122C04_PGA_DISABLED;
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_INTERNAL;             // Use the internal 2.048V reference
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_ON;          // Enable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF;         // Disable the IDAC current
        initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED;            // Disable IDAC1
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;            // Disable IDAC2
        _wireMode = ADS122C04_TEMPERATURE_MODE;                      // Update the wire mode
    }
    else if (wire_mode == ADS122C04_RAW_MODE) // Raw mode : disable the IDAC and use the internal reference
    {
        initParams.inputMux = ADS122C04_MUX_AIN1_AIN0; // Route AIN1 to AINP and AIN0 to AINN
        initParams.gainLevel = ADS122C04_GAIN_1;       // Set the gain to 1
        initParams.pgaBypass = ADS122C04_PGA_DISABLED;
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_NORMAL;                // Disable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_INTERNAL;             // Use the internal 2.048V reference
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;       // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF;         // Disable the IDAC current
        initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED;            // Disable IDAC1
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;            // Disable IDAC2
        _wireMode = ADS122C04_RAW_MODE;                              // Update the wire mode
    }
    else if (wire_mode == ADS122C04_BRIDGE_MODE)
    {
        initParams.inputMux = ADS122C04_MUX_AIN0_AIN1;               // Route AIN0 to AINP and AIN1 to AINN
        initParams.gainLevel = ADS122C04_GAIN_128;                   // Set the gain to 128
        initParams.pgaBypass = ADS122C04_PGA_ENABLED;                // The PGA must be enabled for gains >= 8
        initParams.dataRate = rate;                                  // Set the data rate (samples per second). Defaults to 20
        initParams.opMode = ADS122C04_OP_MODE_TURBO;                 // Enable turbo mode
        initParams.convMode = ADS122C04_CONVERSION_MODE_SINGLE_SHOT; // Use single shot mode
        initParams.selectVref = ADS122C04_VREF_AVDD;                 // Use AVDD as the reference
        initParams.tempSensorEn = ADS122C04_TEMP_SENSOR_OFF;         // Disable the temperature sensor
        initParams.dataCounterEn = ADS122C04_DCNT_DISABLE;           // Disable the data counter
        initParams.dataCRCen = ADS122C04_CRC_DISABLED;               // Disable CRC checking
        initParams.burnOutEn = ADS122C04_BURN_OUT_CURRENT_OFF;     // Disable the burn-out current
        initParams.idacCurrent = ADS122C04_IDAC_CURRENT_OFF;       // Disable the IDAC current
        initParams.routeIDAC1 = ADS122C04_IDAC1_DISABLED;          // Disable IDAC1
        initParams.routeIDAC2 = ADS122C04_IDAC2_DISABLED;          // Disable IDAC2
        _wireMode = ADS122C04_BRIDGE_MODE;                        // Update the wire mode
    }
    else
    {
        return (false);
    }
    return (ADS122C04_init(&initParams)); // Configure the chip
}

// Returns true if device answers on _deviceAddress
bool SFE_ADS122C04::isConnected(void)
{
    uint8_t val;
    return ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &val);
}

float SFE_ADS122C04::readPT100Centigrade(void) // Read the temperature in Centigrade
{
    raw_voltage_union raw_v; // union to convert uint32_t to int32_t
    float ret_val = 0.0;     // Return value
    float RTD, POLY;         // Variables needed to convert RTD to Centigrade

    // Start the conversion (assumes we are using single shot mode)
    start();

    // Wait for DRDY to go valid
    if (!wait_for_data())
    {
        return (ret_val);
    }

    // Read the conversion result
    if (ADS122C04_getConversionData(&raw_v.UINT32) == false)
    {
        return (ret_val);
    }

    // The raw voltage is in the bottom 24 bits of raw_temp
    // If we just do a <<8 we will multiply the result by 256
    // Instead pad out the MSB with the MS bit of the 24 bits
    // to preserve the two's complement
    if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    {
        raw_v.UINT32 |= 0xFF000000;
    }

    // raw_v.UINT32 now contains the ADC result, correctly signed
    // Now we need to convert it to temperature using the PT100 resistance,
    // the gain, excitation current and reference resistor value

    // Formulae are taken from:
    // http://www.ti.com/lit/an/sbaa275/sbaa275.pdf
    // https://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf

    // 2^23 is 8388608
    RTD = ((float)raw_v.INT32) / 8388608.0f; // Load RTD with the scaled ADC value
    RTD *= PT100_REFERENCE_RESISTOR;         // Multiply by the reference resistor
    // Use the correct gain for high and low temperatures
    if ((_wireMode == ADS122C04_4WIRE_HI_TEMP) || (_wireMode == ADS122C04_3WIRE_HI_TEMP) || (_wireMode == ADS122C04_2WIRE_HI_TEMP))
    {
        RTD /= PT100_AMP_GAIN_HI_TEMP; // Divide by the amplifier gain for high temperatures
    }
    else
    {
        RTD /= PT100_AMPLIFIER_GAIN; // Divide by the amplifier gain for low temperatures
    }
    if ((_wireMode == ADS122C04_3WIRE_MODE) || (_wireMode == ADS122C04_3WIRE_HI_TEMP)) // If we are using 3-wire mode
    {
        RTD *= 2.0f; // 3-wire mode needs a factor of 2
    }

    // RTD now contains the PT100 resistance in Ohms
    // Now we need to convert this to temperature

    // Calculate the temperature
    ret_val = RTD * -23.10e-9f;
    ret_val += 17.5848089e-6f;
    ret_val = sqrt(ret_val);
    ret_val -= 3.9083e-3f;
    ret_val /= -1.155e-6f;

    //  Check if the temperature is positive, return if it is
    if (ret_val >= 0.0f)
        return (ret_val);

    // The temperature is negative so we need to use a different formula
    ret_val = -242.02f;
    ret_val += 2.2228f * RTD;
    POLY = RTD * RTD; // Load the polynomial with RTD^2
    ret_val += 2.5859e-3f * POLY;
    POLY *= RTD; // Load the polynomial with RTD^3
    ret_val -= 4.8260e-6f * POLY;
    POLY *= RTD; // Load the polynomial with RTD^4
    ret_val -= 2.8183e-8f * POLY;
    POLY *= RTD; // Load the polynomial with RTD^5
    ret_val += 1.5243e-10f * POLY;

    return (ret_val);
}

float SFE_ADS122C04::readPT100Fahrenheit(void) // Read the temperature in Fahrenheit
{
    return ((readPT100Centigrade() * 1.8f) + 32.0f); // Read Centigrade and convert to Fahrenheit
}

float SFE_ADS122C04::readBridgeVoltage(void) // Read the bridge voltage in Volts
{
    raw_voltage_union raw_v; // union to convert uint32_t to int32_t
    float ret_val = 0.0;     // Return value

    const float ADS122C04_AVDD_VOLT = 3.0f; // AVDD voltage in Volts

    // Start the conversion (assumes we are using single shot mode)
    start();

    // Wait for DRDY to go valid
    if (!wait_for_data())
    {
        return (ret_val);
    }

    // Read the conversion result
    if (ADS122C04_getConversionData(&raw_v.UINT32) == false)
    {
        return (ret_val);
    }

    if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
    {
        raw_v.UINT32 |= 0xFF000000;
    }

    // The bridge voltage is in the bottom 24 bits of raw_v
    ret_val = ((float)raw_v.INT32 / 8388608.0f) * (ADS122C04_AVDD_VOLT / 128.0f) * 1000.0f; // Extract the 24-bit value

    return (ret_val);
}


// Read the raw signed 24-bit ADC value as int32_t
// The result needs to be multiplied by VREF / GAIN to convert to Volts
int32_t SFE_ADS122C04::readRawVoltage(uint8_t rate)
{
    raw_voltage_union raw_v;                                                          // union to convert uint32_t to int32_t
    uint8_t previousWireMode = _wireMode;                                             // Record the previous wire mode so we can restore it
    uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR;                                 // Record the previous rate so we can restore it
    bool configChanged = (_wireMode != ADS122C04_RAW_MODE) || (previousRate != rate); // Only change the configuration if we need to

    // Configure the ADS122C04 for raw mode
    // Disable the IDAC, use the internal 2.048V reference and set the gain to 1
    if (configChanged)
    {
        if ((configureADCmode(ADS122C04_RAW_MODE, rate)) == false)
        {
            return (0);
        }
    }

    // Start the conversion (assumes we are using single shot mode)
    start();

    // Wait for DRDY to go valid
    if (!wait_for_data())
    {
        if (configChanged)
            configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
        return (0);
    }

    // Read the conversion result
    if (ADS122C04_getConversionData(&raw_v.UINT32) == false)
    {
        if (configChanged)
            configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
        return (0);
    }

    // Restore the previous wire mode
    if (configChanged)
    {
        if ((configureADCmode(previousWireMode, previousRate)) == false)
        {
            return (0);
        }
    }

    // The raw voltage is in the bottom 24 bits of raw_temp
    // If we just do a <<8 we will multiply the result by 256
    // Instead pad out the MSB with the MS bit of the 24 bits
    // to preserve the two's complement
    if ((raw_v.UINT32 & 0x00800000) == 0x00800000)
        raw_v.UINT32 |= 0xFF000000;
    return (raw_v.INT32);
}

// Read the raw signed 24-bit ADC value as uint32_t
// The ADC data is returned in the least-significant 24-bits
// Higher functions will need to convert the result to (e.g.) int32_t
uint32_t SFE_ADS122C04::readADC(void)
{
    uint32_t ret_val; // The return value

    // Read the conversion result
    if (ADS122C04_getConversionData(&ret_val) == false)
    {
        return (0);
    }

    return (ret_val);
}

// Read the internal temperature
float SFE_ADS122C04::readInternalTemperature(uint8_t rate)
{
    internal_temperature_union int_temp;                                                      // union to convert uint16_t to int16_t
    uint32_t raw_temp;                                                                        // The raw temperature from the ADC
    float ret_val = 0.0;                                                                      // The return value
    uint8_t previousWireMode = _wireMode;                                                     // Record the previous wire mode so we can restore it
    uint8_t previousRate = ADS122C04_Reg.reg1.bit.DR;                                         // Record the previous rate so we can restore it
    bool configChanged = (_wireMode != ADS122C04_TEMPERATURE_MODE) || (previousRate != rate); // Only change the configuration if we need to

    // Enable the internal temperature sensor
    // Reading the ADC value will return the temperature
    if (configChanged)
    {
        if ((configureADCmode(ADS122C04_TEMPERATURE_MODE, rate)) == false)
        {
            return (ret_val);
        }
    }

    // Start the conversion
    start();

    // Wait for DRDY to go valid
    if (!wait_for_data())
    {
        if (configChanged)
            configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
        return (ret_val);
    }

    // Read the conversion result
    if (ADS122C04_getConversionData(&raw_temp) == false)
    {
        if (configChanged)
            configureADCmode(previousWireMode, previousRate); // Attempt to restore the previous wire mode
        return (ret_val);
    }

    // Restore the previous wire mode
    if (configChanged)
    {
        if ((configureADCmode(previousWireMode, previousRate)) == false)
        {
            return (ret_val);
        }
    }

    // The temperature is in the top 14 bits of the bottom 24 bits of raw_temp
    int_temp.UINT16 = (uint16_t)(raw_temp >> 10); // Extract the 14-bit value

    // The signed temperature is now in the bottom 14 bits of int_temp.UINT16
    // If we just do a <<2 we will multiply the result by 4
    // Instead we will pad out the two MS bits with the MS bit of the 14 bits
    // to preserve the two's complement
    if ((int_temp.UINT16 & 0x2000) == 0x2000) // Check if the MS bit is 1
    {
        int_temp.UINT16 |= 0xC000; // Value is negative so pad with 1's
    }
    else
    {
        int_temp.UINT16 &= 0x3FFF; // Value is positive so make sure the two MS bits are 0
    }

    ret_val = ((float)int_temp.INT16) * TEMPERATURE_SENSOR_RESOLUTION; // Convert to float including the 2 bit shift
    return (ret_val);
}

// Configure the input multiplexer
bool SFE_ADS122C04::setInputMultiplexer(uint8_t mux_config)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
        return (false);
    ADS122C04_Reg.reg0.bit.MUX = mux_config;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Configure the gain
bool SFE_ADS122C04::setGain(uint8_t gain_config)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
        return (false);
    ADS122C04_Reg.reg0.bit.GAIN = gain_config;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Enable/disable the Programmable Gain Amplifier
bool SFE_ADS122C04::enablePGA(uint8_t enable)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all)) == false)
        return (false);
    ADS122C04_Reg.reg0.bit.PGA_BYPASS = enable;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all));
}

// Set the data rate (sample speed)
bool SFE_ADS122C04::setDataRate(uint8_t rate)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
        return (false);
    ADS122C04_Reg.reg1.bit.DR = rate;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the operating mode (normal / turbo)
bool SFE_ADS122C04::setOperatingMode(uint8_t mode)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
        return (false);
    ADS122C04_Reg.reg1.bit.MODE = mode;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the conversion mode (single-shot / continuous)
bool SFE_ADS122C04::setConversionMode(uint8_t mode)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
        return (false);
    ADS122C04_Reg.reg1.bit.CMBIT = mode;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Configure the voltage reference
bool SFE_ADS122C04::setVoltageReference(uint8_t ref)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
        return (false);
    ADS122C04_Reg.reg1.bit.VREF = ref;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the internal temperature sensor
bool SFE_ADS122C04::enableInternalTempSensor(uint8_t enable)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all)) == false)
        return (false);
    ADS122C04_Reg.reg1.bit.TS = enable;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all));
}

// Enable / disable the conversion data counter
bool SFE_ADS122C04::setDataCounter(uint8_t enable)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
        return (false);
    ADS122C04_Reg.reg2.bit.DCNT = enable;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the data integrity check
bool SFE_ADS122C04::setDataIntegrityCheck(uint8_t setting)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
        return (false);
    ADS122C04_Reg.reg2.bit.CRCbits = setting;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Enable / disable the 10uA burn-out current source
bool SFE_ADS122C04::setBurnOutCurrent(uint8_t enable)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
        return (false);
    ADS122C04_Reg.reg2.bit.BCS = enable;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the internal programmable current sources
bool SFE_ADS122C04::setIDACcurrent(uint8_t current)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all)) == false)
        return (false);
    ADS122C04_Reg.reg2.bit.IDAC = current;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all));
}

// Configure the IDAC1 routing
bool SFE_ADS122C04::setIDAC1mux(uint8_t setting)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
        return (false);
    ADS122C04_Reg.reg3.bit.I1MUX = setting;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Configure the IDAC2 routing
bool SFE_ADS122C04::setIDAC2mux(uint8_t setting)
{
    if ((ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all)) == false)
        return (false);
    ADS122C04_Reg.reg3.bit.I2MUX = setting;
    return (ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all));
}

// Read Config Reg 2 and check the DRDY bit
// Data is ready when DRDY is high
bool SFE_ADS122C04::checkDataReady(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    return (ADS122C04_Reg.reg2.bit.DRDY > 0);
}

// Get the input multiplexer configuration
uint8_t SFE_ADS122C04::getInputMultiplexer(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
    return (ADS122C04_Reg.reg0.bit.MUX);
}

// Get the gain setting
uint8_t SFE_ADS122C04::getGain(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
    return (ADS122C04_Reg.reg0.bit.GAIN);
}

// Get the Programmable Gain Amplifier status
uint8_t SFE_ADS122C04::getPGAstatus(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_0_REG, &ADS122C04_Reg.reg0.all);
    return (ADS122C04_Reg.reg0.bit.PGA_BYPASS);
}

// Get the data rate (sample speed)
uint8_t SFE_ADS122C04::getDataRate(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    return (ADS122C04_Reg.reg1.bit.DR);
}

// Get the operating mode (normal / turbo)
uint8_t SFE_ADS122C04::getOperatingMode(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    return (ADS122C04_Reg.reg1.bit.MODE);
}

// Get the conversion mode (single-shot / continuous)
uint8_t SFE_ADS122C04::getConversionMode(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    return (ADS122C04_Reg.reg1.bit.CMBIT);
}

// Get the voltage reference configuration
uint8_t SFE_ADS122C04::getVoltageReference(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    return (ADS122C04_Reg.reg1.bit.VREF);
}

// Get the internal temperature sensor status
uint8_t SFE_ADS122C04::getInternalTempSensorStatus(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_1_REG, &ADS122C04_Reg.reg1.all);
    return (ADS122C04_Reg.reg1.bit.TS);
}

// Get the data counter status
uint8_t SFE_ADS122C04::getDataCounter(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    return (ADS122C04_Reg.reg2.bit.DCNT);
}

// Get the data integrity check configuration
uint8_t SFE_ADS122C04::getDataIntegrityCheck(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    return (ADS122C04_Reg.reg2.bit.CRCbits);
}

// Get the burn-out current status
uint8_t SFE_ADS122C04::getBurnOutCurrent(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    return (ADS122C04_Reg.reg2.bit.BCS);
}

// Get the IDAC setting
uint8_t SFE_ADS122C04::getIDACcurrent(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_2_REG, &ADS122C04_Reg.reg2.all);
    return (ADS122C04_Reg.reg2.bit.IDAC);
}

// Get the IDAC1 mux configuration
uint8_t SFE_ADS122C04::getIDAC1mux(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
    return (ADS122C04_Reg.reg3.bit.I1MUX);
}

// Get the IDAC2 mux configuration
uint8_t SFE_ADS122C04::getIDAC2mux(void)
{
    ADS122C04_readReg(ADS122C04_CONFIG_3_REG, &ADS122C04_Reg.reg3.all);
    return (ADS122C04_Reg.reg3.bit.I2MUX);
}

void SFE_ADS122C04::drdy_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    SFE_ADS122C04 *instance = CONTAINER_OF(cb, SFE_ADS122C04, drdy_cb_data);
    k_sem_give(&instance->data_ready_sem);
}

bool SFE_ADS122C04::wait_for_data(void)
{
    if (_drdy_gpio != NULL)
    {
        if (k_sem_take(&data_ready_sem, K_MSEC(ADS122C04_CONVERSION_TIMEOUT)) == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        int64_t start_time = k_uptime_get();
        while (k_uptime_get() - start_time < ADS122C04_CONVERSION_TIMEOUT)
        {
            if (checkDataReady())
            {
                return true;
            }
            k_msleep(1);
        }
        return false;
    }
}

// Update ADS122C04_Reg and initialise the ADS122C04 using the supplied parameters
bool SFE_ADS122C04::ADS122C04_init(ADS122C04_initParam *param)
{
    ADS122C04_Reg.reg0.all = 0; // Reset all four register values to the default value of 0x00
    ADS122C04_Reg.reg1.all = 0;
    ADS122C04_Reg.reg2.all = 0;
    ADS122C04_Reg.reg3.all = 0;

    ADS122C04_Reg.reg0.bit.MUX = param->inputMux;
    ADS122C04_Reg.reg0.bit.GAIN = param->gainLevel;
    ADS122C04_Reg.reg0.bit.PGA_BYPASS = param->pgaBypass;

    ADS122C04_Reg.reg1.bit.DR = param->dataRate;
    ADS122C04_Reg.reg1.bit.MODE = param->opMode;
    ADS122C04_Reg.reg1.bit.CMBIT = param->convMode;
    ADS122C04_Reg.reg1.bit.VREF = param->selectVref;
    ADS122C04_Reg.reg1.bit.TS = param->tempSensorEn;

    ADS122C04_Reg.reg2.bit.DCNT = param->dataCounterEn;
    ADS122C04_Reg.reg2.bit.CRCbits = param->dataCRCen;
    ADS122C04_Reg.reg2.bit.BCS = param->burnOutEn;
    ADS122C04_Reg.reg2.bit.IDAC = param->idacCurrent;

    ADS122C04_Reg.reg3.bit.I1MUX = param->routeIDAC1;
    ADS122C04_Reg.reg3.bit.I2MUX = param->routeIDAC2;

    bool ret_val = true; // Flag to show if the four writeRegs were successful
    // (If any one writeReg returns false, ret_val will be false)
    ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_0_REG, ADS122C04_Reg.reg0.all);
    ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_1_REG, ADS122C04_Reg.reg1.all);
    ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_2_REG, ADS122C04_Reg.reg2.all);
    ret_val &= ADS122C04_writeReg(ADS122C04_CONFIG_3_REG, ADS122C04_Reg.reg3.all);

    return (ret_val);
}

bool SFE_ADS122C04::reset(void)
{
    return (ADS122C04_sendCommand(ADS122C04_RESET_CMD));
}

bool SFE_ADS122C04::start(void)
{
    return (ADS122C04_sendCommand(ADS122C04_START_CMD));
}

bool SFE_ADS122C04::powerdown(void)
{
    return (ADS122C04_sendCommand(ADS122C04_POWERDOWN_CMD));
}

bool SFE_ADS122C04::ADS122C04_writeReg(uint8_t reg, uint8_t writeValue)
{
    uint8_t command = 0;
    command = ADS122C04_WRITE_CMD(reg);
    return (ADS122C04_sendCommandWithValue(command, writeValue));
}

bool SFE_ADS122C04::ADS122C04_readReg(uint8_t reg, uint8_t *readValue)
{
    uint8_t command = 0;
    command = ADS122C04_READ_CMD(reg);

    int ret = i2c_write_read_dt(_i2c_dev, &command, 1, readValue, 1);

    if (ret != 0)
    {
        return (false);
    }
    return (true);
}

bool SFE_ADS122C04::ADS122C04_sendCommand(uint8_t command)
{
    int ret = i2c_write_dt(_i2c_dev, &command, 1);
    return (ret == 0);
}

bool SFE_ADS122C04::ADS122C04_sendCommandWithValue(uint8_t command, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = command;
    buf[1] = value;

    int ret = i2c_write_dt(_i2c_dev, buf, 2);
    return (ret == 0);
}

// Read the conversion result with count byte.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
bool SFE_ADS122C04::ADS122C04_getConversionDataWithCount(uint32_t *conversionData, uint8_t *count)
{
    uint8_t command = ADS122C04_RDATA_CMD;
    uint8_t RXByte[4] = {0};

    // Write RDATA command, then read 4 bytes (Count + 3 Data)
    int ret = i2c_write_read_dt(_i2c_dev, &command, 1, RXByte, 4);

    if (ret != 0)
    {
        return false;
    }

    *count = RXByte[0];
    *conversionData = ((uint32_t)RXByte[1] << 16) | ((uint32_t)RXByte[2] << 8) | ((uint32_t)RXByte[3]);
    return true;
}

// Read the conversion result.
// The conversion result is 24-bit two's complement (signed)
// and is returned in the 24 lowest bits of the uint32_t conversionData.
// Hence it will always appear positive.
// Higher functions will need to take care of converting it to (e.g.) float or int32_t.
bool SFE_ADS122C04::ADS122C04_getConversionData(uint32_t *conversionData)
{
    uint8_t command = ADS122C04_RDATA_CMD;
    uint8_t RXByte[3] = {0};

    int ret = i2c_write_read_dt(_i2c_dev, &command, 1, RXByte, 3);
    if (ret != 0)
    {
        return false;
    }
    *conversionData = ((uint32_t)RXByte[0] << 16) | ((uint32_t)RXByte[1] << 8) | ((uint32_t)RXByte[2]);
    return true;
}
