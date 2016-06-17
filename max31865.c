/**************************************************************************
 * C driver library for the MAX31865.
 *
 * Work Copyright 2016 Magnus Lucchese <magnus.lucchese@gmail.com>
 *
 * This driver is based on Ole Wolf <wolf@blazingangles.com> arduino driver
 * 
 * The user should define the macros below
 * CS_ENABLE(cs_pin) 
 * CS_DISABLE(cs_pin)
 * SPI_WRITE(value)  
 * SPI_READ()
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
**************************************************************************/
#include <math.h>
#include "max31865/max31865.h"

void configure_device(struct Max31865Device *instance);
void set_threshold_values(struct Max31865Device *instance, uint16_t config_low_threshold, uint16_t config_high_threshold );
float calculate_temperature(float resistance, float resistance_0);

/**
 * The initializer for the MAX31865 device 
 *
 * @param [in] instance device instance
 * @param [in] config configuration for device
 */
void max31865_init(struct Max31865Device *instance,struct Max31865Config *config) {
    /* CS pin for the SPI device. */
    instance->cs_pin = config->cs_pin;

    /* Set the confgiuration register */
    uint8_t control_bits = 0;
    control_bits |= ( config->conversion_mode ? 0x40 : 0 );
    control_bits |= ( config->one_shot ? 0x20 : 0 );
    control_bits |= ( config->three_wire ? 0x10 : 0 );
    control_bits |= config->fault_cycle & 0b00001100;
    control_bits |= ( config->filter_50hz ? 0x01 : 0 );
    instance->config_control_bits = control_bits;

    /* Set the PTD. */
    instance->resistance_at_zero = config->resistance_at_zero;
    instance->resistance_reference = config->resistance_reference;

    /* Send configuration to device */
    configure_device(instance);

    /* Send threshold to device */
    set_threshold_values(instance, config->low_threshold, config->high_threshold);
}

/**
 * Enable/Disable Vbias for MAX31865
 *
 * @param [in] instance device instance
 * @param [in] enable true, turn on vbias, false, turn off vbias
 */
void max31865_set_vbias(struct Max31865Device *instance, bool enable) {
    if(enable) {
        SETBIT(instance->config_control_bits, 7);
    } else {
        CLEARBIT(instance->config_control_bits, 7);
    }
    configure_device(instance);
}

/**
 * Get default config values for MAX31865 device 
 *
 * @param [in] config configuration for device
 */
void max31865_get_config_default(struct Max31865Config *config) {
    config->cs_pin = 0;
    config->resistance_at_zero = 100;
    config->resistance_reference = 400;
    config->conversion_mode = true;
    config->one_shot = false;
    config->three_wire = false;
    config->fault_cycle = 0;
    config->filter_50hz = false;      
    config->low_threshold = 0;
    config->high_threshold = 32767;
}

/**
 * Get temperature value in oC for device
 *
 * @param [in] instance device instance
 * @param [out] temperature measured temperature
 * @return 0 if OK, fault register value if error
 */
uint8_t max31865_get_temperature(struct Max31865Device *instance, float *temperature) {
    uint16_t read_reg;
    float resistance;
    uint8_t fault_register = 0;

    /* Read resistance measured value */
    CS_ENABLE( instance->cs_pin);
    SPI_WRITE( REG_RTD_MSB );
    read_reg  = SPI_READ() << 8;
    read_reg |= SPI_READ();
    CS_DISABLE( instance->cs_pin);

    /* Check for fault */
    if ( TESTBIT(read_reg,0) ) {
        /* Fault: read fault register */
        CS_ENABLE( instance->cs_pin);
        SPI_WRITE( REG_FAULT_STATUS );
        fault_register = SPI_READ();
        CS_DISABLE( instance->cs_pin);

        /*Clear fault register */ 
        instance->config_control_bits |= 0x02;
        configure_device(instance);
        return fault_register;
    }
    read_reg = read_reg >> 1;

    resistance = (read_reg * instance->resistance_reference) / (1u << 15);
    *temperature = calculate_temperature(resistance, instance->resistance_at_zero);
    return 0;
}

/**
 * Apply the Callendar-Van Dusen equation to convert the RTD resistance
 * to temperature:
 * Tr = (-A + SQRT(delta) ) / 2*B
 * delta = A^2 - 4B*(1-Rt/Ro)
 * @param [in] resistance measured resistance
 * @param [in] resistance_0 constant resistance at 0oC
 * @return calculated temperature
 */
float calculate_temperature(float resistance, float resistance_0) {
    float temperature;
    float delta = (RTD_A * RTD_A) - 4 * RTD_B * (1.0 - resistance/resistance_0);
    temperature = (-RTD_A + sqrt(delta)) / (2 * RTD_B);
    return temperature;
}

/**
 * Set device configuration register
 *
 * @param [in] instance device instance
 */
void configure_device(struct Max31865Device *instance) {
  CS_ENABLE( instance->cs_pin);
  SPI_WRITE( WR(REG_CONFIG) );
  SPI_WRITE( instance->config_control_bits );
  CS_DISABLE( instance->cs_pin );
}

/**
 * Set device fail threshold registers
 *
 * @param [in] instance device instance
 * @param [in] config_low_threshold threshold for fail on low value
 * @param [in] config_high_threshold threshold for fail on high value
 */
void set_threshold_values(struct Max31865Device *instance, uint16_t config_low_threshold, uint16_t config_high_threshold ) {
  CS_ENABLE( instance->cs_pin);
  SPI_WRITE( WR(REG_HIGH_FAULT_THR_MSB) );
  SPI_WRITE( ( config_high_threshold >> 8 ) & 0x00ff );
  SPI_WRITE(   config_high_threshold        & 0x00ff );
  SPI_WRITE( ( config_low_threshold >> 8 ) & 0x00ff );
  SPI_WRITE(   config_low_threshold        & 0x00ff );
  CS_DISABLE( instance->cs_pin );
}


