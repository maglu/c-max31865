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
#ifndef _MAX31865_H
#define _MAX31865_H

#include <stdint.h>
#include <stdbool.h>

#warning "Re-define the macros below acording to your hardware"
#include <asf.h>
extern struct spi_module spi_master_instance;

static inline uint8_t my_spi_read_(void) {
   uint16_t rx_data = 0;
   spi_read(&spi_master_instance, &rx_data);
   return (uint8_t) rx_data;
}

#define CS_ENABLE(cs_pin)        do { port_pin_set_output_level(cs_pin, false); } while (0) /**< Chip select enable macro  */
#define CS_DISABLE(cs_pin)       do { port_pin_set_output_level(cs_pin, true); } while (0)  /**< Chip select disable macro */
#define SPI_WRITE(value)         do { spi_write(&spi_master_instance, value); } while (0)   /**< SPI Write one byte macro  */
#define SPI_READ()               my_spi_read_()                                             /**< SPI Read one byte macro   */


#define MAX31865_FAULT_HIGH_THRESHOLD  ( 1 << 7 )
#define MAX31865_FAULT_LOW_THRESHOLD   ( 1 << 6 )
#define MAX31865_FAULT_REFIN           ( 1 << 5 )
#define MAX31865_FAULT_REFIN_FORCE     ( 1 << 4 )
#define MAX31865_FAULT_RTDIN_FORCE     ( 1 << 3 )
#define MAX31865_FAULT_VOLTAGE         ( 1 << 2 )

#define MAX31865_FAULT_DETECTION_NONE      ( 0x00 << 2 )
#define MAX31865_FAULT_DETECTION_AUTO      ( 0x01 << 2 )
#define MAX31865_FAULT_DETECTION_MANUAL_1  ( 0x02 << 2 )
#define MAX31865_FAULT_DETECTION_MANUAL_2  ( 0x03 << 2 )

/* Read Register Address */
#define REG_CONFIG                  0x00
#define REG_RTD_MSB                 0x01
#define REG_RTD_LSB                 0x02
#define REG_HIGH_FAULT_THR_MSB      0x03
#define REG_HIGH_FAULT_THR_LSB      0x04
#define REG_LOW_FAULT_THR_MSB       0x05
#define REG_LOW_FAULT_THR_LSB       0x06
#define REG_FAULT_STATUS            0x07
#define WR(reg)                     ( (reg) | 0x80 )

/* RTD data, RTD current, and measurement reference
   voltage. The ITS-90 standard is used; other RTDs
   may have coefficients defined by the DIN 43760 or
   the U.S. Industrial (American) standard. */

#define RTD_A_ITS90         3.9080e-3
#define RTD_A_USINDUSTRIAL  3.9692e-3
#define RTD_A_DIN43760      3.9848e-3
#define RTD_B_ITS90         -5.870e-7
#define RTD_B_USINDUSTRIAL  -5.8495e-7
#define RTD_B_DIN43760      -5.8019e-7

/* RTD coefficient C is required only for temperatures
   below 0 deg. C.  The selected RTD coefficient set
   is specified below. */
//#define SELECT_RTD_HELPER(x) x
//#define SELECT_RTD(x) SELECT_RTD_HELPER(x)
#define RTD_A         (RTD_A_ITS90)
#define RTD_B         (RTD_B_ITS90)

/**
 * Device instance control struct
 */
struct Max31865Device {
  uint8_t  cs_pin;
  uint8_t  config_control_bits;
  uint16_t resistance_at_zero;
  uint16_t resistance_reference;
};

/**
 * Configuration struct to the MAX31865.
 */
struct Max31865Config {
    uint8_t  cs_pin;                /**< Chip select pin */
    uint16_t resistance_at_zero;    /**< Sensor resistance at 0 Celsius (100 for PT100, 1000 for PT1000). */
    uint16_t resistance_reference;  /**< Circuit reference resistance (recommended on MAX31865 datasheet: 400 ohms for PT100, 4000 for PT1000).*/
    bool conversion_mode;           /**< Conversion_mode Conversion mode auto (@a true) or off (@a false). */
    bool one_shot;                  /**< One_shot 1-shot measurement enabled (@a true) or disabled (@a false). */
    bool three_wire;                /**< Three_wire 3-wire enabled (@a true) or 2-wire/4-wire (@a false). */
    uint8_t fault_cycle;            /**< Fault_detection Fault detection cycle control (see Table 3 in the MAX31865 datasheet). */
    bool filter_50hz;               /**< Filter_50hz 50 Hz filter enabled (@a true) or 60 Hz filter enabled (@a false). */
    uint16_t low_threshold;         /**< Low fault threshold. */
    uint16_t high_threshold;        /**< High fault threshold. */
};

/* Bit manipulation macros */
#define BIT(pos)           (1 << (pos))
#define SETBIT(x,y)        (x) |= BIT(y)
#define CLEARBIT(x,y)      (x) &= ~BIT(y)
#define TESTBIT(data,pos)  ((0u == (data & BIT(pos)))?0u:1u)

void max31865_init(struct Max31865Device *instance,struct Max31865Config *config);
void max31865_set_vbias(struct Max31865Device *instance, bool enable);
uint8_t max31865_get_temperature(struct Max31865Device *instance, float *temperature);
void max31865_get_config_default(struct Max31865Config *config);
#endif /* _MAX31865_H */
