/*
 * Baseado na biblioteca "Arduino driver library for the MAX31865."
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 *
 *
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno            -->  MAX31865
 *    ------------------------------------
 *    CS: any available pin  -->  CS
 *    MOSI: pin 11           -->  SDI (mandatory for hardware SPI)
 *    MISO: pin 12           -->  SDO (mandatory for hardware SPI)
 *    SCK: pin 13            -->  SCLK (mandatory for hardware SPI)
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
 */ 
#include <asf.h>
#include <stdint.h>
#include <math.h>
#include "max31865/max31865.h"
#include "temperature/temperature.h"
#include "rtc_ptc_spi/rtc_ptc_spi.h"

void reconfigure(Max31865Control *ptc);
double resistance(Max31865Control *ptc);
double temperature(Max31865Control *ptc);
uint8_t read_all(Max31865Control *ptc );
void wr_spi( uint8_t data);
uint8_t rd_spi(void);
void max318655_configure( PtcId id, bool v_bias, bool conversion_mode, bool one_shot,
                           bool three_wire, uint8_t fault_cycle, bool fault_clear,
                           bool filter_50hz, uint16_t low_threshold,
                           uint16_t high_threshold );
Max31865Control ptc_control[PTC_LAST] = {
   { .cs_pin = PTC_1_CS_OUTPUT_PIN, .type = RTD_PT100 },
   { .cs_pin = PTC_2_CS_OUTPUT_PIN, .type = RTD_PT100 },
   { .cs_pin = PTC_3_CS_OUTPUT_PIN, .type = RTD_PT100 },
   { .cs_pin = PTC_4_CS_OUTPUT_PIN, .type = RTD_PT100 },
   { .cs_pin = PTC_5_CS_OUTPUT_PIN, .type = RTD_PT100 },
   { .cs_pin = PTC_6_CS_OUTPUT_PIN, .type = RTD_PT100 },
};

Status Max31865GetTemperature(Temperature *temp, PtcId id, bool *failed) {
   Max31865Control *ptc;
   ptc = &ptc_control[id];
   max318655_configure(id, true, false,true,true, 0x00, true, false,0,0);
   vTaskDelay(10);
   read_all(ptc);
   *failed = false;
   *temp = temperature(ptc);
   return SUCCESS;
}

/**
 * Configure the MAX31865.  The parameters correspond to Table 2 in the MAX31865
 * datasheet.  The parameters are combined into a control bit-field that is stored
 * internally in the class for later reconfiguration, as are the fault threshold values.
 *
 * @param [in] v_bias Vbias enabled (@a true) or disabled (@a false).
 * @param [in] conversion_mode Conversion mode auto (@a true) or off (@a false).
 * @param [in] one_shot 1-shot measurement enabled (@a true) or disabled (@a false).
 * @param [in] three_wire 3-wire enabled (@a true) or 2-wire/4-wire (@a false).
 * @param [in] fault_detection Fault detection cycle control (see Table 3 in the MAX31865
 *             datasheet).
 * @param [in] fault_clear Fault status auto-clear (@a true) or manual clear (@a false).
 * @param [in] filter_50hz 50 Hz filter enabled (@a true) or 60 Hz filter enabled
 *             (@a false).
 * @param [in] low_threshold Low fault threshold.
 * @param [in] high_threshold High fault threshold.
*/
void max318655_configure( PtcId id, bool v_bias, bool conversion_mode, bool one_shot,
                          bool three_wire, uint8_t fault_cycle, bool fault_clear,
                          bool filter_50hz, uint16_t low_threshold,
                          uint16_t high_threshold ) {
  
  uint8_t control_bits = 0;

  /* Assemble the control bit mask. */
  control_bits |= ( v_bias ? 0x80 : 0 );
  control_bits |= ( conversion_mode ? 0x40 : 0 );
  control_bits |= ( one_shot ? 0x20 : 0 );
  control_bits |= ( three_wire ? 0x10 : 0 );
  control_bits |= fault_cycle & 0b00001100;
  control_bits |= ( fault_clear ? 0x02 : 0 );
  control_bits |= ( filter_50hz ? 0x01 : 0 );

  /* Store the control bits and the fault threshold limits for reconfiguration
     purposes. */
  Max31865Control *ptc = &ptc_control[id];
  ptc->configuration_control_bits   = control_bits;
  ptc->configuration_low_threshold  = low_threshold;
  ptc->configuration_high_threshold = high_threshold;

  /* Perform an initial "reconfiguration." */
  reconfigure(ptc);
}

/**
 * Reconfigure the MAX31865 by writing the stored control bits and the stored fault
 * threshold values back to the chip.
 */ 
void reconfigure(Max31865Control *ptc) {
   GetRtcSpiPort();

   /* Write the configuration to the MAX31865. */
   SelectRtcSpi( ptc->cs_pin, true );
   wr_spi( 0x80 );
   wr_spi( ptc->configuration_control_bits );
   SelectRtcSpi( ptc->cs_pin, false );


   /* Write the threshold values. */
   SelectRtcSpi( ptc->cs_pin, true );
   
   wr_spi( 0x83 );
   wr_spi( ( ptc->configuration_high_threshold >> 8 ) & 0x00ff );
   wr_spi(   ptc->configuration_high_threshold        & 0x00ff );
   wr_spi( ( ptc->configuration_low_threshold >> 8 )  & 0x00ff );
   wr_spi(   ptc->configuration_low_threshold         & 0x00ff );
   SelectRtcSpi( ptc->cs_pin, false );
   ReleaseRtcSpiPort();
}

double resistance(Max31865Control *ptc) {
   double rtd_rref = ( ptc->type == RTD_PT100 ) ? (double)RTD_RREF_PT100 : (double)RTD_RREF_PT1000;
   return( (double)ptc->measured_resistance * rtd_rref / (double)RTD_ADC_RESOLUTION );
}

/**
 * Apply the Callendar-Van Dusen equation to convert the RTD resistance
 * to temperature:
 *
 *   \f[
 *   t=\frac{-A\pm \sqrt{A^2-4B\left(1-\frac{R_t}{R_0}\right)}}{2B}
 *   \f],
 *
 * where
 *
 * \f$A\f$ and \f$B\f$ are the RTD coefficients, \f$R_t\f$ is the current
 * resistance of the RTD, and \f$R_0\f$ is the resistance of the RTD at 0
 * degrees Celsius.
 *
 * For more information on measuring with an RTD, see:
 * <http://newton.ex.ac.uk/teaching/CDHW/Sensors/an046.pdf>.
 *
 * @param [in] resistance The measured RTD resistance.
 * @return Temperature in degrees Celsius.
 */
double temperature(Max31865Control *ptc) {
  static const double a2   = 2.0 * RTD_B;
  static const double b_sq = RTD_A * RTD_A;

  const double rtd_resistance =
    ( ptc->type == RTD_PT100 ) ? RTD_RESISTANCE_PT100 : RTD_RESISTANCE_PT1000;

  double c = 1.0 - resistance(ptc) / rtd_resistance;
  double D = b_sq - 2.0 * a2 * c;
  double temperature_deg_C = ( -RTD_A + sqrt( D ) ) / a2;

  return( temperature_deg_C );
}

/**
 * Read all settings and measurements from the MAX31865 and store them
 * internally in the class.
 *
 * @return Fault status byte
 */
uint8_t read_all(Max31865Control *ptc ) {
   uint16_t combined_bytes;
   GetRtcSpiPort();
  
   /* Start the read operation. */
   SelectRtcSpi( ptc->cs_pin, true );
  
   /* Tell the MAX31865 that we want to read, starting at register 0. */
   wr_spi( 0x00 );

   /* Read the MAX31865 registers in the following order:
         Configuration
         RTD
         High Fault Threshold
         Low Fault Threshold
         Fault Status */
   ptc->measured_configuration = rd_spi();
  
  
   combined_bytes  = rd_spi() << 8;
   combined_bytes |= rd_spi();
   ptc->measured_resistance = combined_bytes >> 1;
   
   combined_bytes  = rd_spi() << 8;
   combined_bytes |= rd_spi();
   ptc->measured_high_threshold = combined_bytes >> 1;

   combined_bytes  = rd_spi() << 8;
   combined_bytes |= rd_spi();
   ptc->measured_low_threshold = combined_bytes >> 1;

   ptc->measured_status = rd_spi();

   SelectRtcSpi( ptc->cs_pin, false );
   ReleaseRtcSpiPort();

   /* Reset the configuration if the measured resistance is
      zero or a fault occurred. */
   if(    ( ptc->measured_resistance == 0 )
      || ( ptc->measured_status != 0 ) ) {
      reconfigure( ptc );
   }

   return ptc->measured_status;
}

void wr_spi( uint8_t data) {
   WriteRtcPtcSpi(&data, 1);
}

uint8_t rd_spi(void) {
   uint8_t value_read;
   ReadRtcPtcSpi(&value_read, 1);
   return value_read;
}