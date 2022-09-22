////////////////////////////////////////////////////////////////////////////////
//  September 2022
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Updated by Inkyu Sa <enddl22@gmail.com> for ADIS16448AMLZ
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
//  Adapted by Thomas Mantel <thomas.mantel@mavt.ethz.ch> for use with versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.h
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides all the functions necessary to interface the
//  ADIS16448AMLZ IMU with an 8-Bit Atmel-based Arduino development board.
//  Functions for SPI configuration, reads and writes, and scaling are included.
//  This library may be used for the entire ADIS164XX family of devices with
//  some modification.
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free
//  Software Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for
//  more details.
//
//  You should have received a copy of the GNU Lesser Public License along with
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ADIS16448_h
#define ADIS16448_h
#include "Arduino.h"
#include "Imu.h"
#include <SPI.h>
#include <ros.h>

#define ADIS16448_FLASH_CNT  0x00 /* Flash memory write count */
#define ADIS16448_SUPPLY_OUT 0x02 /* Power supply measurement */
#define ADIS16448_XGYRO_OUT 0x04 /* X-axis gyroscope output */
#define ADIS16448_YGYRO_OUT 0x06 /* Y-axis gyroscope output */
#define ADIS16448_ZGYRO_OUT 0x08 /* Z-axis gyroscope output */
#define ADIS16448_XACCL_OUT 0x0A /* X-axis accelerometer output */
#define ADIS16448_YACCL_OUT 0x0C /* Y-axis accelerometer output */
#define ADIS16448_ZACCL_OUT 0x0E /* Z-axis accelerometer output */
#define ADIS16448_XMAGN_OUT 0x10 /* X-axis magnetometer measurement */
#define ADIS16448_YMAGN_OUT 0x12 /* Y-axis magnetometer measurement */
#define ADIS16448_ZMAGN_OUT 0x14 /* Z-axis magnetometer measurement */
#define ADIS16448_BARO_OUT	0x16 /* Barometric pressure output */
#define ADIS16448_TEMP_OUT  0x18 /* Temperature output */

/* Calibration parameters */
#define ADIS16448_XGYRO_OFF 0x1A /* X-axis gyroscope bias offset factor */
#define ADIS16448_YGYRO_OFF 0x1C /* Y-axis gyroscope bias offset factor */
#define ADIS16448_ZGYRO_OFF 0x1E /* Z-axis gyroscope bias offset factor */
#define ADIS16448_XACCL_OFF 0x20 /* X-axis acceleration bias offset factor */
#define ADIS16448_YACCL_OFF 0x22 /* Y-axis acceleration bias offset factor */
#define ADIS16448_ZACCL_OFF 0x24 /* Z-axis acceleration bias offset factor */
#define ADIS16448_XMAGN_HIF 0x26 /* X-axis magnetometer, hard-iron factor */
#define ADIS16448_YMAGN_HIF 0x28 /* Y-axis magnetometer, hard-iron factor */
#define ADIS16448_ZMAGN_HIF 0x2A /* Z-axis magnetometer, hard-iron factor */
#define ADIS16448_XMAGN_SIF 0x2C /* X-axis magnetometer, soft-iron factor */
#define ADIS16448_YMAGN_SIF 0x2E /* Y-axis magnetometer, soft-iron factor */
#define ADIS16448_ZMAGN_SIF 0x30 /* Z-axis magnetometer, soft-iron factor */

#define ADIS16448_GPIO_CTRL 0x32 /* Auxiliary digital input/output control */
#define ADIS16448_MSC_CTRL  0x34 /* Miscellaneous control */
#define ADIS16448_SMPL_PRD  0x36 /* Internal sample period (rate) control */
#define ADIS16448_SENS_AVG  0x38 /* Dynamic range and digital filter control */
#define ADIS16448_SLP_CNT   0x3A /* Sleep mode control */
#define ADIS16448_DIAG_STAT 0x3C /* System status */

/* Alarm functions */
#define ADIS16448_GLOB_CMD  0x3E /* System command */
#define ADIS16448_ALM_MAG1  0x40 /* Alarm 1 amplitude threshold */
#define ADIS16448_ALM_MAG2  0x42 /* Alarm 2 amplitude threshold */
#define ADIS16448_ALM_SMPL1 0x44 /* Alarm 1 sample size */
#define ADIS16448_ALM_SMPL2 0x46 /* Alarm 2 sample size */
#define ADIS16448_ALM_CTRL  0x48 /* Alarm control */
#define ADIS16448_AUX_DAC   0x4A /* Auxiliary DAC data */

#define ADIS16448_PRODUCT_ID 0x56 /* Product identifier */

#define ADIS16448_ERROR_ACTIVE			(1<<14)
#define ADIS16448_NEW_DATA			(1<<14)

/* MSC_CTRL */
#define ADIS16448_MSC_CTRL_MEM_TEST		(1<<11)
#define ADIS16448_MSC_CTRL_INT_SELF_TEST	(1<<10)
#define ADIS16448_MSC_CTRL_ACCL_ALIGN		(1<<6)
#define ADIS16448_MSC_CTRL_BURST_MODE_CRC (1<<4)
#define ADIS16448_MSC_CTRL_DATA_RDY_EN		(1<<2)
#define ADIS16448_MSC_CTRL_DATA_RDY_POL_HIGH	(1<<1)
#define ADIS16448_MSC_CTRL_DATA_RDY_DIO2	(1<<0)

/* DIAG_STAT */
#define ADIS16448_DIAG_STAT_ZACCL_FAIL	15
#define ADIS16448_DIAG_STAT_YACCL_FAIL	14
#define ADIS16448_DIAG_STAT_XACCL_FAIL	13
#define ADIS16448_DIAG_STAT_XGYRO_FAIL	12
#define ADIS16448_DIAG_STAT_YGYRO_FAIL	11
#define ADIS16448_DIAG_STAT_ZGYRO_FAIL	10
#define ADIS16448_DIAG_STAT_ALARM2	9
#define ADIS16448_DIAG_STAT_ALARM1	8
#define ADIS16448_DIAG_STAT_FLASH_CHK	6
#define ADIS16448_DIAG_STAT_SELF_TEST	5
#define ADIS16448_DIAG_STAT_OVERFLOW	4
#define ADIS16448_DIAG_STAT_SPI_FAIL	3
#define ADIS16448_DIAG_STAT_FLASH_UPT	2
#define ADIS16448_DIAG_STAT_POWER_HIGH	1
#define ADIS16448_DIAG_STAT_POWER_LOW	0

/* GLOB_CMD */
#define ADIS16448_GLOB_CMD_SW_RESET	                        (1<<7)
#define ADIS16448_GLOB_CMD_P_AUTO_NULL	                    (1<<4)
#define ADIS16448_GLOB_CMD_FLASH_UPD	                      (1<<3)
#define ADIS16448_GLOB_CMD_DAC_LATCH	                      (1<<2)
#define ADIS16448_GLOB_CMD_FAC_CALIB	                      (1<<1)
#define ADIS16448_GLOB_CMD_AUTO_NULL	                      (1<<0)

/* SMPL_PRD BITS */
#define ADIS16448_SMPL_PRD_NO_TAP_CFG                       (0<<8)
#define ADIS16448_SMPL_PRD_2_TAP_CFG	                      (1<<8)
#define ADIS16448_SMPL_PRD_4_TAP_CFG	                      (2<<8)
#define ADIS16448_SMPL_PRD_8_TAP_CFG                        (3<<8)
#define ADIS16448_SMPL_PRD_16_TAP_CFG                       (4<<8)

/* SENS_AVG BITS */
#define ADIS16448_GYRO_DYN_RANGE_1000_CFG                   (4<<8)
#define ADIS16448_GYRO_DYN_RANGE_500_CFG                    (2<<8)
#define ADIS16448_GYRO_DYN_RANGE_250_CFG                    (1<<8)

#define ADIS16448_FIR_NO_TAP_CFG                            (0<<0)
#define ADIS16448_FIR_2_TAP_CFG                             (1<<0)
#define ADIS16448_FIR_4_TAP_CFG                             (2<<0)
#define ADIS16448_FIR_8_TAP_CFG                             (3<<0)
#define ADIS16448_FIR_16_TAP_CFG                            (4<<0)
#define ADIS16448_FIR_32_TAP_CFG                            (5<<0)
#define ADIS16448_FIR_64_TAP_CFG                            (6<<0)
#define ADIS16448_FIR_128_TAP_CFG                           (7<<0)


// ADIS16448 Class Definition
class ADIS16448 : public Imu {
public:
  enum Adis16448Type {
    AMLZ,
    BMLZ
  };
  // ADIS16448 Constructor (ChipSelect, DataReady output pin, HardwareReset)
  ADIS16448(ros::NodeHandle *nh, const String &topic, Adis16448Type type, int CS, int DR, int RST);

  // Destructor
  ~ADIS16448();

  void setup();

  // Hardware reset sent to pin.
  int resetDUT(uint8_t ms);

  // Sets SPI bit order, clock divider, and data mode. Returns 1 when complete.
  int configSPI();

  // Read register (two bytes) Returns signed 16 bit data.
  int16_t regRead(uint8_t regAddr);

  // Burst read imu data excluding checksum.
  bool sensorReadAll(int16_t *sensorData);

  // Write register (two bytes). Returns 1 when complete.
  int regWrite(uint8_t regAddr, int16_t regData);

  // Modify register (two bytes). Returns 1 when complete.
  int regMod(uint8_t regAddr, uint16_t clearbits, uint16_t setbits);

  // Calculate CRC-16 Checksum.
  int16_t checksum(int16_t *sensorData);
  void updateCRC(unsigned int *crc, unsigned int *data,
                 const unsigned int &POLY);

  // Update data internally with validity checks.
  bool updateDataIterative();

  // Update data withoput recursion.
  bool updateData();

  //
  Adis16448Type getType();

  //
  static Imu::ImuType getImuType(ADIS16448::Adis16448Type adis16448_type);

private:
  // Variables to store hardware pin assignments.
  int CS_;
  int RST_;

  Adis16448Type type_;
};

#endif
