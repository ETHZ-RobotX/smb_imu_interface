////////////////////////////////////////////////////////////////////////////////
//  May 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Updated by Inkyu Sa <enddl22@gmail.com> for ADIS16448AMLZ
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16448AMLZ.h
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

#ifndef ADIS16448AMLZ_h
#define ADIS16448AMLZ_h
#include "Arduino.h"
#include "Imu.h"
#include <SPI.h>
#include <ros.h>

// User Register Memory Map from Table 6
#define FLASH_CNT 0x00  // Flash memory write count
#define XGYRO_OUT 0x04  // X-axis gyroscope output
#define YGYRO_OUT 0x06  // Y-axis gyroscope output
#define ZGYRO_OUT 0x08  // Z-axis gyroscope output
#define XACCL_OUT 0x0A  // X-axis accelerometer output
#define YACCL_OUT 0x0C  // Y-axis accelerometer output
#define ZACCL_OUT 0x0E  // Z-axis accelerometer output
#define XMAGN_OUT 0X10  // X-axis magnetometer output
#define YMAGN_OUT 0x12  // Y-axis magnetometer output
#define ZMAGN_OUT 0x14  // Z-axis magnetometer output
#define BARO_OUT 0x16   // Barometer pressure measurement, high word
#define TEMP_OUT 0x18   // Temperature output
#define XGYRO_OFF 0x1A  // X-axis gyroscope bias offset factor
#define YGYRO_OFF 0x1C  // Y-axis gyroscope bias offset factor
#define ZGYRO_OFF 0x1E  // Z-axis gyroscope bias offset factor
#define XACCL_OFF 0x20  // X-axis acceleration bias offset factor
#define YACCL_OFF 0x22  // Y-axis acceleration bias offset factor
#define ZACCL_OFF 0x24  // Z-axis acceleration bias offset factor
#define XMAGN_HIC 0x26  // X-axis magnetometer, hard iron factor
#define YMAGN_HIC 0x28  // Y-axis magnetometer, hard iron factor
#define ZMAGN_HIC 0x2A  // Z-axis magnetometer, hard iron factor
#define XMAGN_SIC 0x2C  // X-axis magnetometer, soft iron factor
#define YMAGN_SIC 0x2E  // Y-axis magnetometer, soft iron factor
#define ZMAGN_SIC 0x30  // Z-axis magnetometer, soft iron factor
#define GPIO_CTRL 0x32  // GPIO control
#define MSC_CTRL 0x34   // MISC control
#define SMPL_PRD 0x36   // Sample clock/Decimation filter control
#define SENS_AVG 0x38   // Digital filter control
#define SEQ_CNT 0x3A    // MAGN_OUT and BARO_OUT counter
#define DIAG_STAT 0x3C  // System status
#define GLOB_CMD 0x3E   // System command
#define ALM_MAG1 0x40   // Alarm 1 amplitude threshold
#define ALM_MAG2 0x42   // Alarm 2 amplitude threshold
#define ALM_SMPL1 0x44  // Alarm 1 sample size
#define ALM_SMPL2 0x46  // Alarm 2 sample size
#define ALM_CTRL 0x48   // Alarm control
#define LOT_ID1 0x52    // Lot identification number
#define LOT_ID2 0x54    // Lot identification number
#define PROD_ID 0x56    // Product identifier
#define SERIAL_NUM 0x58 // Lot-specific serial number

#define BITS_SMPL_PRD_NO_TAP_CFG                       (0<<8)
#define BITS_SMPL_PRD_2_TAP_CFG	                       (1<<8)
#define BITS_SMPL_PRD_4_TAP_CFG	                       (2<<8)
#define BITS_SMPL_PRD_8_TAP_CFG                        (3<<8)
#define BITS_SMPL_PRD_16_TAP_CFG                       (4<<8)

#define BITS_GYRO_DYN_RANGE_1000_CFG                   (4<<8)
#define BITS_GYRO_DYN_RANGE_500_CFG                    (2<<8)
#define BITS_GYRO_DYN_RANGE_250_CFG                    (1<<8)

#define BITS_FIR_NO_TAP_CFG                            (0<<0)
#define BITS_FIR_2_TAP_CFG                             (1<<0)
#define BITS_FIR_4_TAP_CFG                             (2<<0)
#define BITS_FIR_8_TAP_CFG                             (3<<0)
#define BITS_FIR_16_TAP_CFG                            (4<<0)
#define BITS_FIR_32_TAP_CFG                            (5<<0)
#define BITS_FIR_64_TAP_CFG                            (6<<0)
#define BITS_FIR_128_TAP_CFG                           (7<<0)


// ADIS16448AMLZ Class Definition
class ADIS16448AMLZ : public Imu {
public:
  // ADIS16448AMLZ Constructor (ChipSelect, DataReady output pin, HardwareReset)
  ADIS16448AMLZ(ros::NodeHandle *nh, const String &topic, int CS, int DR, int RST);

  // Destructor
  ~ADIS16448AMLZ();

  void setup();

  // Hardware reset sent to pin.
  int resetDUT(uint8_t ms);

  // Sets SPI bit order, clock divider, and data mode. Returns 1 when complete.
  int configSPI();

  // Read register (two bytes) Returns signed 16 bit data.
  int16_t regRead(uint8_t regAddr);

  // Burst read imu data excluding checksum.
  int16_t *sensorReadWoCRC();

  // Write register (two bytes). Returns 1 when complete.
  int regWrite(uint8_t regAddr, int16_t regData);

  // Modify register (two bytes). Returns 1 when complete.
  int regMod(uint8_t regAddr, uint16_t clearbits, uint16_t setbits);

  // Calculate CRC-16 Checksum.
  int checksum(int16_t *sensorData);
  void updateCRC(unsigned int *crc, unsigned int *data,
                 const unsigned int &POLY);

  // Update data internally with validity checks.
  bool updateDataIterative();

  // Update data withoput recursion.
  bool updateData();

private:
  // Variables to store hardware pin assignments.
  int CS_;
  int RST_;
};

#endif
