////////////////////////////////////////////////////////////////////////////////
//  September 2022
//  Author: Juan Jose Chong <juan.chong@analog.com>
//  Updated by Inkyu Sa <enddl22@gmail.com> for ADIS16448BMLZ
//  Adapted by Florian Tschopp <ftschopp@ethz.ch> for use in versavis
//  Adapted by Thomas Mantel <thomas.mantel@mavt.ethz.ch> for use with versavis
////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.cpp
////////////////////////////////////////////////////////////////////////////////
//
//  This library provides all the functions necessary to interface the
//  ADIS16448BMLZ IMU with an 8-Bit Atmel-based Arduino development board.
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

#include "ADIS16448.h"
#include "helper.h"
#include "versavis_configuration.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16448::ADIS16448(ros::NodeHandle *nh, const String &topic, 
                             Adis16448Type type, int CS, int DR, int RST)
    : Imu(nh, topic, Imu::ImuType::ADIS16448BMLZ), CS_(CS), RST_(RST), type_(type) {
  if (DR >= 0) {
    pinMode(DR, INPUT); // Set DR pin to be an input
  }

  if (RST_ >= 0) {
    pinMode(RST_, OUTPUT);    // Set RST pin to be an output
    digitalWrite(RST_, HIGH); // Initialize RST pin to be high
  }
}

void ADIS16448::setup() {
  DEBUG_PRINTLN((topic_ + " (ADIS16448.cpp): Setup.").c_str());
  SPI.begin(); // Initialize SPI bus
  configSPI(); // Configure SPI

  // Reset IMU.
  regWrite(ADIS16448_GLOB_CMD, 0xBE80); // Perform an IMU reset.
  delay(300);
  regWrite(ADIS16448_GLOB_CMD, 0xBE00); // Take IMU out of reset state.
  delay(100);
  configSPI(); // Configure SPI
  delay(100);

  // Set default pin states
  pinMode(CS_, OUTPUT);    // Set CS pin to be an output
  digitalWrite(CS_, HIGH); // Initialize CS pin to be high

  /* ---------------- Perform IMU configuration  --------------------------- */
  //===========================================================================
  //          MSC_CTRL (16bits Default=0x0006, B0000 0000 0000 0110)
  // Bits    : Description
  // 15-12   : Not used
  // 11      : Checksum mem test(0/1=off/on)
  // 10      : Internal self test, (cleared (low) upon completion)
  // 9-8     : Do not use
  // 7       : Not used
  // 6       : Point of percussion (0/1=off/on)
  // 5       : Not used
  // 4       : CRC-16 code for burst mode (0/1=off/on)
  // 3       : Not used
  // 2       : Data ready enable (0/1=off/on)
  // 1       : Data ready polarity (0/1=active low/high when data is valid_
  // 0       : Data ready line select (0=DIO1, 1=DIO2)
  //===========================================================================

  // Enable data ready. 
  int16_t kMscCtrlRegister = ADIS16448_MSC_CTRL_DATA_RDY_EN; 
  // Enable CRC (CRC is only valid for 16448BMLZ not for AMLZ..)
  if (getType() == Adis16448Type::BMLZ) {
    kMscCtrlRegister |= ADIS16448_MSC_CTRL_BURST_MODE_CRC;
    DEBUG_PRINTLN("Activating CRC for burst read");
  }
  regWrite(ADIS16448_MSC_CTRL, kMscCtrlRegister);
  delay(20);

  //===========================================================================
  //          SMPL_PRD (16bits, default=0x0001
  //      15-13    |         12-8           |        7-1    |        0
  //      Not-used |  D, decimation rate    | Not used      | Clock, 0= ext,
  //      1=internal
  //===========================================================================
  // internal clock,
  int16_t kSmplPrdRegister = 0x0001; // B0000 0000 0000 0001;
  // decimation rate D=2 (reduce sample rate by factor of 2^2 = 4)
  kSmplPrdRegister |= ADIS16448_SMPL_PRD_4_TAP_CFG; 
  regWrite(ADIS16448_SMPL_PRD, kSmplPrdRegister);
  delay(20);

  //===========================================================================
  //          SENS_AVG (16bits, default= 0x0402, B0000 0100 0000 0010)
  // Bits    : Description
  // 15-11   : Not-used
  // 10-8    : Measurement range selection,
  //          100=±1000°/sec default) 010=±500°/sec 001=±250°/sec
  // 7-3     : Not used
  // 2-0     : Filter Size variable B, Number of taps in each stage; N_B=2^B
  //===========================================================================
  // ±1000°/sec, note that the lower dynamic range settings limit the
  int16_t kSensAvgRegister = ADIS16448_GYRO_DYN_RANGE_1000_CFG;
  // no digital filtering
  kSensAvgRegister |= ADIS16448_FIR_NO_TAP_CFG;
  regWrite(ADIS16448_SENS_AVG, kSensAvgRegister);
  delay(20);

#if defined(ASL_ADIS_ADAPTOR_BOARD)
  // Turn on ADIS16448 adaptor board LED
  // (by setting DIO2 on ADIS16448)
  regMod(ADIS16448_GPIO_CTRL, 0x0200, 0x0002);
  delay(20);
#endif

  Imu::setupPublisher();
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16448::~ADIS16448() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting RST_ pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
int ADIS16448::resetDUT(uint8_t ms) {
  if (RST_ == -1) {
    return -1;
  } else {
    digitalWrite(RST_, LOW);
    delay(ms);
    digitalWrite(RST_, HIGH);
    delay(20);
    return (1);
  }
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16448::configSPI() {
  SPI.setBitOrder(MSBFIRST);            // Per the datasheet
  SPI.setClockDivider(SPI_CLOCK_DIV16); // Config for 1MHz (ADIS16448BMLZ max
                                        // 2MHz, burst read max 1MHz)
  SPI.setDataMode(SPI_MODE3); // Clock base at one, sampled on falling edge
  return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16448::regRead(uint8_t regAddr) {
  // Read registers using SPI

  // Write register address to be read
  digitalWrite(CS_, LOW); // Set CS low to enable device
  SPI.transfer(regAddr);  // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement
  digitalWrite(CS_, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)

  // Read data from requested register
  digitalWrite(CS_, LOW); // Set CS low to enable device
  uint8_t _msbData =
      SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData =
      SPI.transfer(0x00);  // Send (0x00) and place lower byte into variable
  digitalWrite(CS_, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)

  int16_t _dataOut =
      (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return (_dataOut);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads all gyro, accel, magn, baro, and tmp registers in one instance (faster)
// including CRC-16 Checksum (if ADIS16448BMLZ)
////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - (pointer) array of signed 16 bit 2's complement numbers
// return - bool true on success
////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16448::sensorReadAll(int16_t *sensorData) {
  // Read registers using SPI
  // Initialize sensor raw data array.
  uint8_t rawData[26] = {0};

  // Write each requested register address and read back it's data
  digitalWrite(CS_, LOW); // Set CS low to enable communication with the device
  SPI.transfer(ADIS16448_GLOB_CMD); // Initial SPI read. Returned data for this transfer
                          // is invalid
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction
                      // requirement

  // DIAG_STAT
  rawData[0] =
      SPI.transfer(0x00); // Write next address to device and read upper byte
  rawData[1] = SPI.transfer(0x00); // Read lower byte
  // XGYRO_OUT
  rawData[2] = SPI.transfer(0x00);
  rawData[3] = SPI.transfer(0x00);
  // YGYRO_OUT
  rawData[4] = SPI.transfer(0x00);
  rawData[5] = SPI.transfer(0x00);
  // ZGYRO_OUT
  rawData[6] = SPI.transfer(0x00);
  rawData[7] = SPI.transfer(0x00);
  // XACCL_OUT
  rawData[8] = SPI.transfer(0x00);
  rawData[9] = SPI.transfer(0x00);
  // YACCL_OUT
  rawData[10] = SPI.transfer(0x00);
  rawData[11] = SPI.transfer(0x00);
  // ZACCL_OUT
  rawData[12] = SPI.transfer(0x00);
  rawData[13] = SPI.transfer(0x00);
  // XMAGN_OUT
  rawData[14] = SPI.transfer(0x00);
  rawData[15] = SPI.transfer(0x00);
  // YMAGN_OUT
  rawData[16] = SPI.transfer(0x00);
  rawData[17] = SPI.transfer(0x00);
  // ZMAGN_OUT
  rawData[18] = SPI.transfer(0x00);
  rawData[19] = SPI.transfer(0x00);
  // BARO_OUT
  rawData[20] = SPI.transfer(0x00);
  rawData[21] = SPI.transfer(0x00);
  // TEMP_OUT
  rawData[22] = SPI.transfer(0x00);
  rawData[23] = SPI.transfer(0x00);

  if (getType() == Adis16448Type::BMLZ) {
    // CRC-16
    rawData[24] =
        SPI.transfer(0x00); // Final transfer. Data after this invalid
    rawData[25] = SPI.transfer(0x00);
  }

  digitalWrite(CS_, HIGH); // Disable communication with device.

  // Concatenate two bytes into word
  sensorData[0] = (rawData[0] << 8) | (rawData[1] & 0xFF);    // DIAG_STAT
  sensorData[1] = (rawData[2] << 8) | (rawData[3] & 0xFF);    // XGYRO_OUT
  sensorData[2] = (rawData[4] << 8) | (rawData[5] & 0xFF);    // YGYRO_OUT
  sensorData[3] = (rawData[6] << 8) | (rawData[7] & 0xFF);    // ZGYRO_OUT
  sensorData[4] = (rawData[8] << 8) | (rawData[9] & 0xFF);    // XACCL_OUT
  sensorData[5] = (rawData[10] << 8) | (rawData[11] & 0xFF);  // YACCL_OUT
  sensorData[6] = (rawData[12] << 8) | (rawData[13] & 0xFF);  // ZACCL_OUT
  sensorData[7] = (rawData[14] << 8) | (rawData[15] & 0xFF);  // XMAGN_OUT
  sensorData[8] = (rawData[16] << 8) | (rawData[17] & 0xFF);  // YMAGN_OUT
  sensorData[9] = (rawData[18] << 8) | (rawData[19] & 0xFF);  // ZMAGN_OUT
  sensorData[10] = (rawData[20] << 8) | (rawData[21] & 0xFF); // BARO_OUT
  sensorData[11] = (rawData[22] << 8) | (rawData[23] & 0xFF); // TEMP_OUT
  if (getType() == Adis16448Type::BMLZ) {
    sensorData[12] = (rawData[24] << 8) | (rawData[25] & 0xFF); // CRC-16
  }
  return true; // success
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16448::regWrite(uint8_t regAddr, int16_t regData) {
  // Write register address and data
  uint16_t addr =
      (((regAddr & 0x7F) | 0x80)
       << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord =
      (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord =
      ((addr | 0x100) |
       ((regData >> 8) &
        0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  digitalWrite(CS_, LOW);         // Set CS low to enable device
  SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord);  // Write low byte from high word to SPI bus
  digitalWrite(CS_, HIGH);        // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (40us)

  // Write lowWord to SPI bus
  digitalWrite(CS_, LOW);        // Set CS low to enable device
  SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord);  // Write low byte from low word to SPI bus
  digitalWrite(CS_, HIGH);       // Set CS high to disable device

  return (1);
}

////////////////////////////////////////////////////////////////////////////
// Modify a the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// clearBits - bits to be cleared
// setBits - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16448::regMod(uint8_t regAddr, uint16_t clearBits, uint16_t setBits) {
  // Read current value of register
  uint16_t val = regRead(regAddr);

  // Clear specified bits
  val &= ~clearBits;
  // set specified bits
  val |= setBits;

  // write to register
  regWrite(regAddr, val);

  return (1);
}

////////////////////////////////////////////////////////////////////////////
// Calculates checksum based on burst data.
// Returns the calculated checksum.
////////////////////////////////////////////////////////////////////////////
// *burstArray - array of burst data
// return - (int16_t) signed calculated checksum
////////////////////////////////////////////////////////////////////////////
int16_t ADIS16448::checksum(int16_t *burstArray) {
  unsigned char i;        // Tracks each burstArray word
  unsigned int data;      // Holds the lower/Upper byte for CRC computation
  unsigned int crc;       // Holds the CRC value
  unsigned int lowerByte; // Lower Byte of burstArray word
  unsigned int upperByte; // Upper Byte of burstArray word
  unsigned int POLY;      // Divisor used during CRC computation
  POLY = 0x1021;          // Define divisor
  crc = 0xFFFF;           // Set CRC to ‐1 prior to beginning CRC computation
  // Compute CRC on burst data starting from XGYRO_OUT and ending with TEMP_OUT.
  // Start with the lower byte and then the upper byte of each word.
  // i.e. Compute XGYRO_OUT_LSB CRC first and then compute XGYRO_OUT_MSB CRC.
  for (i = 1; i < 12; i++) {
    upperByte = (burstArray[i] >> 8) & 0xFF;
    lowerByte = (burstArray[i] & 0xFF);
    updateCRC(&crc, &lowerByte, POLY);
    updateCRC(&crc, &upperByte, POLY);
  }
  crc = ~crc; // Compute complement of CRC
  data = crc;
  crc = (crc << 8) |
        (data >> 8 & 0xFF); // Perform byte swap prior to returning CRC
  return crc;
}

void ADIS16448::updateCRC(unsigned int *crc, unsigned int *data,
                              const unsigned int &POLY) {
  unsigned char ii; // Counter for each bit of the current burstArray word
  for (ii = 0; ii < 8; ii++, *data >>= 1) {
    if ((*crc & 0x0001) ^ (*data & 0x0001))
      *crc = (*crc >> 1) ^ POLY;
    else
      *crc >>= 1;
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the sensor data
// uses CRC if available (i.e. on ADIS16448BMLZ)
// no additional validity checks (may result in spikes).
///////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16448::updateData() {
  sensorReadAll(sensor_data_);
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Method to update the internally stored sensor data recusivelly by checking
// the validity.
///////////////////////////////////////////////////////////////////////////////////////////////
bool ADIS16448::updateDataIterative()
{
  uint64_t tic = micros();
  bool success = false;
  for (size_t depth = 0; depth < kMaxRecursiveUpdateDepth; ++depth) {
//    setTimestampNow();
    if (!sensorReadAll(sensor_data_) || sensor_data_[12] != checksum(sensor_data_)) {
      if (micros() - tic > kImuSyncTimeoutUs) {
        return false;
      }
      DEBUG_PRINTLN(
          topic_ +
          " (ADIS16448.cpp): Failed IMU update detected, trying again " +
          (String)(kMaxRecursiveUpdateDepth - depth) + " times.");
    }
    else{
      return true;
    }
  }
  return false;
}

ADIS16448::Adis16448Type ADIS16448::getType() {
  return type_;
}