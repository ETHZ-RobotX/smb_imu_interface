////////////////////////////////////////////////////////////////////////////////
//  April 2019
//  Author: Florian Tschopp <ftschopp@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  Imu.h
////////////////////////////////////////////////////////////////////////////////
//
//  Basic implementation for IMUs in the versavis framework. Refer to
//  the parent package versavis for license information.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef Imu_h
#define Imu_h

#include "Arduino.h"
#include <ros.h>
#include <versavis_adis16448_receiver/ImuMicro.h>

enum ImuReading {
  STAT = 0,
  GX = 1,
  GY = 2,
  GZ = 3,
  AX = 4,
  AY = 5,
  AZ = 6,
  // The reminder of the entries is not identical for the different ADIS IMUs.
  // Check burst read function of the specific IMU.
};

class Imu {
public:
  enum ImuType
  {
    ADIS16448AMLZ,
    ADIS16448BMLZ,
    undefined = 99
  };

  Imu(ros::NodeHandle *nh, const String &topic, ImuType imu_type);
  virtual void setup() = 0;
  void begin();
  void publish();
  void setupPublisher();

  // Update data internally with recursion.
  virtual bool updateDataIterative() = 0;

  // Update data internally without recursion.
  virtual bool updateData() = 0;

  // 
  bool isNewMeasurementAvailable() const;

  // set new_measurement_available true
  void newMeasurementIsAvailable();

  // set new_measurement_available false
  void newMeasurementIsNotAvailable();

  // 
  void setTimestampNow();

  //
  ros::Time getTimestamp() const;

  ImuType getType();

protected:
  ros::NodeHandle *nh_;
  String topic_;
  ros::Publisher publisher_;
  int16_t sensor_data_[16];
  const unsigned int kMaxRecursiveUpdateDepth;
  const uint64_t kImuSyncTimeoutUs;

private:
  versavis_adis16448_receiver::ImuMicro imu_msg_;
  volatile bool new_measurement_available_;
  ros::Time timestamp_;
  ImuType imu_type_;
};

#endif
