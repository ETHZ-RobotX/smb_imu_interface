#include "Arduino.h"
#include "Imu.h"
#include "helper.h"
#include "versavis_configuration.h"

Imu::Imu(ros::NodeHandle *nh, const String &topic, ImuType imu_type)
    : nh_(nh), topic_(topic), new_measurement_available_(false),
      imu_type_(imu_type), publisher_(topic_.c_str(), &imu_msg_),
      kMaxRecursiveUpdateDepth(5u), kImuSyncTimeoutUs(4000) {}

void Imu::publish()
{
  if (isNewMeasurementAvailable())
  {
    DEBUG_PRINTLN((topic_ + " (Imu.cpp): Publish."));
    bool success;
    success = updateDataIterative();
    if (!success)
    {
      newMeasurementIsNotAvailable();
      DEBUG_PRINTLN((topic_ + " (Imu.cpp): IMU update failed.").c_str());
    }
    else
    {
      imu_msg_.time.data = getTimestamp();
      if (sensor_data_ == nullptr)
      {
        error((topic_ + " (Imu.cpp): sensor_data == nullptr").c_str(), 10);
      }
      imu_msg_.gx = sensor_data_[ImuReading::GX];
      imu_msg_.gy = sensor_data_[ImuReading::GY];
      imu_msg_.gz = sensor_data_[ImuReading::GZ];
      imu_msg_.ax = sensor_data_[ImuReading::AX];
      imu_msg_.ay = sensor_data_[ImuReading::AY];
      imu_msg_.az = sensor_data_[ImuReading::AZ];
#ifndef DEBUG
      publisher_.publish(&imu_msg_);
#endif
      newMeasurementIsNotAvailable();
    }
  }
}

void Imu::setupPublisher()
{
  DEBUG_PRINT((topic_ + " (Imu.cpp): Setup publisher with topic ").c_str());
  DEBUG_PRINTLN(publisher_.topic_);
#ifndef DEBUG
  nh_->advertise(publisher_);
#endif
}

void Imu::begin()
{
  DEBUG_PRINTLN((topic_ + " (Imu.cpp): Begin.").c_str());
}

void Imu::setTimestampNow()
{
  timestamp_ = nh_->now();
}

ros::Time Imu::getTimestamp() const {
  return timestamp_;
}

bool Imu::isNewMeasurementAvailable() const
{
  return new_measurement_available_;
}

void Imu::newMeasurementIsAvailable()
{
  new_measurement_available_ = true;
}

void Imu::newMeasurementIsNotAvailable()
{
  new_measurement_available_ = false;
}