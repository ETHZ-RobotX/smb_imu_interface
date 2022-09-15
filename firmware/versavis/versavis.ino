// Import all settings for the chosen sensor configuration.
#include "versavis_configuration.h"

#include <math.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

#include "Arduino.h"

#if defined(USE_ADIS16448AMLZ)
#include <ADIS16448.h>
#elif defined(USE_ADIS16448BMLZ)
#include <ADIS16448.h>
#endif
#include <helper.h>

static void resetCb(const std_msgs::Bool & /*msg*/) { NVIC_SystemReset(); }

/* ----- ROS ----- */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> reset_sub("/versavis/reset", &resetCb);

/* ----- IMU ----- */
#if defined(USE_ADIS16448AMLZ)
ADIS16448 imu(&nh, IMU_TOPIC, ADIS16448::Adis16448Type::AMLZ, 10, 2, 0);
#elif defined(USE_ADIS16448BMLZ)
ADIS16448 imu(&nh, IMU_TOPIC, ADIS16448::Adis16448Type::BMLZ, 10, 2, 0);
#endif
// static const Imu* cur_imu = &imu;

void setup() {
  DEBUG_INIT(115200);

  delay(1000);

/* ----- ROS ----- */
#ifndef DEBUG
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(reset_sub);
#else
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  DEBUG_PRINTLN(F("Main: Start setup."));

  imu.setup();

  /* -----  attach interrupt ----- */
  // use data ready signal
  pinMode(38, INPUT);
  attachInterrupt(38, readIMUdata, RISING);

  imu.begin();
}

void loop() {
  while(!imu.isNewMeasurementAvailable()) {}
  imu.publish();

#ifndef DEBUG
  nh.spinOnce();
#endif
}

void readIMUdata() { // Called by interrupt.
  imu.setTimestampNow();
  imu.newMeasurementIsAvailable();
  DEBUG_PRINTLN("imu interrupt");  
}
