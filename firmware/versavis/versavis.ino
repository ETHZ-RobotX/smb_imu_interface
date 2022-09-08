// Import all settings for the chosen sensor configuration.
#include "versavis_configuration.h"

#include <math.h>
#include <ros.h>
#include <std_msgs/UInt8.h>

#include "Arduino.h"

#ifdef USE_ADIS16445
#include <ADIS16445.h>
#elif defined(USE_ADIS16448AMLZ)
#include <ADIS16448AMLZ.h>
#elif defined(USE_ADIS16448BMLZ)
#include <ADIS16448BMLZ.h>
#elif defined(USE_ADIS16460)
#include <ADIS16460.h>
#elif defined(USE_VN100)
#include <VN100.h>
#endif
#include <Camera.h>
#include <Timer.h>
#include <helper.h>

static void resetCb(const std_msgs::Bool & /*msg*/) { NVIC_SystemReset(); }

/* ----- ROS ----- */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Bool> reset_sub("/versavis/reset", &resetCb);

/* ----- Timers ----- */
// In the current setup: TC5 -> IMU
// Be careful, there is NO bookkeeping whether the timer is already used or
// not. Only use a timer once, otherwise there will be unexpected behavior.
Timer timer_imu = Timer((TcCount16 *)TC5);

/* ----- IMU ----- */
#ifdef USE_ADIS16445
ADIS16445 imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_ADIS16448AMLZ)
ADIS16448AMLZ imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_ADIS16448BMLZ)
ADIS16448BMLZ imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_ADIS16460)
ADIS16460 imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu, 10, 2, 9);
#elif defined(USE_VN100)
VN100 imu(&nh, IMU_TOPIC, IMU_RATE, timer_imu);
#endif

void setup() {
  DEBUG_INIT(115200);

  delay(1000);

/* ----- ROS ----- */
#ifndef DEBUG
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(reset_sub);
#else
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  DEBUG_PRINTLN(F("Main: Start setup."));

  imu.setup();

  /* -----  Declare timers ----- */
  // Enable TC4 (not used) and TC5 timers.
  REG_GCLK_CLKCTRL = static_cast<uint16_t>(
      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5);
  while (GCLK->STATUS.bit.SYNCBUSY == 1) {
    ; // wait for sync
  }

  // enable InterruptVector.
  NVIC_EnableIRQ(TC5_IRQn);

  imu.begin();
}

void loop() {
  imu.publish();

#ifndef DEBUG
  nh.spinOnce();
#endif
}

void TC5_Handler() { // Called by imu_timer for imu trigger.
  imu.triggerMeasurement();
}
