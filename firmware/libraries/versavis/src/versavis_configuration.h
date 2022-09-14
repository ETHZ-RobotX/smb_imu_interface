#ifndef versavis_configuration_h
#define versavis_configuration_h

/* ----- General configuration -----*/
// Activate USB serial interface for ARM processor types.
#define USE_USBCON

// Specify the CPU frequency of the controller.
#define CPU_FREQ_HZ 48e6

// Specify the trigger pulse width;
#define TRIGGER_PULSE_US 10

/* ----- IMU -----*/
// Possible values: USE_ADIS16445, USE_ADIS16448AMLZ, USE_ADIS16448BMLZ,
// USE_ADIS16460 and USE_VN100
#define USE_ADIS16448AMLZ
#define IMU_TOPIC "/versavis/imu_micro"
#define IMU_RATE 200

/* ----- Additional triggers ----- */
// Define whether additional test outputs should be used.
// #define ADD_TRIGGERS
#ifdef ADD_TRIGGERS
#define ADDITIONAL_TEST_PIN 2
#endif


/* ----- Debug mode. ----- */
// Define whether debug mode should be used. This provides output on the
// standard console but invalidates ROS communication.
// #define DEBUG

#endif // versavis_configuration_h
