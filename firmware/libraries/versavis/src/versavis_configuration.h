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
// Possible values: ADIS16448AMLZ, ADIS16448BMLZ
#define USE_ADIS16448BMLZ
#define IMU_TOPIC "/versavis/imu_micro"

// If defined, LED on adaptor board will turn be turned on
// on succesful configuration using the ADIS GPIO 
#define ASL_ADIS_ADAPTOR_BOARD

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
