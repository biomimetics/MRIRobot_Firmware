#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

// for debugging
#define PRINT_STATEMACHINE 0
#define PRINT_USM 0
#define PRINT_ENCODER 0
#define PRINT_UART 0
#define USE_EX_DMA 0

// for communication
#define DMA_TX_BUFFER_SIZE 400 //256//(STM_BUFFER_SIZE) //200
#define DMA_RX_BUFFER_SIZE 400 //256//(STM_BUFFER_SIZE) //200

// constants
#define TWO_PI 6.28318
#define NSEC_TO_SEC 0.000000001
#define MSEC_TO_SEC 0.001

// conversion for if inputs are in rad_per_sec
#define RAD_PER_SEC_TO_RPM 9.549297
#define RAD_PER_SEC_TO_DEG_PER_SEC 57.2958 // we shouldn't use this
#define RPM_TO_RAD_PER_SEC (1.0 / RAD_PER_SEC_TO_RPM)
#define RPM_TO_DEG_PER_SEC 6.0

// USM constants
#define INITIAL_CPR 1440.0
#define REAL_CPR 2000.0
#define CPR_RATIO (INITIAL_CPR / REAL_CPR) // REAL_CPR is higher, so this should scale the max rpm down and also the encoder velocities down.

#define INITIAL_PWM_RPM_MAX 250.0
#define PWM_RPM_MAX (INITIAL_PWM_RPM_MAX * CPR_RATIO)
#define PWM_RAD_PER_SEC_MAX (PWM_RPM_MAX * RPM_TO_RAD_PER_SEC)
#define PWM_DEG_PER_SEC_MAX (PWM_RPM_MAX * RPM_TO_DEG_PER_SEC)

#define ARR_PERIOD 2000 //2000 would be great but still sawtooth-esque //65535 // 16-bit timer period for ARR (auto reload register)


// basic safety limits
#define USM_MAX_DUTY_CYCLE 0.70//0.30 //0.20 //0.30 //0.06 //0.5 // only higher than 6% for PWM pin testing //0.06 // 6% duty cycle
#define USM_MIN_DUTY_CYCLE 0.00 //0.005 // if below this amount, we don't expect the motors to be able to move smoothly on their own based on bench testing.

#define MOTOR_EXP_FILTER_ALPHA 0.8
#define MOTOR_VELOCITY_DEADBAND_LIMIT 0.00 //0.0 //0.1309 // rad/s or 7.5 deg/s
#define MOTOR_VELOCITY_MAX_CHANGE 0.5236 // rad/s or 30 deg/s

// observer safety limits (NOT YET IMPLIMENTED)
#define VEL_DIFFERENCE_THRESHOLD_RAD_PER_SEC 0.785 // 0.785 rad/s ~= 45 deg/s

