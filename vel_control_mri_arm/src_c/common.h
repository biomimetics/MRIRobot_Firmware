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
// Must be 1: CommandMessage and StateMessage are no longer the same size
// (see stm_comms.h), so UART4's fixed-length HAL_UART_Receive_DMA would wait
// for UART_BUFFER_SIZE bytes (sized for the larger StateMessage) even though
// only CommandMessage-sized packets ever arrive on this line, corrupting
// framing by concatenating multiple packets per DMA completion. The idle-line
// path uses the actual received byte count instead.
#define USE_EX_DMA 1

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

// USM constants for external encoder handling

#define INITIAL_CPR 1440.0 //5760.0 //1440.0
#define INITIAL_INTERP_FACTOR 4.0 // either 1, 2, or 4

#define REAL_CPR 2000.0
#define L2_INTERP_FACTOR 1.0
#define CPR_RATIO ((INITIAL_CPR * INITIAL_INTERP_FACTOR) / (REAL_CPR * L2_INTERP_FACTOR))

#define REAL_CPR_BASE 10000.0
#define E2_INTERP_FACTOR 1.0
#define CPR_RATIO_BASE ((INITIAL_CPR * INITIAL_INTERP_FACTOR) / (REAL_CPR_BASE * E2_INTERP_FACTOR))

#define INITIAL_PWM_RPM_MAX 250.0
#define PWM_RPM_MAX (INITIAL_PWM_RPM_MAX * CPR_RATIO)
#define PWM_RAD_PER_SEC_MAX (PWM_RPM_MAX * RPM_TO_RAD_PER_SEC)
#define PWM_DEG_PER_SEC_MAX (PWM_RPM_MAX * RPM_TO_DEG_PER_SEC)

#define PWM_RPM_MAX_BASE (INITIAL_PWM_RPM_MAX * CPR_RATIO_BASE)
#define PWM_RAD_PER_SEC_MAX_BASE (PWM_RPM_MAX_BASE * RPM_TO_RAD_PER_SEC)
#define PWM_DEG_PER_SEC_MAX_BASE (PWM_RPM_MAX_BASE * RPM_TO_DEG_PER_SEC) // this should all get moved somewhere else I think. motor_config.h?


#define ARR_PERIOD 2000 //2000 would be great but still sawtooth-esque //65535 // 16-bit timer period for ARR (auto reload register)


// basic safety limits
#define USM_MAX_DUTY_CYCLE 0.70//0.30 //0.20 //0.30 //0.06 //0.5 // only higher than 6% for PWM pin testing //0.06 // 6% duty cycle
#define USM_MIN_DUTY_CYCLE 0.00 //0.005 // if below this amount, we don't expect the motors to be able to move smoothly on their own based on bench testing.

#define MOTOR_EXP_FILTER_ALPHA 0.8
#define MOTOR_VELOCITY_DEADBAND_LIMIT 0.00 //0.0 //0.1309 // rad/s or 7.5 deg/s ## DEPRECATED
#define MOTOR_VELOCITY_MAX_CHANGE 0.5236 // rad/s or 30 deg/s

// observer safety limits (NOT YET IMPLIMENTED - MOVED TO ROS-SIDE)
#define VEL_DIFFERENCE_THRESHOLD_RAD_PER_SEC 0.785 // 0.785 rad/s ~= 45 deg/s

