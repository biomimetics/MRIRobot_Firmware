#ifndef COMMON_H
#define COMMON_H

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
// Master switch for every PRINT_* flag below: 0 forces all of them off
// regardless of their individual values below, so performance testing (e.g.
// characterizing loop timing) doesn't require hunting down and disabling
// each flag separately. Set to 1 to fall back to each flag's own value.
#define GLOBAL_PRINT_GATE 1

#define PRINT_STATEMACHINE (GLOBAL_PRINT_GATE && 0)
#define PRINT_USM (GLOBAL_PRINT_GATE && 0)
#define PRINT_ENCODER (GLOBAL_PRINT_GATE && 0)
#define PRINT_UART (GLOBAL_PRINT_GATE && 0)

// Combined USM+SEA position/velocity readings for one QDEC sample. Sent as a
// single struct-typed port between Encoder.lf and State_Machine.lf (instead
// of 4 separate float[7] ports) so the four arrays can never be observed out
// of step with each other downstream, and so State_Machine only needs one
// reaction to consume all of them.
typedef struct {
  float motor_position[7];
  float motor_velocity[7];
  float sea_position[7];
  float sea_velocity[7];
} EncoderStateMessage;
// 0: fixed-length HAL_UART_Receive_DMA, now sized to COMMAND_PACKET_SIZE
// (see UART.lf) rather than UART_BUFFER_SIZE, so it completes on exactly one
// CommandMessage-sized packet instead of waiting for StateMessage-sized
// (158-byte) input that never arrives on this RX-only line. Switched away
// from the idle-line path (1) because at 921600 baud a large CommandMessage
// packet can get split across USB-serial bulk transfers, and idle-line
// detection would fire early on the inter-transfer gap, truncating the
// packet (see git history around 2026-07-07 for the "Got 73, expected 100"
// bug this caused). Revisit if CommandMessage packets become variable-length
// -- fixed-size reception assumes every packet on this line is the same size.
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

// USM constants for external encoder handling

#define INITIAL_CPR 1440.0 //5760.0 //1440.0
#define INITIAL_INTERP_FACTOR 4.0 // either 1, 2, or 4

#define REAL_CPR_BASE 10000.0
#define E2_INTERP_FACTOR 1.0
#define CPR_RATIO_BASE ((INITIAL_CPR * INITIAL_INTERP_FACTOR) / (REAL_CPR_BASE * E2_INTERP_FACTOR))

#define REAL_CPR 2000.0
#define L2_INTERP_FACTOR 1.0 // this encoder has no interpolation option, so this stays 1.0 -- it's a hardware fact, not a tuning knob
#define L2_CAL_FACTOR 1.0 // empirical fit (measured 1.3-1.4x) for an unexplained slowdown on the 2000 CPR encoders not captured by CPR_RATIO alone. Refine with a proper speed sweep.
#define CPR_RATIO ((INITIAL_CPR * INITIAL_INTERP_FACTOR) / (REAL_CPR * L2_INTERP_FACTOR * L2_CAL_FACTOR))


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

#endif // COMMON_H

