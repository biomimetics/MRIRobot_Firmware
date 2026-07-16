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

// For EncoderDiagnostics, embedded in EncoderStateMessage below so the FPGA's
// per-channel health rides the same port as the counts it describes and there
// is exactly ONE definition of that layout between Encoder.lf and the host.
// One-directional: stm_comms.h does not include this file, so the host's copy
// of it stays standalone.
#include "stm_comms.h"

// for debugging
// Master switch for every PRINT_* flag below: 0 forces all of them off
// regardless of their individual values below, so performance testing (e.g.
// characterizing loop timing) doesn't require hunting down and disabling
// each flag separately. Set to 1 to fall back to each flag's own value.
#define GLOBAL_PRINT_GATE 1

// Encoder.lf: 0 = fire the FPGA burst and block on HAL_UART_Receive every
// trigger cycle (current/default behavior). 1 = fire and arm a DMA receive
// instead, processing the frame in reaction(encoder_rx_action) once the ISR
// schedules it, so the trigger reaction never blocks. Both code paths stay
// in the source; flip this back to 0 and rebuild to revert if on-hardware
// testing goes badly -- no other changes needed.
#define ENCODER_DMA_RX_ENABLE 1

#define PRINT_STATEMACHINE (GLOBAL_PRINT_GATE && 0)
#define PRINT_USM (GLOBAL_PRINT_GATE && 0)
#define PRINT_ENCODER (GLOBAL_PRINT_GATE && 0)
#define PRINT_UART (GLOBAL_PRINT_GATE && 0)

// Combined USM+SEA position/velocity readings for one QDEC sample. Sent as a
// single struct-typed port between Encoder.lf and State_Machine.lf (instead
// of 4 separate float[7] ports) so the four arrays can never be observed out
// of step with each other downstream, and so State_Machine only needs one
// reaction to consume all of them.
//
// UNITS: raw REAL-encoder counts, straight off the FPGA with no conversion
// applied (see counts_domain_io_plan.md). "Real encoder" means the physical
// encoder on the joint -- motor_configs[i]->qdec_cpr counts/rev for USM,
// sea_cpr counts/inch for SEA -- NOT the 5760 counts/rev the Tekceleo drive
// hardcodes for its own duty-cycle math (see USM_ASSUMED_COUNTS_PER_REV
// below). These two count spaces are different; the host owns the mapping
// between them.
//
// Positions are int32_t because counts are genuinely integral. Velocities
// stay float because the FPGA's native estimate is fractional (Q20.11
// counts/sec, see Encoder.lf's ENCODER_VELOCITY_FRAC_BITS) -- truncating it
// to int would discard resolution for no gain, it is 4 bytes either way.
typedef struct {
  int32_t motor_position[7];  // counts (real encoder)
  float motor_velocity[7];    // counts/sec (real encoder)
  int32_t sea_position[7];    // counts (real encoder)
  float sea_velocity[7];      // counts/sec (real encoder)
  // Per-channel FPGA health for the readings above, forwarded verbatim to the
  // host in StateMessage. Same bundling rationale as the arrays: diagnostics
  // that describe a sample must travel with that sample, not on a side channel
  // where they could be observed a cycle out of step with it.
  EncoderDiagnostics diagnostics;
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

// =====================
// USM command conversion (counts/sec -> duty cycle)
// =====================
// The Tekceleo drive has ONE hardcoded, undocumented assumption about the
// encoder attached to it: 1440 CPR with 4x onboard interpolation, i.e. 5760
// counts/rev. Its published spec -- 100% duty cycle = 250 RPM -- is stated in
// terms of THAT assumed encoder, not whichever encoder is physically on the
// joint. So:
//
//     250 RPM / 60          = 4.1667 rev/s
//     4.1667 * 5760         = 24000 counts/sec at 100% duty
//
// Commands arrive from the host already in this "manufacturer count space"
// (CommandMessage.velocity_counts_per_sec), so USM.lf's conversion is a single
// divide by PWM_COUNTS_PER_SEC_MAX with NO dependence on qdec_cpr. That is the
// whole point: the old rad/s path multiplied in CPR_RATIO (below) to reconcile
// the real encoder against the drive's assumption, and that reconciliation --
// including L2_CAL_FACTOR's unexplained empirical fudge -- is the suspect
// behavior this change exists to rule out. See counts_domain_io_plan.md.
//
// All 7 motors share these constants precisely because the manufacturer count
// space does not vary with the real encoder.
#define USM_ASSUMED_CPR 1440.0
#define USM_ASSUMED_INTERP_FACTOR 4.0
#define USM_ASSUMED_COUNTS_PER_REV (USM_ASSUMED_CPR * USM_ASSUMED_INTERP_FACTOR) // 5760

#define PWM_COUNTS_PER_SEC_MAX ((INITIAL_PWM_RPM_MAX / 60.0) * USM_ASSUMED_COUNTS_PER_REV) // 24000

// Exact restatements of the old rad/s limits in the manufacturer count space,
// so safety behavior is unchanged by the unit switch:
//   12.566 rad/s (2 rev/s)  -> 2 * 5760      = 11520 counts/sec
//   0.5236 rad/s (30 deg/s) -> (1/12) * 5760 = 480 counts/sec
#define MOTOR_MAX_SPEED_COUNTS_PER_SEC 11520
#define MOTOR_VELOCITY_MAX_CHANGE_COUNTS_PER_SEC 480

// =====================
// LEGACY: rad/s -> duty cycle conversion chain
// =====================
// Superseded by PWM_COUNTS_PER_SEC_MAX above and no longer on the live command
// path. Retained only for USM_DAC.lf (imported by no Main) and the now-unused
// Motor_Config::pwm_rad_per_sec_max / max_speed fields. This is the machinery
// the counts/sec change is meant to bypass -- do not reintroduce it into
// USM.lf's command path without a good reason.

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
// LEGACY rad/s value -- State_Machine.lf's clamp now uses
// MOTOR_VELOCITY_MAX_CHANGE_COUNTS_PER_SEC (480, the same limit expressed in
// manufacturer counts/sec) since the command path is counts-domain now.
#define MOTOR_VELOCITY_MAX_CHANGE 0.5236 // rad/s or 30 deg/s

// observer safety limits (NOT YET IMPLIMENTED - MOVED TO ROS-SIDE)
#define VEL_DIFFERENCE_THRESHOLD_RAD_PER_SEC 0.785 // 0.785 rad/s ~= 45 deg/s

#endif // COMMON_H

