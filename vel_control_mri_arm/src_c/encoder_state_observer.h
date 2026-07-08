#ifndef ENCODER_STATE_OBSERVER_H
#define ENCODER_STATE_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>
#include "pulse_motor_model.h"

// Per-joint fixed-gain 3-state observer (position, velocity, input-bias
// "disturbance") fusing:
//
//   - FPGA QDEC position (the primary, and only unbiased, measurement --
//     corrected against with a Huber-style innovation gate, see
//     innovation_gate below, since the FPGA is suspected of overcounting
//     EMI noise as real counts),
//   - FPGA QDEC velocity (deweighted: it's differentiated from the SAME
//     counts as position, so it's not independent information, and its
//     windowed difference overshoots at starts/stops -- used only as a
//     small trim via Lvv below),
//   - the commanded velocity actually sent to the USM (optional -- see
//     use_commanded_velocity), pushed through an asymmetric delay line
//     that models the motor's dead time: starts respond after T_start
//     (~100-200 ms, per-motor, refined online by
//     PulseMotorModelEstimator/PulseMotorModelEstimatorMixture_Bank.lf),
//     stops settle after the much shorter T_stop. See
//     EncoderStateObserver_Update's expected-velocity block for the
//     two-tap min-magnitude rule that implements the asymmetry.
//
// Deliberately NO online adaptation loops (delay correlation search,
// confidence weighting, etc. -- see the rejected drafts in
// src_c/possible_additions/): the command delay is already estimated
// empirically per-pulse by PulseMotorModelEstimator.lf and arrives here
// via EncoderStateObserver_SetModel, and fixed gains keep the thing
// tunable and provably stable. Gains are steady-state-Kalman-shaped
// (position innovation corrects position, velocity, and disturbance at
// fixed rates), chosen offline.
//
// Used by EncoderStateEstimator.lf, one instance per joint. Also usable
// for the SEA encoders: with use_commanded_velocity == false the command
// path (delay line, expected velocity, rest gating) is inert and this
// degrades to a plain constant-velocity + disturbance smoother.

// Command-history ring depth, in samples. Must exceed the largest
// plausible T_start / dt: at the QDEC's 10 ms sample period, 64 samples =
// 640 ms of history, ~3x the ~200 ms T_start currently being observed on
// hardware -- headroom for the large pulse-to-pulse/motor-to-motor
// variance without being resizable at runtime. Fixed at compile time so
// the struct stays a flat, statically-sized blob (no heap on the F446).
#define ESO_CMD_HISTORY_SIZE 64

typedef struct {
  /*----------------------------------------------------------
      Estimated states
  ----------------------------------------------------------*/

  float position;    // rad
  float velocity;    // rad/s
  // rad/s^2 -- extended state: unmodeled input acceleration (load torque,
  // stiction release, model gain error). Learned slowly from position
  // innovations (Ld below), decayed toward zero (disturbance_decay) so a
  // transient can't be mistaken for a permanent bias, and clamped
  // (max_disturbance) so it can't wind up.
  float disturbance;

  /*----------------------------------------------------------
      Configuration (set once at Init)
  ----------------------------------------------------------*/

  float dt;                    // s, fixed update period (the QDEC sample period)
  // false = ignore the command path entirely (SEA use) -- prediction
  // becomes constant-velocity + disturbance, and rest gating (which needs
  // a trustworthy "nothing is commanded" signal) is disabled.
  bool use_commanded_velocity;

  /*----------------------------------------------------------
      Motor-model-derived terms (refreshed by SetModel)
  ----------------------------------------------------------*/

  float v_real_mean;           // rad/s, PulseMotorModel.V_real.mean
  float v_min_cmd;             // rad/s, PulseMotorModel.V_min_cmd
  int start_delay_samples;     // round(T_start / dt), clamped to the ring
  int stop_delay_samples;      // round(T_stop / dt), clamped to the ring

  /*----------------------------------------------------------
      Observer gains (fixed; defaults set in Init)
  ----------------------------------------------------------*/

  // s -- first-order lag pulling velocity toward the delayed expected
  // command. Deliberately loose (default 100 ms): the command term's job
  // is to bridge start/stop transients where the measured velocity is at
  // its worst, not to dominate the encoder.
  float tau;

  float Lp;                    // dimensionless, position innovation -> position
  float Lv;                    // 1/s^2, position innovation -> velocity (scaled by dt in Update, so tuning is roughly sample-rate independent)
  float Lvv;                   // dimensionless, measured-velocity trim -> velocity (small -- see the header comment on why measured velocity is deweighted)
  float Ld;                    // 1/s^3, position innovation -> disturbance
  float disturbance_decay;     // 1/s, exponential decay rate of disturbance
  float max_disturbance;       // rad/s^2, hard clamp on |disturbance| (anti-windup)

  /*----------------------------------------------------------
      Innovation gating (EMI / outlier defense)
  ----------------------------------------------------------*/

  // rad -- Huber-style saturation on the position innovation while
  // moving: an innovation is clamped to +-this before any gain sees it,
  // so a burst of spurious counts tugs the estimate instead of yanking
  // it. Default sized just above the largest physically possible
  // per-sample motion (max_speed 12.566 rad/s * 10 ms ~= 0.126 rad).
  float innovation_gate;

  // rad -- much tighter gate used while at_rest (see below): with nothing
  // commanded and the estimate at rest, real motion can't (shouldn't)
  // happen, so encoder counts beyond the noise floor are presumed EMI.
  // Default sized against PulseMotorModelEstimator.lf's
  // movement_detection_threshold (0.01 rad) -- TODO: characterize against
  // the actual idle-encoder noise floor, same TODO as over there.
  float rest_innovation_gate;

  // dimensionless 0..1 -- multiplier on Lp/Lv/Lvv/Ld while at_rest.
  // Small but nonzero: a REAL unexpected motion at rest (someone bumps
  // the arm) still gets tracked, just slowly (time constant
  // ~ dt / (Lp * rest_gain_scale) ~= 1 s at the defaults) -- EMI
  // rejection while parked in the scanner bore is the priority.
  float rest_gain_scale;

  float rest_cmd_threshold;    // rad/s, both current and delayed command must be under this to count as rest
  float rest_vel_threshold;    // rad/s, |velocity estimate| must be under this to count as rest
  float max_velocity_error;    // rad/s, clamp on the measured-velocity trim's innovation

  /*----------------------------------------------------------
      Command history
  ----------------------------------------------------------*/

  float cmd_history[ESO_CMD_HISTORY_SIZE];
  uint16_t cmd_index;          // next slot to write

  /*----------------------------------------------------------
      Bookkeeping / diagnostics
  ----------------------------------------------------------*/

  // First-sample latch: position can legitimately start far from zero
  // (position_offset restores pre-restart pose), so the first
  // measurement SEEDS the estimate instead of the observer slewing from
  // zero through the innovation gate one clamped step at a time.
  bool position_seeded;

  float last_innovation;       // rad, most recent position innovation, PRE-clamp -- log this to characterize the gates/noise floor
  bool last_innovation_clamped;// true if last_innovation exceeded the active gate
  bool at_rest;                // true if the rest gate/gain scaling was active on the last Update
} EncoderStateObserver;

// Zeros the states/history, installs the default gains documented above,
// and stores dt/use_commanded_velocity. Call SetModel afterwards (with
// INITIAL_MOTOR_MODEL if nothing better) to size the delay taps.
void EncoderStateObserver_Init(EncoderStateObserver *o, float dt, bool use_commanded_velocity);

// Refreshes the motor-model-derived terms (V_real.mean, V_min_cmd, and
// the T_start/T_stop delay taps) from an updated PulseMotorModel -- the
// hook for PulseMotorModelEstimatorMixture_Bank.lf's refined model to
// improve this observer's prediction online. Safe to call at any time:
// only prediction terms change, never the estimated states.
void EncoderStateObserver_SetModel(EncoderStateObserver *o, const PulseMotorModel *model);

// One full predict + correct step. Call exactly once per dt (the QDEC
// sample period) with that tick's values:
//   commanded_velocity -- rad/s, the command actually sent to the USM
//                         this tick (Small_DeltaP_Pulse_Controller's
//                         command_velocity_output -- correct in both
//                         pass-through and pulsing modes). Ignored (may
//                         be 0) when use_commanded_velocity is false.
//   motor_state        -- that joint's MotorState. Only used to (a) scale
//                         the expected pulse-height command by
//                         V_real.mean/V_min_cmd while a pulse is in
//                         flight and (b) veto rest gating mid-pulse.
//                         MOTOR_STOPPED is fine for SEA use.
//   measured_position  -- rad, QDEC position (post-FPGA, pre-offset).
//   measured_velocity  -- rad/s, QDEC velocity.
void EncoderStateObserver_Update(EncoderStateObserver *o,
                                 float commanded_velocity,
                                 MotorState motor_state,
                                 float measured_position,
                                 float measured_velocity);

#endif // ENCODER_STATE_OBSERVER_H
