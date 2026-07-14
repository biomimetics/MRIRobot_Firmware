#ifndef ENCODER_STATE_OBSERVER_H
#define ENCODER_STATE_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

// Per-joint fixed-gain 3-state observer (position, velocity, input-bias
// "disturbance") fusing:
//
//   - FPGA QDEC position (the primary, and only unbiased, measurement --
//     corrected against with a Huber-style innovation gate, see
//     innovation_gate below, since the FPGA is suspected of overcounting
//     EMI noise as real counts),
//   - FPGA QDEC velocity (deweighted: it's differentiated from the SAME
//     counts as position, so it's not independent information -- used
//     only as a small trim via Lvv below).
//
// No command/motor-model input: commanded-velocity prediction and pulse
// timing now live in the higher-level (ROS2) code, not the firmware --
// see the removed PulseMotorModel-based delay line this replaced. This
// degrades to a plain constant-velocity + disturbance smoother: predict
// forward on the last velocity estimate, then correct against the fresh
// position/velocity measurement each tick.
//
// Deliberately NO online adaptation loops (delay correlation search,
// confidence weighting, etc. -- see the rejected drafts in
// src_c/possible_additions/): fixed gains keep the thing tunable and
// provably stable. Gains are steady-state-Kalman-shaped (position
// innovation corrects position, velocity, and disturbance at fixed
// rates), chosen offline.
//
// Used by EncoderStateEstimator.lf, one instance per joint (both USM and
// SEA encoders).

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

  /*----------------------------------------------------------
      Observer gains (fixed; defaults set in Init)
  ----------------------------------------------------------*/

  float Lp;                    // dimensionless, position innovation -> position
  float Lv;                    // 1/s^2, position innovation -> velocity (scaled by dt in Update, so tuning is roughly sample-rate independent)
  float Lvv;                   // dimensionless, measured-velocity trim -> velocity (small -- see the header comment on why measured velocity is deweighted)
  float Ld;                    // 1/s^3, position innovation -> disturbance
  float disturbance_decay;     // 1/s, exponential decay rate of disturbance
  float max_disturbance;       // rad/s^2, hard clamp on |disturbance| (anti-windup)

  /*----------------------------------------------------------
      Innovation gating (EMI / outlier defense)
  ----------------------------------------------------------*/

  // rad -- Huber-style saturation on the position innovation: an
  // innovation is clamped to +-this before any gain sees it, so a burst
  // of spurious counts tugs the estimate instead of yanking it. Default
  // sized just above the largest physically possible per-sample motion
  // (max_speed 12.566 rad/s * 10 ms ~= 0.126 rad).
  float innovation_gate;

  float max_velocity_error;    // rad/s, clamp on the measured-velocity trim's innovation

  /*----------------------------------------------------------
      Bookkeeping / diagnostics
  ----------------------------------------------------------*/

  // First-sample latch: position can legitimately start far from zero
  // (position_offset restores pre-restart pose), so the first
  // measurement SEEDS the estimate instead of the observer slewing from
  // zero through the innovation gate one clamped step at a time.
  bool position_seeded;

  float last_innovation;       // rad, most recent position innovation, PRE-clamp -- log this to characterize the gate/noise floor
  bool last_innovation_clamped;// true if last_innovation exceeded the active gate
} EncoderStateObserver;

// Zeros the states, installs the default gains documented above, and
// stores dt.
void EncoderStateObserver_Init(EncoderStateObserver *o, float dt);

// One full predict + correct step. Call exactly once per dt (the QDEC
// sample period) with that tick's values:
//   measured_position  -- rad, QDEC position (post-FPGA, pre-offset).
//   measured_velocity  -- rad/s, QDEC velocity.
void EncoderStateObserver_Update(EncoderStateObserver *o,
                                 float measured_position,
                                 float measured_velocity);

#endif // ENCODER_STATE_OBSERVER_H
