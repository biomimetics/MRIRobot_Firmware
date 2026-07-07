#ifndef PULSE_MOTOR_MODEL_H
#define PULSE_MOTOR_MODEL_H

#include <stdbool.h>

// Physical/open-loop motor model, shared bang-bang FSM shape, pulse command
// type, and pulse-outcome prediction helper -- split out so any controller
// that bang-bangs the same physical motor (pulse_mpc.c's motion-debt design,
// dpos_pulse_mpc.c's delta-position design, and future alternative
// implementations meant to be compared against them) can drive it without
// duplicating this layer. Only each controller's own planning logic on top
// of this differs. See each controller's own header for its
// decision/action types built on top of these.

// Physical/open-loop motor model, all placeholders (see each controller
// reactor's `startup` reaction for current values). A future live parameter
// estimator has a single, self-contained struct to update without touching
// any controller's own tuning or FSM state.
typedef struct {
  float V_min;              // rad/s, magnitude of the fixed pulse command speed
  float gain;                // unitless 0..1, actual/commanded ratio
  float T_start;             // s, startup command delay
  float T_stop;              // s, hard-stop settle time
  float minimumPulseWidth;   // s, hysteresis guard before STOP eligible
  float V_variance;          // rad^2/s^2, variance of the actual speed around V_min * gain
} MotorModel;

// Single shared initial-guess MotorModel, all placeholders pending real
// bench characterization -- same values every reactor's own startup used to
// hard-code individually (see e.g. Small_DeltaP_Pulse_Controller.lf's old
// V_min_initial/gain_initial/etc. constructor parameters). Centralized here,
// same role as motor_config.h's per-joint Motor_Config array, so every
// downstream reactor (Small_DeltaP_Pulse_Controller.lf,
// PulseMotorModelEstimator.lf, and any future one) starts from exactly the
// same initial belief about the motor -- one value to update as
// characterization improves, rather than the same six numbers duplicated
// across each reactor's own constructor defaults. Not a compile-time
// constant expression once copied into a `static` array initializer
// elsewhere (C initializers must be constant expressions, and naming
// another object isn't one) -- copy it at runtime instead (e.g. in a
// `reaction(startup)`), the same way each existing reactor already does.
static const MotorModel INITIAL_MOTOR_MODEL = {
  0.35f,    // V_min             -- rad/s, ~20 deg/s -- TODO: characterize
  0.9f,     // gain              -- unitless 0..1 -- TODO: characterize
  0.005f,   // T_start           -- s (5 ms) -- TODO: characterize further
  0.001f,   // T_stop            -- s (1 ms), datasheet value, likely optimistic
  0.04f,    // minimumPulseWidth -- s (40 ms) -- TODO: characterize
  0.0025f,  // V_variance        -- rad^2/s^2 -- TODO: characterize
};

// Shared FSM states for any controller that bang-bangs this motor between a
// fixed pulse speed and stopped, regardless of how it decides when to
// transition.
typedef enum {
  MOTOR_STOPPED,
  MOTOR_STARTING,
  MOTOR_RUNNING,
  MOTOR_STOPPING
} MotorState;

// A single planned pulse: direction and how long to spend in MOTOR_RUNNING
// (excluding T_start/T_stop dead time, which contribute zero motion under
// the current model). Shared by any controller that plans discrete pulses
// directly (currently dpos_pulse_mpc.c) -- see
// dpos_pulse_mpc_planning.md §2.
typedef struct {
  float dir;          // +1.0f or -1.0f
  float run_duration; // s
} PulseCommand;

// +1.0f if x >= 0.0f, else -1.0f. Shared tiny helper -- used anywhere a
// commanded/measured velocity's direction needs latching.
float sign_f(float x);

// Point estimate of a pulse's effect on the motor's change of position without uncertainty:
// dir * gain * V_min * run_duration.
float PulseMotorModel_PredictDelta(const MotorModel *model, PulseCommand pulse);

// Generic pulse execution state: which FSM state we're in, the
// currently-latched direction, elapsed-time timers, the pulse currently
// committed to, and the resulting command velocity. Deliberately decoupled
// from any specific decision logic (MPC cost comparisons, planning
// searches, etc.) -- a controller decides WHEN to start a pulse (and what
// PulseCommand to run) and WHETHER a running pulse should be cut short;
// PulseMotorModel_StartPulse/PulseMotorModel_Advance below execute the FSM
// given those decisions, so any controller (DposPulseMPC, PulseMPC, or a
// future simpler alternative) can drive the same execution logic without
// reimplementing it.
typedef struct {
  MotorState state;
  float dir;                 // +1.0/-1.0, latched at STOPPED->START, held until back to STOPPED
  float stateTimer;          // s, elapsed since entering STARTING/STOPPING
  float pulseTimer;          // s, elapsed since entering RUNNING
  PulseCommand plannedPulse; // the pulse currently committed to (valid from STARTING through STOPPING)
  float commandVelocity;     // rad/s, signed, last commanded output
} PulseMotorState;

// Commits to running `pulse`: latches dir/plannedPulse, transitions
// MOTOR_STOPPED -> MOTOR_STARTING, and sets the initial commandVelocity
// (dir * model->V_min). Caller is responsible for only calling this while
// state == MOTOR_STOPPED (i.e. after its own decision logic, run at
// MOTOR_STOPPED, chooses to start a pulse).
void PulseMotorModel_StartPulse(PulseMotorState *s, const MotorModel *model, PulseCommand pulse);

// Advances the FSM by dt seconds: MOTOR_STARTING -> MOTOR_RUNNING once
// model->T_start elapses, MOTOR_STOPPING -> MOTOR_STOPPED once
// model->T_stop elapses, and (while MOTOR_RUNNING) -> MOTOR_STOPPING once
// either plannedPulse.run_duration elapses OR the caller passes
// stop_early = true -- the hook for a controller's own early-exit
// conditions (e.g. demand has moved back to normal speed, the target has
// reversed) that should cut the pulse short regardless of its planned
// duration. MOTOR_STOPPED is left untouched here -- deciding whether/how to
// leave it is the caller's job (see PulseMotorModel_StartPulse).
void PulseMotorModel_Advance(PulseMotorState *s, const MotorModel *model, float dt, bool stop_early);

#endif // PULSE_MOTOR_MODEL_H
