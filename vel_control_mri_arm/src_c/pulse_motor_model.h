#ifndef PULSE_MOTOR_MODEL_H
#define PULSE_MOTOR_MODEL_H

#include <stdbool.h>
#include "gaussian_motion.h"

// s -- ceiling on PulseMotorModel_PlanPulseWithOvershootBound's run_duration,
// guarding against bad edge cases (e.g. GaussianMotion_ArrivalTimeInvCdf
// returning +INFINITY when V_real's mean isn't far enough from zero
// relative to its variance at the requested overshoot probability -- see
// that function's header comment). Fixed rather than a per-instance
// parameter -- this is a safety backstop, not a tuning knob.
#define MAX_PULSE_DURATION_S 0.5f

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
  float V_min_cmd;           // rad/s, magnitude of the fixed pulse command speed
  GaussianRV V_real;         // rad/s, actual expected pulse velocity given V_min_cmd --
                              // mean/variance combined (was separate V_min_real/V_variance
                              // floats) since every use of one already used the other
  float T_start;             // s, startup command delay
  float T_stop;              // s, hard-stop settle time
  float minimumPulseWidth;   // s, minimum length of a pulse to see a response from the motor controller.
} PulseMotorModel;

// Single shared initial-guess PulseMotorModel, all placeholders pending real
// bench characterization
static const PulseMotorModel INITIAL_MOTOR_MODEL = {
  0.40f,          // V_min_cmd -- rad/s, was ~20 deg/s -- TODO: characterize // 0.872665f
  {0.30f, 0.00001f}, // V_real -- {mean rad/s, variance rad^2/s^2} -- TODO: characterize
  0.100f,         // T_start           -- s (50 ms) -- TODO: characterize further
  0.100f,         // T_stop            -- s (50 ms), datasheet value, likely optimistic
  0.001f,         // minimumPulseWidth -- s (1 ms) -- TODO: characterize
};

// Shared FSM states for any controller that bang-bangs this motor between a
// fixed pulse speed and stopped, regardless of how it decides when to
// transition.
typedef enum {
  MOTOR_STOPPED, // pulse command should be inactive (i.e. zero) and motor should not be moving
  MOTOR_STARTING, // pulse command should be active but we shouldn't expect to see motor movement yet
  MOTOR_RUNNING, // pulse command should be active and we should see (noisey) motor movement
  MOTOR_STOPPING // pulse command should be inactive (i.e. zero) but we should still see motor movement
} MotorState;

// Human-readable name for s, for debug printfs (e.g.
// PulseMotorModelEstimator.lf's PRINT_PME_DEBUG tracing) -- lives here
// (compiled exactly once, in pulse_motor_model.c) rather than in a .lf
// preamble, since preamble code can get transcluded into the generated
// build more than once and duplicate-define it -- same reasoning as
// SmallDeltaPPulse_PrintDebugInfo living in small_deltap_pulse_controller.c.
const char *MotorState_Name(MotorState s);

// A single planned pulse: direction and how long to spend in MOTOR_RUNNING
// (excluding T_start/T_stop dead time, which contribute zero motion under
// the current model). Shared by any controller that plans discrete pulses
// directly (currently dpos_pulse_mpc.c) -- see
// dpos_pulse_mpc_planning.md §2.
typedef struct {
  float dir;          // +1.0f or -1.0f
  float run_duration; // s
  float pulse_height; // rad/s
} PulseCommand;

// +1.0f if x >= 0.0f, else -1.0f. Shared tiny helper -- used anywhere a
// commanded/measured velocity's direction needs latching.
float sign_f(float x);

// Point estimate of a pulse's effect on the motor's change of position without uncertainty:
// dir * V_real.mean * run_duration.
float PulseMotorModel_PredictDelta(const PulseMotorModel *model, PulseCommand pulse);

// Estimate of a pulse's effect on the motor's change of position, treating the delta position as a GaussianRV:
GaussianRV PulseMotorModel_PredictDeltaDistribution(const PulseMotorModel *model, PulseCommand pulse);

GaussianRV PulseMotorModel_PredictVelocityDistribution(GaussianRV delta_dist, PulseCommand pulse);

// Assembles a point-estimate PulseMotorModel out of a single pulse's worth
// of observations: T_start/T_stop directly (seconds, as timed against real
// movement -- see PulseMotorModelEstimator.lf's t_start_observation_s_/
// t_stop_observation_s_), and v_real_estimate for V_real (typically
// SampleStats_ToGaussian of that pulse's measured-velocity samples --
// stats.h's generic stats-to-distribution step, not folded in here since
// it's not motor-model-specific). V_min_cmd/minimumPulseWidth are copied
// from *current* unchanged -- neither is something this estimator observes
// (V_min_cmd is a commanded, not measured, quantity; minimumPulseWidth
// needs bench characterization this reactor doesn't do) -- so the
// corresponding fields of PulseMotorModel_Subtract's residual against
// *current* come out exactly zero for those two.
PulseMotorModel PulseMotorModel_PointEstimateFromObservations(const PulseMotorModel *current, float t_start_s, float t_stop_s, GaussianRV v_real_estimate);

// Element-wise a - b across every PulseMotorModel field, including
// GaussianRV's mean/variance as plain arithmetic differences -- NOT
// GaussianRV_Subtract's uncertainty-propagation formula (which adds
// variances, assuming a and b are independent random variables). a and b
// here are two point *beliefs* about the same model (e.g. a fresh
// per-pulse estimate vs. the running model), so the residual this produces
// is a diagnostic difference between them, not a distribution.
PulseMotorModel PulseMotorModel_Subtract(PulseMotorModel a, PulseMotorModel b);

// Running sample statistics over a stream of PulseMotorModel-shaped values
// (e.g. per-pulse residuals -- see PulseMotorModel_Subtract above -- pooled
// across many pulses/joints, per the "all 7 joints are the same motor"
// assumption behind PulseMotorModelEstimatorMixture_Bank.lf) -- one
// SampleStats (stats.h) per scalar field, V_real's GaussianRV split into
// its mean and variance since SampleStats itself only tracks a single
// running scalar.
typedef struct {
  SampleStats V_min_cmd;
  SampleStats V_real_mean;
  // Currently always fed 0 (PulseMotorModelEstimator.lf's per-pulse
  // v_real_estimate.variance is left at 0 -- a single pulse's position-
  // delta/duration is one point measurement, not a distribution), so this
  // field just accumulates a running stat over a stream of zeros for now.
  // The actual V_real variance worth having is the pulse-to-pulse SPREAD of
  // V_real_mean above, not this field -- kept here rather than removed in
  // case a future per-pulse variance estimate (e.g. from multiple
  // running_velocity_stats_ samples) replaces the always-0 placeholder.
  SampleStats V_real_variance;
  SampleStats T_start;
  SampleStats T_stop;
  SampleStats minimumPulseWidth;
} PulseMotorModelSampleStats;

// Zeros every field's SampleStats -- same role as SampleStats_Init, just
// across all of PulseMotorModelSampleStats's fields at once.
void PulseMotorModelSampleStats_Init(PulseMotorModelSampleStats *s);

// Folds one PulseMotorModel-shaped sample into s, field by field, via
// SampleStats_Add.
void PulseMotorModelSampleStats_Add(PulseMotorModelSampleStats *s, PulseMotorModel sample);

// Plans a pulse to close remainingPositionDelta: direction from its sign,
// pulse_height fixed at model->V_min_cmd, and a duration bounded so that
// P(the motor has already covered remainingPositionDelta before the pulse
// ends) stays at or below max_overshoot_probability, per
// GaussianMotion_ArrivalTimeInvCdf (gaussian_motion.h) against model->V_real
// -- clamped up to model->minimumPulseWidth (same hysteresis floor every
// other controller here uses) and down to MAX_PULSE_DURATION_S (safety
// backstop against e.g. an unbounded result -- see that constant's
// comment).
PulseCommand PulseMotorModel_PlanPulseWithOvershootBound(const PulseMotorModel *model, float remainingPositionDelta, float max_overshoot_probability);

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
  float pulseTimer;          // s, elapsed since start of pulse, current started at START
  PulseCommand plannedPulse; // the pulse currently committed to (valid from STARTING through STOPPING)
  float commandVelocity;     // rad/s, signed, last commanded output
} PulseMotorState;

// Commits to running `pulse`: latches dir/plannedPulse, transitions
// MOTOR_STOPPED -> MOTOR_STARTING, and sets the initial commandVelocity
// (dir * model->V_min_cmd). Caller is responsible for only calling this while
// state == MOTOR_STOPPED (i.e. after its own decision logic, run at
// MOTOR_STOPPED, chooses to start a pulse).
void PulseMotorModel_StartPulse(PulseMotorState *s, const PulseMotorModel *model, PulseCommand pulse);

// Advances the FSM by dt seconds: MOTOR_STARTING -> MOTOR_RUNNING once
// model->T_start elapses, MOTOR_STOPPING -> MOTOR_STOPPED once
// model->T_stop elapses, and (while MOTOR_RUNNING) -> MOTOR_STOPPING once
// either plannedPulse.run_duration elapses OR the caller passes
// stop_early = true -- the hook for a controller's own early-exit
// conditions (e.g. demand has moved back to normal speed, the target has
// reversed) that should cut the pulse short regardless of its planned
// duration. MOTOR_STOPPED is left untouched here -- deciding whether/how to
// leave it is the caller's job (see PulseMotorModel_StartPulse).
void PulseMotorModel_Advance(PulseMotorState *s, const PulseMotorModel *model, float dt, bool stop_early);

#endif // PULSE_MOTOR_MODEL_H
