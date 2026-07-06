#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

// Physical/open-loop motor model and shared bang-bang FSM shape, split out
// of pulse_mpc.h so both pulse_mpc.c (motion-debt controller) and
// dpos_pulse_mpc.c (delta-position controller) can drive the same physical
// motor/pulse behavior without duplicating it -- only their planning logic
// on top of this differs. See each controller's own header for its
// decision/action types built on top of these.

// Physical/open-loop motor model, all placeholders (see each controller
// reactor's `startup` reaction for current values). A future live parameter
// estimator has a single, self-contained struct to update without touching
// either controller's own tuning or FSM state.
typedef struct {
  float V_min;              // rad/s, magnitude of the fixed pulse speed
  float gain;                // unitless 0..1, actual/commanded ratio
  float T_start;             // s, startup command delay
  float T_stop;              // s, hard-stop settle time
  float minimumPulseWidth;   // s, hysteresis guard before STOP eligible
} MotorModel;

// Shared FSM states for any controller that bang-bangs this motor between a
// fixed pulse speed and stopped, regardless of how it decides when to
// transition.
typedef enum {
  MOTOR_STOPPED,
  MOTOR_STARTING,
  MOTOR_RUNNING,
  MOTOR_STOPPING
} MotorState;

// +1.0f if x >= 0.0f, else -1.0f. Shared tiny helper -- used anywhere a
// commanded/measured velocity's direction needs latching.
float sign_f(float x);

#endif // MOTOR_MODEL_H
