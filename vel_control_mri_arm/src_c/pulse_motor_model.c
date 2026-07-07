#include "pulse_motor_model.h"
#include <math.h>

float sign_f(float x) {
  return (x >= 0.0f) ? 1.0f : -1.0f;
}

float PulseMotorModel_PredictDelta(const MotorModel *model, PulseCommand pulse) {
  return pulse.dir * model->gain * model->V_min * pulse.run_duration;
}

PulseCommand PulseMotorModel_PlanPulseFromPositionDelta(const MotorModel *model, float remainingPositionDelta) {
  PulseCommand pulse;
  pulse.dir = sign_f(remainingPositionDelta);
  pulse.run_duration = fabsf(remainingPositionDelta) / (model->gain * model->V_min);
  return pulse;
}

void PulseMotorModel_StartPulse(PulseMotorState *s, const MotorModel *model, PulseCommand pulse) {
  s->dir = pulse.dir;
  s->plannedPulse = pulse;
  s->state = MOTOR_STARTING;
  s->stateTimer = 0.0f;
  s->pulseTimer = 0.0f;
  s->commandVelocity = pulse.dir * model->V_min;
}

void PulseMotorModel_Advance(PulseMotorState *s, const MotorModel *model, float dt, bool stop_early) {
  switch (s->state) {
    case MOTOR_STARTING:
      s->stateTimer += dt;
      if (s->stateTimer >= model->T_start) {
        s->state = MOTOR_RUNNING;
        s->stateTimer = 0.0f;
      }
      break;

    case MOTOR_STOPPING:
      s->stateTimer += dt;
      if (s->stateTimer >= model->T_stop) {
        s->state = MOTOR_STOPPED;
        s->stateTimer = 0.0f;
      }
      break;

    case MOTOR_RUNNING:
      s->pulseTimer += dt;
      if (stop_early || s->pulseTimer >= s->plannedPulse.run_duration) {
        s->state = MOTOR_STOPPING;
        s->stateTimer = 0.0f;
        s->commandVelocity = 0.0f;
      }
      break;

    case MOTOR_STOPPED:
      break;
  }
}
