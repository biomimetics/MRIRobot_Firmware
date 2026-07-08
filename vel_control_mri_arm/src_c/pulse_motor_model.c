#include "pulse_motor_model.h"
#include <math.h>

float sign_f(float x) {
  return (x >= 0.0f) ? 1.0f : -1.0f;
}

const char *MotorState_Name(MotorState s) {
  switch (s) {
    case MOTOR_STOPPED: return "STOPPED";
    case MOTOR_STARTING: return "STARTING";
    case MOTOR_RUNNING: return "RUNNING";
    case MOTOR_STOPPING: return "STOPPING";
    default: return "?";
  }
}

float PulseMotorModel_PredictDelta(const PulseMotorModel *model, PulseCommand pulse) {
  return pulse.dir * model->V_real.mean * pulse.run_duration;
}

GaussianRV PulseMotorModel_PredictDeltaDistribution(const PulseMotorModel *model, PulseCommand pulse) {
  GaussianRV delta_dist;
  delta_dist.mean = pulse.dir * model->V_real.mean * pulse.run_duration;
  delta_dist.variance = model->V_real.variance * pulse.run_duration * pulse.run_duration;

  return delta_dist;
}

GaussianRV PulseMotorModel_PredictVelocityDistribution(GaussianRV delta_dist, PulseCommand pulse) {
  GaussianRV vel_dist;
  vel_dist.mean = pulse.dir * delta_dist.mean / pulse.run_duration;
  vel_dist.variance = delta_dist.variance / (pulse.run_duration * pulse.run_duration);
  return delta_dist;
}

PulseMotorModel PulseMotorModel_PointEstimateFromObservations(const PulseMotorModel *current, float t_start_s, float t_stop_s, GaussianRV v_real_estimate) {
  PulseMotorModel out = *current;
  out.V_real = v_real_estimate;
  out.T_start = t_start_s;
  out.T_stop = t_stop_s;
  return out;
}

PulseMotorModel PulseMotorModel_Subtract(PulseMotorModel a, PulseMotorModel b) {
  PulseMotorModel out;
  out.V_min_cmd = a.V_min_cmd - b.V_min_cmd;
  out.V_real.mean = a.V_real.mean - b.V_real.mean;
  out.V_real.variance = a.V_real.variance - b.V_real.variance;
  out.T_start = a.T_start - b.T_start;
  out.T_stop = a.T_stop - b.T_stop;
  out.minimumPulseWidth = a.minimumPulseWidth - b.minimumPulseWidth;
  return out;
}

void PulseMotorModelSampleStats_Init(PulseMotorModelSampleStats *s) {
  SampleStats_Init(&s->V_min_cmd);
  SampleStats_Init(&s->V_real_mean);
  SampleStats_Init(&s->V_real_variance);
  SampleStats_Init(&s->T_start);
  SampleStats_Init(&s->T_stop);
  SampleStats_Init(&s->minimumPulseWidth);
}

void PulseMotorModelSampleStats_Add(PulseMotorModelSampleStats *s, PulseMotorModel sample) {
  SampleStats_Add(&s->V_min_cmd, sample.V_min_cmd);
  SampleStats_Add(&s->V_real_mean, sample.V_real.mean);
  SampleStats_Add(&s->V_real_variance, sample.V_real.variance);
  SampleStats_Add(&s->T_start, sample.T_start);
  SampleStats_Add(&s->T_stop, sample.T_stop);
  SampleStats_Add(&s->minimumPulseWidth, sample.minimumPulseWidth);
}

// right now this is unused, main plan pulse function is in method of low-level controller reactor
PulseCommand PulseMotorModel_PlanPulseFromPositionDelta(const PulseMotorModel *model, float remainingPositionDelta) {
  PulseCommand pulse;
  pulse.pulse_height = model->V_min_cmd; // simple approach of setting the command to be the lowest stable velocity
  pulse.dir = sign_f(remainingPositionDelta);
  pulse.run_duration = fabsf(remainingPositionDelta) / (model->V_real.mean);
  return pulse;
}

PulseCommand PulseMotorModel_PlanPulseWithOvershootBound(const PulseMotorModel *model, float remainingPositionDelta, float max_overshoot_probability) {
  PulseCommand pulse;
  pulse.pulse_height = model->V_min_cmd;
  pulse.dir = sign_f(remainingPositionDelta);
  float distance = fabsf(remainingPositionDelta);
  float run_duration = GaussianMotion_ArrivalTimeInvCdf(max_overshoot_probability, distance, model->V_real);
  
  // I removed this check because I think it's more interesting if we don't prevent plans that are too small.
  // We may want to use that info elsewhere to trigger a convergence state, but we don't want to block them like this.
  //if (run_duration < model->minimumPulseWidth) {
  //  run_duration = model->minimumPulseWidth;
  //}

  if (run_duration > MAX_PULSE_DURATION_S) {
    run_duration = MAX_PULSE_DURATION_S;
  }
  pulse.run_duration = run_duration;
  return pulse;
}

void PulseMotorModel_StartPulse(PulseMotorState *s, const PulseMotorModel *model, PulseCommand pulse) {
  s->dir = pulse.dir;
  s->plannedPulse = pulse;
  s->state = MOTOR_STARTING;
  s->stateTimer = 0.0f;
  s->pulseTimer = 0.0f;
  s->commandVelocity = pulse.dir * pulse.pulse_height;
}

void PulseMotorModel_Advance(PulseMotorState *s, const PulseMotorModel *model, float dt, bool stop_early) {
  switch (s->state) {
    case MOTOR_STARTING: // command velocity should still be set from StartPulse 
      s->stateTimer += dt;
      s->pulseTimer += dt;

      // guard for when pulse can be shorter than T_start
      if (s->pulseTimer >= (s->plannedPulse.run_duration)) { 
        // we should avoid this case if possible, but theoretically the pulse can be shorter than the communication delay so MOTOR_RUNNING should exit instantly when this happens.
        s->commandVelocity = 0.0f;
      }

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

    case MOTOR_RUNNING: // command velocity should still be set from StartPulse
    // when we transition into MOTOR_STOPPING, change the command velocity back to zero
      s->pulseTimer += dt;

      if (stop_early || s->pulseTimer >= (s->plannedPulse.run_duration)) {
        s->state = MOTOR_STOPPING;
        s->stateTimer = 0.0f;
        s->commandVelocity = 0.0f;
      }
      break;

    case MOTOR_STOPPED:
      break;
  }
}
