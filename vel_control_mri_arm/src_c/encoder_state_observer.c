#include "encoder_state_observer.h"

#include <math.h>

static float Clampf(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

// seconds -> whole delay-line samples, kept inside the ring. Rounded, not
// truncated, so e.g. a 195 ms T_start at dt = 10 ms taps 20 samples back
// rather than 19.
static int DelaySamples(float delay_s, float dt) {
  int n = (int)(delay_s / dt + 0.5f);
  if (n < 0) n = 0;
  if (n > ESO_CMD_HISTORY_SIZE - 1) n = ESO_CMD_HISTORY_SIZE - 1;
  return n;
}

void EncoderStateObserver_Init(EncoderStateObserver *o, float dt, bool use_commanded_velocity) {
  o->position = 0.0f;
  o->velocity = 0.0f;
  o->disturbance = 0.0f;

  o->dt = dt;
  o->use_commanded_velocity = use_commanded_velocity;

  // Placeholder until the first SetModel -- zero delay just means the
  // taps read the freshest command, which is wrong but harmless for the
  // handful of ticks before EncoderStateEstimator.lf's startup reaction
  // installs INITIAL_MOTOR_MODEL.
  o->v_real_mean = 0.0f;
  o->v_min_cmd = 0.0f;
  o->start_delay_samples = 0;
  o->stop_delay_samples = 0;

  // Default gains -- see each field's comment in the header for units and
  // reasoning. Lp/Lv inherited from possible_additions/observer.c's
  // conservative defaults; the rest sized for dt = 10 ms.
  o->tau = 0.100f;
  o->Lp = 0.20f;
  o->Lv = 12.0f;
  o->Lvv = 0.10f;
  o->Ld = 4.0f;
  o->disturbance_decay = 0.5f;
  o->max_disturbance = 5.0f;

  o->innovation_gate = 0.15f;      // rad -- just above max_speed * dt, see header
  o->rest_innovation_gate = 0.02f; // rad -- ~2x the 0.01 rad movement threshold, see header
  o->rest_gain_scale = 0.05f;
  o->rest_cmd_threshold = 0.05f;   // rad/s
  o->rest_vel_threshold = 0.05f;   // rad/s
  o->max_velocity_error = 5.0f;    // rad/s

  for (int i = 0; i < ESO_CMD_HISTORY_SIZE; i++) {
    o->cmd_history[i] = 0.0f;
  }
  o->cmd_index = 0;

  o->position_seeded = false;
  o->last_innovation = 0.0f;
  o->last_innovation_clamped = false;
  o->at_rest = false;
}

void EncoderStateObserver_SetModel(EncoderStateObserver *o, const PulseMotorModel *model) {
  o->v_real_mean = model->V_real.mean;
  o->v_min_cmd = model->V_min_cmd;
  o->start_delay_samples = DelaySamples(model->T_start, o->dt);
  o->stop_delay_samples = DelaySamples(model->T_stop, o->dt);
}

void EncoderStateObserver_Update(EncoderStateObserver *o,
                                 float commanded_velocity,
                                 MotorState motor_state,
                                 float measured_position,
                                 float measured_velocity) {
  // First sample seeds the estimate outright -- see position_seeded's
  // comment in the header.
  if (!o->position_seeded) {
    o->position = measured_position;
    o->velocity = measured_velocity;
    o->position_seeded = true;
  }

  /*----------------------------------------------------------
      Command delay line
  ----------------------------------------------------------*/

  o->cmd_history[o->cmd_index] = commanded_velocity;
  uint16_t head = o->cmd_index;
  o->cmd_index = (uint16_t)((o->cmd_index + 1u) % ESO_CMD_HISTORY_SIZE);

  // Two taps into the same history: what the command was T_start ago
  // (the slow tap) and T_stop ago (the fast tap). The motor's dead time
  // is asymmetric -- it takes T_start (~100-200 ms) to start moving but
  // only T_stop (~10 ms) to settle after the command drops -- so a single
  // T_start-delayed command would keep the expected velocity high for
  // ~T_start - T_stop after every stop, exactly the transient where the
  // measured velocity is already least trustworthy.
  float slow_cmd = o->cmd_history[(head + ESO_CMD_HISTORY_SIZE - (uint16_t)o->start_delay_samples) % ESO_CMD_HISTORY_SIZE];
  float fast_cmd = o->cmd_history[(head + ESO_CMD_HISTORY_SIZE - (uint16_t)o->stop_delay_samples) % ESO_CMD_HISTORY_SIZE];

  // Starts are slow, stops are fast: with both taps on the same side of
  // zero, the smaller magnitude wins -- a rising command doesn't take
  // effect until it shows up on the slow tap, a falling command takes
  // effect as soon as it shows up on the fast tap. Opposite signs means
  // we're mid-reversal: the old direction has already stopped (fast) but
  // the new one hasn't started (slow), so expect no motion at all.
  float expected_velocity;
  if (slow_cmd * fast_cmd >= 0.0f) {
    expected_velocity = (fabsf(fast_cmd) < fabsf(slow_cmd)) ? fast_cmd : slow_cmd;
  } else {
    expected_velocity = 0.0f;
  }

  // While a pulse is in flight the command sits at +-V_min_cmd but the
  // motor actually realizes ~V_real.mean -- rescale so the prediction
  // expects the realized speed, not the commanded one. Pass-through mode
  // (MOTOR_STOPPED with a live command) is left at unity: that path's
  // command-to-speed calibration is the USM driver's own business, and
  // any residual gain error lands in the disturbance state instead.
  bool in_pulse = (motor_state != MOTOR_STOPPED);
  if (in_pulse && o->v_min_cmd > 1e-6f) {
    expected_velocity *= o->v_real_mean / o->v_min_cmd;
  }

  /*----------------------------------------------------------
      Predict
  ----------------------------------------------------------*/

  float command_accel = 0.0f;
  if (o->use_commanded_velocity) {
    command_accel = (expected_velocity - o->velocity) / o->tau;
  }

  // Semi-implicit Euler: velocity first, then position from the NEW
  // velocity (same as possible_additions/observer.c).
  o->velocity += (command_accel + o->disturbance) * o->dt;
  o->position += o->velocity * o->dt;

  o->disturbance *= 1.0f - o->disturbance_decay * o->dt;

  /*----------------------------------------------------------
      Rest detection (EMI defense -- see the gate fields' header comments)
  ----------------------------------------------------------*/

  // Requires the command path: without a trustworthy "nothing is being
  // commanded" signal (SEA use), a quiet estimate alone can't rule out a
  // real slow motion, so no rest gating there. slow_cmd is checked as
  // well as the live command so a start that's still inside its T_start
  // dead time -- command up, motion legitimately imminent -- doesn't get
  // rest-gated right as the motor finally moves.
  o->at_rest = o->use_commanded_velocity && !in_pulse &&
               fabsf(commanded_velocity) < o->rest_cmd_threshold &&
               fabsf(slow_cmd) < o->rest_cmd_threshold &&
               fabsf(o->velocity) < o->rest_vel_threshold;

  float gate = o->at_rest ? o->rest_innovation_gate : o->innovation_gate;
  float gain_scale = o->at_rest ? o->rest_gain_scale : 1.0f;

  /*----------------------------------------------------------
      Correct -- position innovation (primary)
  ----------------------------------------------------------*/

  float ep = measured_position - o->position;
  o->last_innovation = ep;
  o->last_innovation_clamped = (fabsf(ep) > gate);
  ep = Clampf(ep, -gate, gate);

  o->position += gain_scale * o->Lp * ep;
  o->velocity += gain_scale * o->Lv * ep * o->dt;
  o->disturbance += gain_scale * o->Ld * ep * o->dt;
  o->disturbance = Clampf(o->disturbance, -o->max_disturbance, o->max_disturbance);

  /*----------------------------------------------------------
      Correct -- measured-velocity trim (deweighted, see header)
  ----------------------------------------------------------*/

  float ev = measured_velocity - o->velocity;
  ev = Clampf(ev, -o->max_velocity_error, o->max_velocity_error);
  o->velocity += gain_scale * o->Lvv * ev;
}
