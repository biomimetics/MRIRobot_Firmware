#include "encoder_state_observer.h"

#include <math.h>

static float Clampf(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

void EncoderStateObserver_Init(EncoderStateObserver *o, float dt) {
  o->position = 0.0f;
  o->velocity = 0.0f;
  o->disturbance = 0.0f;

  o->dt = dt;

  // Default gains -- see each field's comment in the header for units and
  // reasoning. Lp/Lv inherited from possible_additions/observer.c's
  // conservative defaults; the rest sized for dt = 10 ms.
  o->Lp = 0.20f;
  o->Lv = 12.0f;
  o->Lvv = 0.10f;
  o->Ld = 0.1f; // non-zero but we should keep it small so we don't overestimate disturbances, was 4.0f
  o->disturbance_decay = 1.5f; // was 0.5f
  o->max_disturbance = 1.0f;

  o->innovation_gate = 0.15f;      // rad -- just above max_speed * dt, see header
  o->max_velocity_error = 5.0f;    // rad/s

  o->position_seeded = false;
  o->last_innovation = 0.0f;
  o->last_innovation_clamped = false;
}

void EncoderStateObserver_Update(EncoderStateObserver *o,
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
      Predict -- constant velocity + disturbance
  ----------------------------------------------------------*/

  // Semi-implicit Euler: velocity first, then position from the NEW
  // velocity (same as possible_additions/observer.c).
  o->velocity += o->disturbance * o->dt;
  o->position += o->velocity * o->dt;

  o->disturbance *= 1.0f - o->disturbance_decay * o->dt;

  /*----------------------------------------------------------
      Correct -- position innovation (primary)
  ----------------------------------------------------------*/

  float ep = measured_position - o->position;
  o->last_innovation = ep;
  o->last_innovation_clamped = (fabsf(ep) > o->innovation_gate);
  ep = Clampf(ep, -o->innovation_gate, o->innovation_gate);

  o->position += o->Lp * ep;
  o->velocity += o->Lv * ep * o->dt;
  o->disturbance += o->Ld * ep * o->dt;
  o->disturbance = Clampf(o->disturbance, -o->max_disturbance, o->max_disturbance);

  /*----------------------------------------------------------
      Correct -- measured-velocity trim (deweighted, see header)
  ----------------------------------------------------------*/

  float ev = measured_velocity - o->velocity;
  ev = Clampf(ev, -o->max_velocity_error, o->max_velocity_error);
  o->velocity += o->Lvv * ev;
}
