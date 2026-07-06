#include "pulse_mpc.h"
#include <math.h>
#include <stdio.h>

void PulseMPC_GetDebugInfo(const PulseMPC *c, float desiredVelocity, SVCDebugInfo *out) {
  out->state = c->state;
  out->passThrough = (c->state == MOTOR_STOPPED) && (c->filteredVelocityMagnitude >= c->model.V_min);
  out->dir = c->dir;
  out->desiredVelocity = desiredVelocity;
  out->commandVelocity = c->commandVelocity;
  out->motionDebt = c->motionDebt;
  out->filteredVelocityMagnitude = c->filteredVelocityMagnitude;
  out->filteredVelocity = c->filteredVelocity;
}

void PulseMPC_PrintDebugInfo(int index, SVCDebugInfo info) {
  const char *state_str;
  switch (info.state) {
    case MOTOR_STOPPED:  state_str = "STOPPED";  break;
    case MOTOR_STARTING: state_str = "STARTING"; break;
    case MOTOR_RUNNING:  state_str = "RUNNING";  break;
    case MOTOR_STOPPING: state_str = "STOPPING"; break;
    default:              state_str = "?";        break;
  }
  printf(
      "SVC[%d]: %-8s mode=%-9s dir=%+d desired=%6d cmd=%6d debt=%6d filtVel=%6d filtVelSigned=%6d (milli-rad/s, milli-rad)\r\n",
      index, state_str, info.passThrough ? "PASSTHRU" : "SMALL_VEL", (int) info.dir,
      (int) (info.desiredVelocity * 1000.0f), (int) (info.commandVelocity * 1000.0f),
      (int) (info.motionDebt * 1000.0f), (int) (info.filteredVelocityMagnitude * 1000.0f),
      (int) (info.filteredVelocity * 1000.0f));
}

float PulseMPC_EvaluateCost(const PulseMPC *c, MpcAction action, float desiredVelocity) {
  switch (action) {
    case ACTION_IDLE:
      return c->W_debt * fabsf(c->motionDebt);

    case ACTION_START: {
      // Use the direction START would actually commit to (sign_f(desiredVelocity)),
      // not c->dir -- c->dir is whatever direction was last latched by a
      // previous START and can be stale/wrong-signed while MOTOR_STOPPED
      // (e.g. at startup, or after a prior cycle ran the opposite way).
      // Evaluating this cost with a stale/mismatched dir would make START
      // look like it INCREASES predicted debt magnitude whenever dir and
      // the real demand disagree in sign, so START would never beat IDLE
      // and the controller would never correct dir -- a permanent deadlock.
      float dir_candidate = sign_f(desiredVelocity);
      float horizon_remaining = c->H - c->model.T_start;
      if (horizon_remaining < 0.0f) horizon_remaining = 0.0f;
      float predicted = c->motionDebt - dir_candidate * c->model.gain * c->model.V_min * horizon_remaining;
      return c->W_debt * fabsf(predicted) + c->W_switch;
    }

    case ACTION_CONTINUE: {
      float predicted = c->motionDebt - c->dir * c->model.gain * c->model.V_min * c->H;
      return c->W_debt * fabsf(predicted) + c->W_pulse * c->pulseTimer;
    }

    case ACTION_STOP: {
      float D_stop = c->model.V_min * c->model.gain * c->model.T_stop;
      float predicted = c->motionDebt - c->dir * D_stop;
      return c->W_debt * fabsf(predicted) + c->W_switch;
    }
  }
  return PULSE_MPC_UNREACHABLE_COST; // unreachable
}

void PulseMPC_ApplyAction(PulseMPC *c, MpcAction action, float desiredVelocity) {
  switch (action) {
    case ACTION_IDLE:
      c->commandVelocity = 0.0f;
      break;

    case ACTION_START:
      c->state = MOTOR_STARTING;
      c->stateTimer = 0.0f;
      c->pulseTimer = 0.0f;
      c->dir = sign_f(desiredVelocity);
      c->commandVelocity = c->dir * c->model.V_min;
      break;

    case ACTION_CONTINUE:
      c->commandVelocity = c->dir * c->model.V_min;
      break;

    case ACTION_STOP:
      c->state = MOTOR_STOPPING;
      c->stateTimer = 0.0f;
      c->commandVelocity = 0.0f; // hard stop: cut velocity in the same instant
      break;
  }
}

void PulseMPC_ValidateWeights(PulseMPC *c) {
  if (c->W_debt <= 0.0f) {
    printf(
        "WARNING: PulseMPC W_debt (%d milli) must be > 0 -- clamping to %d milli. "
        "A zero/negative W_debt makes START permanently more expensive than IDLE "
        "regardless of motionDebt, the same failure mode an unbounded W_switch causes.\r\n",
        (int) (c->W_debt * 1000.0f), (int) (PULSE_MPC_MIN_W_DEBT * 1000.0f));
    c->W_debt = PULSE_MPC_MIN_W_DEBT;
  }

  float horizon_remaining = c->H - c->model.T_start;
  if (horizon_remaining < 0.0f) horizon_remaining = 0.0f;
  float max_w_switch = c->W_debt * c->model.gain * c->model.V_min * horizon_remaining;

  if (c->W_switch >= max_w_switch) {
    float capped = 0.95f * max_w_switch;
    printf(
        "WARNING: PulseMPC W_switch (%d milli) >= W_debt*gain*V_min*(H-T_start) (%d milli) -- "
        "START would never beat IDLE once motionDebt is large, so the motor would never move "
        "no matter how much error accumulates. Capping W_switch to %d milli (95%% of margin).\r\n",
        (int) (c->W_switch * 1000.0f), (int) (max_w_switch * 1000.0f), (int) (capped * 1000.0f));
    c->W_switch = capped;
  }
}

void PulseMPC_Update(PulseMPC *c, float desiredVelocity, float dt) {
  // Slow EMAs of desiredVelocity, updated every tick regardless of state.
  // filteredVelocityMagnitude drives the MOTOR_STOPPED pass-through
  // decision below; filteredVelocity (signed) drives the RUNNING early-exit
  // checks further down.
  c->filteredVelocityMagnitude =
      c->filterAlpha * fabsf(desiredVelocity) + (1.0f - c->filterAlpha) * c->filteredVelocityMagnitude;
  c->filteredVelocity =
      c->filterAlpha * desiredVelocity + (1.0f - c->filterAlpha) * c->filteredVelocity;

  float actualVelocityEstimate =
      (c->state == MOTOR_RUNNING) ? (c->dir * c->model.gain * c->model.V_min) : 0.0f;
  // Deadband applies only to the desired side of the debt accumulation --
  // command noise/jitter below this magnitude shouldn't slowly build into a
  // debt that eventually forces a spurious pulse. actualVelocityEstimate
  // (the paid-down side) is unaffected.
  float debt_input = (fabsf(desiredVelocity) < c->debtDeadband) ? 0.0f : desiredVelocity;
  c->motionDebt += debt_input * dt;
  c->motionDebt -= actualVelocityEstimate * dt;

  // Advance transient timers -- note this can flip MOTOR_STARTING->RUNNING
  // or MOTOR_STOPPING->STOPPED within this same call, which is exactly why
  // the decision blocks below check c->state AFTER this switch runs, not
  // before: deciding on stale pre-switch state would mean a state that
  // just became MOTOR_STOPPED this tick skips straight past the
  // pass-through check into the plain IDLE-vs-START cost comparison, which
  // (if debt is still large) can choose START again immediately and undo
  // an intended pass-through hand-off before it ever takes effect.
  switch (c->state) {
    case MOTOR_STARTING:
      c->stateTimer += dt;
      if (c->stateTimer >= c->model.T_start) {
        c->state = MOTOR_RUNNING;
        c->stateTimer = 0.0f;
      }
      break;

    case MOTOR_STOPPING:
      c->stateTimer += dt;
      if (c->stateTimer >= c->model.T_stop) {
        c->state = MOTOR_STOPPED;
        c->stateTimer = 0.0f;
      }
      break;

    case MOTOR_RUNNING:
      c->pulseTimer += dt;
      break;

    case MOTOR_STOPPED:
      break;
  }

  if (c->state == MOTOR_STOPPED) {
    // Pass-through eligibility is checked FIRST, ahead of (and instead of)
    // the normal IDLE-vs-START cost comparison -- see the note above the
    // switch statement for why this can't be a separate early-return at
    // the top of the function.
    if (c->filteredVelocityMagnitude >= c->model.V_min) {
      c->commandVelocity = desiredVelocity;
      c->motionDebt = 0.0f; // don't carry stale debt into the next small-velocity episode
      return;
    }

    MpcAction bestAction = ACTION_IDLE;
    float bestCost = PulseMPC_EvaluateCost(c, ACTION_IDLE, desiredVelocity);

    float startCost = PulseMPC_EvaluateCost(c, ACTION_START, desiredVelocity);
    if (startCost < bestCost) {
      bestCost = startCost;
      bestAction = ACTION_START;
    }

    PulseMPC_ApplyAction(c, bestAction, desiredVelocity);
  }
  else if (c->state == MOTOR_RUNNING) {
    // Three unconditional (cost-comparison-bypassing, minimumPulseWidth-
    // bypassing) early exits, checked in order, ahead of the normal
    // CONTINUE/STOP cost comparison below. All three are "we already know
    // this pulse should end" cases where waiting for motionDebt to
    // accumulate enough to tip the cost comparison would just mean running
    // pointlessly (or wrongly) for longer than necessary.
    if (c->filteredVelocityMagnitude >= c->model.V_min) {
      // 1. Demand has clearly moved back into normal/pass-through range --
      // stop so control can return to MOTOR_STOPPED and hand off to
      // pass-through as soon as possible. Without this, J(CONTINUE) credits
      // a full gain*V_min*H of assumed debt repayment every tick, which
      // stays cheaper than J(STOP) indefinitely once motionDebt is large,
      // so the controller could otherwise stay stuck bang-banging at V_min
      // long after it should have handed off to pass-through.
      PulseMPC_ApplyAction(c, ACTION_STOP, desiredVelocity);
    } else if (fabsf(c->filteredVelocity) < 0.1f * c->model.V_min) {
      // 2. Commanded velocity has dropped to (near) zero -- no reason to
      // keep pulsing in the old direction. motionDebt is reset here for
      // the same reason as case 3 below: it was accumulated relative to a
      // demand that no longer applies, so it isn't a real obligation
      // anymore -- see case 3's comment for why leaving it non-zero is
      // actively harmful, not just stale.
      PulseMPC_ApplyAction(c, ACTION_STOP, desiredVelocity);
      c->motionDebt = 0.0f;
    } else if (sign_f(c->filteredVelocity) != c->dir) {
      // 3. Demand has reversed direction while still below V_min -- stop
      // so the next START can re-latch dir correctly, rather than
      // continuing to run (and accumulate debt) in the wrong direction
      // until the cost comparison eventually catches up.
      //
      // motionDebt MUST also be reset here, not just the state: it was
      // accumulated while running in the OLD direction, so it carries the
      // OLD direction's sign. If left alone, the next START's cost
      // (motionDebt - dir_candidate*gain*V_min*horizon) would combine a
      // stale, wrong-signed debt with the new (correct) dir_candidate --
      // subtracting a term of the opposite sign from debt INCREASES
      // |predicted| instead of reducing it, making START in the new
      // (correct) direction look permanently worse than IDLE. That
      // deadlocks the motor at rest, unable to ever start moving in the
      // new direction -- the exact "direction gets stuck" symptom this
      // fixes.
      PulseMPC_ApplyAction(c, ACTION_STOP, desiredVelocity);
      c->motionDebt = 0.0f;
    } else {
      MpcAction bestAction = ACTION_CONTINUE;
      float bestCost = PulseMPC_EvaluateCost(c, ACTION_CONTINUE, desiredVelocity);

      if (c->pulseTimer >= c->model.minimumPulseWidth) {
        float stopCost = PulseMPC_EvaluateCost(c, ACTION_STOP, desiredVelocity);
        if (stopCost < bestCost) {
          bestCost = stopCost;
          bestAction = ACTION_STOP;
        }
      }

      PulseMPC_ApplyAction(c, bestAction, desiredVelocity);
    }
  }
  // MOTOR_STARTING / MOTOR_STOPPING: no decision, timer-advance only (above).
}
