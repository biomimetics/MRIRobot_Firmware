#include "dpos_pulse_mpc.h"
#include <math.h>
#include <stdio.h>

// Sigma-point quadrature nodes/weights, in units of sigma0, approximating
// integration against actual_delta's Gaussian noise (planning doc §5): a
// symmetric 5-point rule at {-2,-1,0,1,2} sigma, weighted by their
// approximate normal probability mass. One mechanism, reused by both the
// single-pulse cost (via EvaluateLookahead at depth 1) and the multi-pulse
// recursion -- see the planning doc's explicit "one mechanism, not two."
#define DPOS_MPC_NUM_SIGMA_POINTS 3
static const float kSigmaOffsets[DPOS_MPC_NUM_SIGMA_POINTS] = {-1.0f, 0.0f, 1.0f};
static const float kSigmaWeights[DPOS_MPC_NUM_SIGMA_POINTS] = {0.272f, 0.4545f, 0.272f};

void DposPulseMPC_GetDebugInfo(const DposPulseMPC *c, DposPulseMPCDebugInfo *out) {
  out->state = c->pulseState.state;
  out->passThrough = c->passThrough;
  out->remainingError = c->remainingError;
  out->plannedPulse = c->pulseState.plannedPulse;
  out->commandVelocity = c->pulseState.commandVelocity;
  out->pulseTimer = c->pulseState.pulseTimer;
  out->stateTimer = c->pulseState.stateTimer;
}

void DposPulseMPC_PrintDebugInfo(int index, DposPulseMPCDebugInfo info) {
  const char *state_str;
  switch (info.state) {
    case MOTOR_STOPPED:  state_str = "STOPPED";  break;
    case MOTOR_STARTING: state_str = "STARTING"; break;
    case MOTOR_RUNNING:  state_str = "RUNNING";  break;
    case MOTOR_STOPPING: state_str = "STOPPING"; break;
    default:              state_str = "?";        break;
  }
  printf(
      "DPOS[%d]: %-8s mode=%-9s remErr=%6d dir=%+d runDur=%6d cmd=%6d pulseT=%6d stateT=%6d (milli-rad, milli-rad/s, milli-s)\r\n",
      index, state_str, info.passThrough ? "PASSTHRU" : "SMALL_VEL",
      (int) (info.remainingError * 1000.0f), (int) info.plannedPulse.dir,
      (int) (info.plannedPulse.run_duration * 1000.0f),
      (int) (info.commandVelocity * 1000.0f),
      (int) (info.pulseTimer * 1000.0f),
      (int) (info.stateTimer * 1000.0f));
}

float DposPulseMPC_ErrorLoss(const DposPulseMPC *c, float remainingError, float resultingError) {
  float s = sign_f(remainingError);
  bool overshot = (s * resultingError) < 0.0f;
  if (overshot) {
    return c->k_overshoot * c->W_error * resultingError * resultingError + c->C_overshoot;
  }
  return c->W_error * resultingError * resultingError;
}

// Per-outcome cost of committing to `pulse` from `remainingError`, given one
// sampled actual_delta: the continuation is either §4's plain ErrorLoss
// (depth == 1, no further pulses to plan) or, for depth > 1, the cheaper of
// (a) stopping here and leaving resultingError as the final answer, or (b)
// planning and scoring one more best-effort pulse from resultingError. (a)
// matters even though there's no explicit per-pulse penalty in this design
// (planning doc §7 item 4): without it, this recursion would always assume
// a further pulse gets planned regardless of how small resultingError
// already is, which would wrongly penalize a first pulse that lands very
// close to zero (a forced, tiny second pulse there would itself only ever
// overshoot). Mirrors DposPulseMPC_WorthPulsing's own idle-vs-pulse
// comparison one level down. Mutually recursive with
// DposPulseMPC_PlanBestPulse/EvaluateLookahead below -- both are declared in
// the header, so ordering here is fine.
static float DposPulseMPC_ContinuationCost(
    const DposPulseMPC *c, float remainingError, float actual_delta, int depth) {
  float resultingError = remainingError - actual_delta;
  if (depth <= 1) {
    return DposPulseMPC_ErrorLoss(c, remainingError, resultingError);
  }

  float idleCost = DposPulseMPC_ErrorLoss(c, resultingError, resultingError);
  PulseCommand nextPulse = DposPulseMPC_PlanBestPulse(c, resultingError, depth - 1);
  float pulseCost = DposPulseMPC_EvaluateLookahead(c, nextPulse, resultingError, depth - 1);
  return (pulseCost < idleCost) ? pulseCost : idleCost;
}

float DposPulseMPC_EvaluateLookahead(const DposPulseMPC *c, PulseCommand pulse, float remainingError, int depth) {
  float mu = PulseMotorModel_PredictDelta(&c->model, pulse);

  float total = 0.0f;
  for (int i = 0; i < DPOS_MPC_NUM_SIGMA_POINTS; i++) {
    float actual_delta = mu + kSigmaOffsets[i] * c->sigma0;
    total += kSigmaWeights[i] * DposPulseMPC_ContinuationCost(c, remainingError, actual_delta, depth);
  }
  return total;
}

PulseCommand DposPulseMPC_PlanBestPulse(const DposPulseMPC *c, float remainingError, int depth) {
  float dir = sign_f(remainingError);
  float speed = c->model.gain * c->model.V_min_cmd;
  // Naive point estimate for the run_duration that would zero remainingError
  // in expectation (ignoring noise) -- grid center, planning doc §7 item 3.
  float naive_duration = fabsf(remainingError) / speed;

  PulseCommand best = {dir, c->model.minimumPulseWidth};
  float bestCost = DposPulseMPC_EvaluateLookahead(c, best, remainingError, depth);

  for (int i = -DPOS_MPC_GRID_HALF_WIDTH; i <= DPOS_MPC_GRID_HALF_WIDTH; i++) {
    if (i == 0) continue; // naive_duration itself is handled below

    float candidate_duration = naive_duration + (float) i * c->grid_step_sigmas * c->sigma0 / speed;
    if (candidate_duration < c->model.minimumPulseWidth) {
      candidate_duration = c->model.minimumPulseWidth;
    }

    PulseCommand candidate = {dir, candidate_duration};
    float cost = DposPulseMPC_EvaluateLookahead(c, candidate, remainingError, depth);
    if (cost < bestCost) {
      bestCost = cost;
      best = candidate;
    }
  }

  // Always also evaluate the naive estimate itself, clamped to the floor.
  {
    float candidate_duration = naive_duration;
    if (candidate_duration < c->model.minimumPulseWidth) {
      candidate_duration = c->model.minimumPulseWidth;
    }
    PulseCommand candidate = {dir, candidate_duration};
    float cost = DposPulseMPC_EvaluateLookahead(c, candidate, remainingError, depth);
    if (cost < bestCost) {
      bestCost = cost;
      best = candidate;
    }
  }

  return best;
}

// Is remainingError worth starting a pulse for at all -- the
// dead-time-cost floor from Small_DeltaP_Controller_Plan.md §2.2/§3 item 3,
// implemented as a direct cost comparison (mirroring
// PulseMPC_EvaluateCost's ACTION_IDLE vs ACTION_START comparison) between
// doing nothing (remainingError stays exactly as it is: resultingError ==
// remainingError, so ErrorLoss's overshoot branch can never trigger) and the
// best pulse PlanBestPulse can find. If bestPulseOut is non-NULL, the best
// candidate is written there regardless of the outcome (only meaningful
// when this returns true).
static bool DposPulseMPC_WorthPulsing(const DposPulseMPC *c, float remainingError, PulseCommand *bestPulseOut) {
  float idleCost = DposPulseMPC_ErrorLoss(c, remainingError, remainingError);
  PulseCommand best = DposPulseMPC_PlanBestPulse(c, remainingError, c->lookahead_depth);
  if (bestPulseOut) *bestPulseOut = best;
  float bestCost = DposPulseMPC_EvaluateLookahead(c, best, remainingError, c->lookahead_depth);
  return bestCost < idleCost;
}

void DposPulseMPC_Update(DposPulseMPC *c, float desiredVelocity, float remainingError, float dt) {
  // Same EMA role as PulseMPC's filteredVelocityMagnitude: drives both the
  // pass-through/small-velocity mode decision below and the RUNNING
  // pass-through-recovery early-exit. Updated unconditionally, every call,
  // regardless of which mode is currently active -- otherwise the STOPPED
  // branch below could never detect a rise back above V_min_cmd while
  // small-velocity mode was engaged (same reasoning as PulseMPC_Update).
  c->filteredVelocityMagnitude =
      c->filterAlpha * fabsf(desiredVelocity) + (1.0f - c->filterAlpha) * c->filteredVelocityMagnitude;

  // stop_early: this design's two RUNNING early-exit conditions, ported
  // from PulseMPC_Update's RUNNING checks but keyed to remainingError rather
  // than desiredVelocity -- see the struct's filteredVelocityMagnitude
  // comment for why (desiredVelocity legitimately sits near zero throughout
  // a dwell-region small-velocity episode for this design, so a
  // desiredVelocity-near-zero check would spuriously abort essentially
  // every pulse). Only meaningful while RUNNING -- computed here (rather
  // than inside PulseMotorModel_Advance) because both conditions are
  // decisions specific to this controller, not the generic pulse-execution
  // FSM.
  bool stop_early = false;
  if (c->pulseState.state == MOTOR_RUNNING) {
    if (c->filteredVelocityMagnitude >= c->model.V_min_cmd) {
      // 1. Demand has clearly moved back into pass-through range -- stop so
      // control can return to MOTOR_STOPPED and hand off as soon as
      // possible, same reasoning as PulseMPC_Update's equivalent check.
      stop_early = true;
    } else if (sign_f(remainingError) != c->pulseState.dir) {
      // 2. The freshly-supplied remainingError's sign no longer matches the
      // latched dir -- the target has moved past/reversed relative to what
      // this pulse was planned against. Abort, and discard any assumption
      // about remainingError in favor of the fresh value next planned from
      // STOPPED (plan doc §3 item 4).
      stop_early = true;
      c->remainingError = remainingError;
    }
  }

  // Advances MOTOR_STARTING->RUNNING, MOTOR_STOPPING->STOPPED, and (while
  // RUNNING) ->STOPPING once either plannedPulse.run_duration elapses or
  // stop_early is true -- see pulse_motor_model.h. Can flip state within
  // this same call; the decision block below checks the post-advance state,
  // same reasoning as PulseMPC_Update.
  PulseMotorModel_Advance(&c->pulseState, &c->model, dt, stop_early);

  if (c->pulseState.state == MOTOR_STOPPED) {
    // Pass-through eligibility is checked FIRST, ahead of (and instead of)
    // the small-velocity floor gate -- see the note above for why this
    // can't be a separate early-return at the top of the function. Engages
    // ordinary continuous velocity control whenever EITHER the filtered
    // velocity is already at/above V_min_cmd OR the remaining gap is still too
    // large to be worth closing via pulsing (Small_DeltaP_Controller_Plan.md
    // §2.3) -- small-velocity mode requires BOTH conditions to fail.
    if (c->filteredVelocityMagnitude >= c->model.V_min_cmd || fabsf(remainingError) > c->engagementGapThreshold) {
      c->passThrough = true;
      c->pulseState.commandVelocity = desiredVelocity;
      return;
    }
    c->passThrough = false;

    // Refreshed from scratch here, not dead-reckoned: with a continuously
    // available synthetic remainingError signal (this stage's test harness),
    // whatever value is freshest at the moment a new pulse is about to be
    // planned already reflects the real outcome of the previous pulse more
    // accurately than a model-predicted dead-reckoning estimate would.
    // Dead-reckoning only starts to matter once the real, intermittent wire
    // cadence lands (Small_DeltaP_Comms_Plan.md) and this same value can go
    // stale between messages.
    c->remainingError = remainingError;

    PulseCommand candidate;
    if (DposPulseMPC_WorthPulsing(c, c->remainingError, &candidate)) {
      PulseMotorModel_StartPulse(&c->pulseState, &c->model, candidate);
    } else {
      c->pulseState.commandVelocity = 0.0f;
    }
  }
  // MOTOR_STARTING / MOTOR_RUNNING (still mid-pulse) / MOTOR_STOPPING: no
  // decision here; commandVelocity already holds whatever
  // PulseMotorModel_StartPulse/PulseMotorModel_Advance set.
}
