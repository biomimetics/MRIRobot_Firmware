#ifndef DPOS_PULSE_MPC_H
#define DPOS_PULSE_MPC_H

#include "motor_model.h"
#include <stdbool.h>

// Delta-position pulse planner: the alternative to pulse_mpc.h's
// motion-debt design (see
// src/lib/Drivers/SmallDeltaPControl/Small_DeltaP_Controller_Plan.md for
// the full design rationale, and dpos_pulse_mpc_planning.md for the exact
// cost/lookahead math implemented below). Deliberately kept separate from
// pulse_mpc.c rather than added alongside it: given a remaining position
// error, plan pulses to close it directly, with no motion-debt integral
// anywhere in it.
//
// Shares MotorModel/MotorState with pulse_mpc.h (both bang-bang the exact
// same physical motor) via motor_model.h -- see that header.
//
// Mode arbitration (ordinary pass-through vs. this small-velocity bang-bang
// planner) is decided inside DposPulseMPC_Update itself, at MOTOR_STOPPED,
// same as PulseMPC_Update -- see that function's comment for why it can't
// live one level up in the reactor instead (the velocity EMA the decision
// depends on has to keep updating every tick regardless of which mode is
// currently active). Small_DeltaP_Controller_Plan.md §2.3 is the source for
// exactly what folds into that decision.

// Set to 1 to enable periodic diagnostic printouts of all 7
// Small_DeltaP_Controller instances (see Small_DeltaP_Controller_Bank.lf's
// print reaction). 0 by default to avoid flooding the UART during normal
// operation.
#define PRINT_DPOS 0

// Number of candidates evaluated on each side of the naive run_duration
// estimate during DposPulseMPC_PlanBestPulse's grid search (see
// dpos_pulse_mpc_planning.md §7 item 3). Total candidates tried per call is
// 2*this + 1 (the +1 is the naive estimate itself, evaluated separately
// below the loop) -- 2 keeps that at 5, down from the original 7, since the
// depth-2 recursive search (this grid nested inside itself, see
// DposPulseMPC_ContinuationCost) made the original 7-wide/5-sigma-point
// combination too expensive to run every 1ms across 7 joints on the F446RE
// (see UART.lf's transmit-cadence regression this caused).
#define DPOS_MPC_GRID_HALF_WIDTH 2

// A single planned pulse: direction and how long to spend in MOTOR_RUNNING
// (excluding T_start/T_stop dead time, which contribute zero motion under
// the current model). See dpos_pulse_mpc_planning.md §2.
typedef struct {
  float dir;          // +1.0f or -1.0f
  float run_duration; // s
} PulseCommand;

// Per-motor controller state and tuning.
typedef struct {
  MotorModel model;

  // ---- MPC tuning (see dpos_pulse_mpc_planning.md §7) ----
  // All placeholders pending bench characterization -- same "TODO:
  // characterize" treatment as model.V_min/gain/etc. Unlike PulseMPC's
  // W_debt/W_switch, there is no coupled inequality between these that can
  // silently deadlock STOPPED->START (see planning doc §7 item 4), so there
  // is no PulseMPC_ValidateWeights analogue here.
  float W_error;      // base quadratic coefficient on resultingError (undershoot side)
  float k_overshoot;  // multiplier on the quadratic term, overshoot side -- start at 10.0
  float C_overshoot;  // flat additional cost charged for overshooting at all
  float sigma0;       // rad, stddev of a pulse's actual_delta around its predicted value
  int lookahead_depth; // pulses scored deep by PlanBestPulse's search; 2 is the practical target
  float grid_step_sigmas; // spacing between grid candidates, in units of sigma0/speed

  // Slow EMA filter weight (0..1) on desiredVelocity, driving both the
  // pass-through/small-velocity mode decision and the RUNNING safety
  // early-exits below -- same role/units as PulseMPC's filterAlpha.
  float filterAlpha;

  // rad -- small-velocity (bang-bang) mode only engages once |remainingError|
  // is at or below this, in addition to the velocity condition below
  // (Small_DeltaP_Controller_Plan.md §2.3): while the remaining gap is still
  // large, ordinary continuous velocity control is trusted to close most of
  // it faster than pulsing would. TODO: characterize -- likely on the order
  // of a few pulses' worth of travel at V_min.
  float engagementGapThreshold;

  // ---- Controller state ----
  MotorState state;
  float dir;                 // +1.0/-1.0, latched at STOPPED->START, held until back to STOPPED
  float stateTimer;          // s, elapsed since entering STARTING/STOPPING
  float pulseTimer;          // s, elapsed since entering RUNNING
  float remainingError;      // rad, remaining position error left to close --
                              // sourced directly from the incoming remaining-
                              // error signal, refreshed each time a new pulse
                              // is planned (see the plan doc's design decisions
                              // for reset rules).
  PulseCommand plannedPulse; // the pulse currently committed to (valid from
                              // STARTING through STOPPING)
  float commandVelocity;     // rad/s, signed, last commanded output
  float filteredVelocityMagnitude; // rad/s, EMA of |desiredVelocity| -- drives
                                    // the mode decision and the RUNNING
                                    // pass-through-recovery early-exit below.
                                    // NOTE: unlike PulseMPC, there is no
                                    // signed filteredVelocity/desiredVelocity-
                                    // based reversal check here -- desiredVelocity
                                    // legitimately sits near zero throughout
                                    // small-velocity operation for this design
                                    // (it's remainingError, not desiredVelocity,
                                    // that drives pulse direction/duration), so
                                    // the RUNNING reversal check below is keyed
                                    // to remainingError's sign instead. See
                                    // DposPulseMPC_Update's comment.
  bool passThrough;                 // 1 if the last MOTOR_STOPPED decision
                                    // chose ordinary pass-through control,
                                    // 0 if small-velocity mode is engaged --
                                    // for telemetry only, mirrors
                                    // SVCDebugInfo.passThrough.
} DposPulseMPC;

// Snapshot of a single controller's status, for diagnostics only (see
// PRINT_DPOS above) -- this design's analogue of SVCDebugInfo.
typedef struct {
  MotorState state;
  bool passThrough;
  float remainingError;
  PulseCommand plannedPulse;
  float commandVelocity;
} DposPulseMPCDebugInfo;

void DposPulseMPC_GetDebugInfo(const DposPulseMPC *c, DposPulseMPCDebugInfo *out);

// Prints one line of status for a single controller (milli-units, scaled to
// int, to avoid floating-point printf on this embedded target -- matches
// PulseMPC_PrintDebugInfo's convention).
void DposPulseMPC_PrintDebugInfo(int index, DposPulseMPCDebugInfo info);

// Deterministic point estimate of a candidate pulse's effect (planning doc
// §3): dir * gain * V_min * run_duration. The real motor won't match this
// exactly -- see DposPulseMPC_EvaluateLookahead for how that's accounted for.
float DposPulseMPC_PredictDelta(const MotorModel *model, PulseCommand pulse);

// Piecewise loss on the position error that would remain after a pulse
// resolves to resultingError, given the original remainingError it was
// meant to close (planning doc §4). Quadratic in resultingError, scaled up
// by k_overshoot and offset by C_overshoot on the overshoot side (crossed
// past the target) vs. a plain quadratic on the undershoot side.
// Discontinuous at the crossing point by design -- harmless, since this is
// only ever evaluated pointwise (grid search / quadrature), never
// differentiated.
float DposPulseMPC_ErrorLoss(const DposPulseMPC *c, float remainingError, float resultingError);

// Recursively scores candidate `pulse` as the first pulse from
// remainingError, assuming the best available continuation for the
// remaining depth-1 pulses (planning doc §5). depth == 1 reduces exactly to
// E[ErrorLoss(...)], with no continuation term (no per-pulse penalty in this
// design -- planning doc §7 item 4). The expectation over the pulse's
// Gaussian noise (§3) is approximated with a small fixed quadrature, reused
// for every expectation in this file rather than deriving a separate
// closed-form truncated-normal expression.
float DposPulseMPC_EvaluateLookahead(const DposPulseMPC *c, PulseCommand pulse, float remainingError, int depth);

// Searches candidate PulseCommands (direction fixed by sign(remainingError);
// run_duration is the free variable, searched via a small discrete grid
// centered on the naive point estimate that would zero remainingError in
// expectation -- planning doc §7 item 3) for the one minimizing
// EvaluateLookahead at the given depth.
PulseCommand DposPulseMPC_PlanBestPulse(const DposPulseMPC *c, float remainingError, int depth);

// Hybrid pass-through / small-velocity (bang-bang) update, run every
// control_period tick, mirroring PulseMPC_Update's signature and calling
// convention. dt is real elapsed time in seconds since the previous call.
//
// Every call updates filteredVelocityMagnitude regardless of mode. Only at
// MOTOR_STOPPED is the pass-through vs. small-velocity decision (re-)made:
// ordinary continuous velocity control is used (commandVelocity =
// desiredVelocity) unless BOTH filteredVelocityMagnitude < model.V_min AND
// |remainingError| <= engagementGapThreshold (Small_DeltaP_Controller_Plan.md
// §2.3) -- otherwise small-velocity mode's STOPPED->START gate runs: a
// direct cost comparison (mirroring PulseMPC_EvaluateCost's ACTION_IDLE vs
// ACTION_START comparison, not a separately hand-derived formula) between
// doing nothing (leaving remainingError exactly as it is) and the best pulse
// PlanBestPulse can find -- the "is this worth the dead-time cost at all"
// floor from Small_DeltaP_Controller_Plan.md §2.2/§3 item 3. Once started,
// the pulse executes open-loop for its planned run_duration -- no per-tick
// stopping-distance recheck (planning doc's intro note and §6).
//
// While RUNNING, two unconditional early exits can cut a pulse short
// regardless of its planned run_duration -- ported from PulseMPC_Update's
// RUNNING checks, but the near-zero/reversal check is deliberately keyed to
// remainingError rather than desiredVelocity (see the struct's
// filteredVelocityMagnitude comment for why): (1) filteredVelocityMagnitude
// has climbed back to/above V_min -- demand has clearly moved into
// pass-through range, so stop and hand control back as soon as possible; (2)
// the freshly-supplied remainingError's sign no longer matches the latched
// dir -- the target has moved past/reversed relative to what this pulse was
// planned against, so abort rather than keep running the wrong way.
void DposPulseMPC_Update(DposPulseMPC *c, float desiredVelocity, float remainingError, float dt);

#endif // DPOS_PULSE_MPC_H
