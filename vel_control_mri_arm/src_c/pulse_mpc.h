#ifndef PULSE_MPC_H
#define PULSE_MPC_H

// Sentinel cost used to seed a min-cost search before any real candidate
// has been evaluated -- must be larger than any real J() value can reach.
#define PULSE_MPC_UNREACHABLE_COST 1e30f

// Floor W_debt gets clamped to if configured <= 0 -- see
// PulseMPC_ValidateWeights. Arbitrary small positive value; its only job is
// to keep the W_switch-vs-W_debt inequality meaningful, not to be a good
// tuning value on its own.
#define PULSE_MPC_MIN_W_DEBT 0.01f

// Set to 1 to enable periodic diagnostic printouts of all 7
// Small_Velocity_Controller instances (see Small_Velocity_Controller_Bank.lf's
// print reaction). 0 by default to avoid flooding the UART during normal
// operation.
#define PRINT_SVC 0

typedef enum {
  MOTOR_STOPPED,
  MOTOR_STARTING,
  MOTOR_RUNNING,
  MOTOR_STOPPING
} MotorState;

typedef enum {
  ACTION_IDLE,
  ACTION_START,
  ACTION_CONTINUE,
  ACTION_STOP
} MpcAction;

// Physical/open-loop motor model, all placeholders (see Small_Velocity_Controller.lf's
// `startup` reaction for current values). Split out from PulseMPC so a
// future live parameter estimator has a single, self-contained struct to
// update without touching MPC tuning or FSM state.
typedef struct {
  float V_min;              // rad/s, magnitude of the fixed pulse speed
  float gain;                // unitless 0..1, actual/commanded ratio
  float T_start;             // s, startup command delay
  float T_stop;              // s, hard-stop settle time
  float minimumPulseWidth;   // s, hysteresis guard before STOP eligible
} MotorModel;

// Per-motor controller state and tuning. Each Small_Velocity_Controller
// bank instance must hold its own PulseMPC as a reactor `state` field
// (never as a preamble/global variable) -- LF preamble variables compile to
// a single file-scope C global shared across every bank instance, which
// would let all 7 motors clobber the same controller.
typedef struct {
  MotorModel model;

  // ---- MPC tuning ----
  // These three weights all scale terms in the same J() cost comparison
  // (see PulseMPC_EvaluateCost), so their effects are relative to each
  // other, not absolute -- doubling all three together changes nothing.
  float W_debt;   // Weight on |predicted debt| in every action's cost (the
                   // "how much do we care about tracking error" knob).
                   // Higher: the controller chases debt more aggressively --
                   // starts sooner, keeps running longer, tracks the average
                   // commanded velocity more tightly. Lower: debt is
                   // tolerated more readily, so W_switch/W_pulse dominate
                   // the decision instead -- fewer pulses, looser tracking,
                   // but less switching/wear.
  float W_switch; // Fixed penalty added to both ACTION_START and ACTION_STOP
                   // (a flat "cost of switching" charge, representing wear/
                   // cost of toggling the motor on or off).
                   // Higher: the controller is more reluctant to start or
                   // stop at all -- fewer, longer pulses, less chatter, but
                   // slower to react (more debt tolerated between
                   // transitions). Lower: transitions become "free"-er, so
                   // the controller starts/stops more readily -- tighter
                   // tracking at the cost of more frequent switching.
  float W_pulse;  // Weight on pulseTimer in ACTION_CONTINUE's cost -- grows
                   // linearly the longer the current pulse has been running.
                   // Higher: long-running pulses get increasingly expensive,
                   // biasing toward stopping sooner rather than running
                   // indefinitely -- shorter pulses, more frequent restarts,
                   // guards against one continuous run overshooting or
                   // running longer than intended. Lower: pulses can run
                   // longer once started (decision depends mostly on debt),
                   // good for paying down a large debt in one go but risks
                   // longer-than-necessary runs if nothing else stops it.
  float H;                   // decision lookahead horizon (s), independent of dt
  float filterAlpha;         // weight on the newest sample in the pass-through/
                              // small-velocity mode filter (0..1) -- smaller means
                              // more smoothing/slower to react, larger means less
                              // smoothing/faster to react. See PulseMPC_Update.
  float debtDeadband;        // rad/s, magnitude below which desiredVelocity does not
                              // contribute to motionDebt at all (treated as exactly 0
                              // for that purpose only) -- keeps command noise/jitter
                              // near zero from slowly accumulating into a debt that
                              // eventually triggers a spurious pulse. Should stay small
                              // relative to V_min. See PulseMPC_Update.

  // ---- Controller state ----
  MotorState state;
  float dir;                 // +1.0/-1.0, latched at STOPPED->START, held until back to STOPPED
  float stateTimer;          // s, elapsed since entering STARTING/STOPPING
  float pulseTimer;          // s, elapsed since entering RUNNING
  float motionDebt;          // rad
  float commandVelocity;     // rad/s, signed, last commanded output
  float filteredVelocityMagnitude; // rad/s, EMA of |desiredVelocity|, used to decide
                                    // whether small-velocity (bang-bang) behavior
                                    // should be engaged -- see PulseMPC_Update.
  float filteredVelocity;          // rad/s, signed EMA of desiredVelocity (same
                                    // filterAlpha), used only to detect a debounced
                                    // direction reversal while RUNNING -- see
                                    // PulseMPC_Update.
} PulseMPC;

// Snapshot of a single controller's status, for diagnostics only (see
// PRINT_SVC above). Deliberately separate from PulseMPC itself -- this is
// what Small_Velocity_Controller_Bank.lf's print reaction reads over the
// `debug_info` output port, since a container reactor can only see a
// contained reactor's ports, not its internal state.
typedef struct {
  MotorState state;
  int passThrough;                 // 1 if commands are passed straight through
                                    // (normal speed), 0 if small-velocity
                                    // bang-bang mode is engaged
  float dir;
  float desiredVelocity;           // rad/s, upstream setpoint
  float commandVelocity;           // rad/s, actually commanded value
  float motionDebt;                // rad
  float filteredVelocityMagnitude; // rad/s, EMA used for the mode decision
  float filteredVelocity;          // rad/s, signed EMA used for the direction-reversal check
} SVCDebugInfo;

void PulseMPC_GetDebugInfo(const PulseMPC *c, float desiredVelocity, SVCDebugInfo *out);

// Prints one line of status for a single controller (milli-units, scaled to
// int, to avoid floating-point printf on this embedded target -- matches
// the convention used elsewhere in this codebase, e.g. USM.lf).
void PulseMPC_PrintDebugInfo(int index, SVCDebugInfo info);

float sign_f(float x);

// J() cost of each candidate action. desiredVelocity is only used by
// ACTION_START, to evaluate the direction it would actually commit to
// (sign_f(desiredVelocity)) rather than the possibly stale/wrong-signed
// c->dir left over from a previous cycle -- see the ACTION_START case in
// pulse_mpc.c for why this matters (a stale dir can otherwise make START
// look permanently worse than IDLE, deadlocking the controller at rest).
float PulseMPC_EvaluateCost(const PulseMPC *c, MpcAction action, float desiredVelocity);

void PulseMPC_ApplyAction(PulseMPC *c, MpcAction action, float desiredVelocity);

// Validates (and corrects, with a printed warning) MPC tuning weights that
// would otherwise make PulseMPC_Update structurally unable to ever start
// moving, no matter how large motionDebt grows. Call once after setting
// model.gain/model.V_min/model.T_start, H, W_debt, and W_switch (e.g. from
// startup), before first use.
//
// The controlling inequality (see ACTION_START in PulseMPC_EvaluateCost):
// once |motionDebt| is large, START only ever beats IDLE if
//   W_switch < W_debt * gain * V_min * (H - T_start)
// If this doesn't hold, START is permanently more expensive than IDLE no
// matter how large debt grows -- the motor just sits still forever while
// debt accumulates without bound (this is exactly the "steady-state error
// never goes to zero" failure mode found via hardware testing). If
// W_switch violates this, it's capped to 95% of the right-hand side and a
// warning is printed.
//
// W_debt is the inequality's other free variable, so it gets the same
// treatment: W_debt <= 0 collapses the right-hand side to <= 0, causing the
// identical "START can never win" failure via a different route. If W_debt
// is <= 0, it's clamped up to PULSE_MPC_MIN_W_DEBT and a warning is
// printed.
//
// W_pulse is NOT covered here: it only appears in ACTION_CONTINUE's cost,
// never in ACTION_START's, so it cannot structurally block a START
// decision the way W_switch/W_debt can. A very large W_pulse can still
// cause every pulse to terminate right at minimumPulseWidth (short,
// frequent, chattery pulses) rather than never happening at all -- a
// tuning concern worth being aware of, but not the same class of permanent
// deadlock, so there's no formulaic cap to derive/enforce for it here.
void PulseMPC_ValidateWeights(PulseMPC *c);

// Hybrid pass-through / small-velocity (bang-bang) update, run every
// control_period tick. dt is real elapsed time in seconds since the
// previous call -- stateTimer/pulseTimer are stored in seconds (not tick
// counts) so retuning control_period or characterizing new T_start/T_stop
// values never requires a code change.
//
// Every call updates a slow EMA filter of |desiredVelocity|
// (filteredVelocityMagnitude). Only when the controller is at rest
// (MOTOR_STOPPED) does that filtered estimate get checked against V_min:
// at/above V_min, ordinary continuous velocity commands are trusted and
// passed straight through (commandVelocity = desiredVelocity, motionDebt
// held at 0 so no stale debt carries into the next small-velocity episode);
// below V_min, the normal small-velocity FCS-MPC debt accumulation and
// action selection engages. Deciding only while MOTOR_STOPPED means an
// in-progress pulse always finishes cleanly before mode can change, and
// filtering (rather than a hysteresis band) means a desiredVelocity
// hovering near V_min doesn't flip modes tick-to-tick -- how much
// smoothing is controlled by filterAlpha.
//
// While RUNNING, two additional debounced early-exits force an immediate
// STOP (bypassing minimumPulseWidth) ahead of the normal CONTINUE/STOP cost
// comparison: (1) the filtered command has dropped near zero, or (2) it has
// reversed sign relative to the currently-latched dir. Without these, a
// wrong-direction/stale pulse only gets corrected once accumulating debt
// tips the cost comparison in STOP's favor, which can take a while.
//
// desiredVelocity magnitudes below debtDeadband do not contribute to
// motionDebt at all (see the deadband note on that field) -- this only
// affects debt accumulation, not filteredVelocity/filteredVelocityMagnitude
// or anything else desiredVelocity feeds into.
void PulseMPC_Update(PulseMPC *c, float desiredVelocity, float dt);

#endif // PULSE_MPC_H
