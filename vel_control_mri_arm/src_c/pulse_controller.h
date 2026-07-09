#ifndef PULSE_CONTROLLER_H
#define PULSE_CONTROLLER_H

#include "pulse_motor_model.h"
#include <stdbool.h>

// Diagnostics-only snapshot of a PulseController instance's
// status -- this design's analogue of DposPulseMPCDebugInfo
// (dpos_pulse_mpc.h)/SVCDebugInfo (pulse_mpc.h). Kept in its own tiny
// header (rather than folded into pulse_motor_model.h, which is
// deliberately controller-agnostic) so a test harness/CSV logger can
// reference this type without pulling in controller-specific planning
// logic -- the planning logic itself still lives inline in
// PulseController.lf's own reactions (a reactor, not a
// separate .c/.h implementation); the one exception is
// PulseController_PrintDebugInfo below, which has to live in
// pulse_controller.c (compiled exactly once) rather than in a
// .lf preamble, since preamble code can get transcluded into the generated
// build more than once and duplicate-define it -- same reasoning as
// DposPulseMPC_PrintDebugInfo living in dpos_pulse_mpc.c instead of
// Small_DeltaP_MPC_Controller_Bank.lf's preamble.

// Set to 1 to populate debug_info in PulseController.lf's
// control_tick reaction -- 0 by default to avoid the overhead of copying
// this struct out every control_period tick during normal operation, same
// role as PRINT_DPOS/PRINT_SVC for the older controllers. On its own this
// only feeds debug_info (e.g. for CSV logging -- DeltaPPulseCsvLogger.lf);
// it does NOT print anything to the console by itself. See PRINT_DPULSE_DEBUG
// below for live serial printing on real hardware.
#define PRINT_DPULSE 0

// Set to 1 for live on-hardware debugging via PulseControllerBank.lf's
// throttled printf of every joint's state/passthrough/remaining
// error/planning status -- unlike PRINT_DPULSE above, this actually prints
// (see that Bank reactor's controllers.debug_info reaction). Independent of
// PRINT_DPULSE so you don't have to pull in the CSV-logging path just to
// watch state on a terminal. 0 by default, same reasoning as PRINT_DPULSE.
#define PRINT_DPULSE_DEBUG 0

typedef struct {
  MotorState state;
  bool passThrough;
  // true once |remainingError| has dropped at/below convergenceLimit_ and
  // this instance has given up on planning any further pulse for it (see
  // PulseController.lf's converged_ state) -- together with
  // passThrough, tells you whether this instance is still actively trying
  // to close remainingError via pulsing (passThrough == false && converged
  // == false) or has stopped for some other reason.
  bool converged;
  float remainingError;
  PulseCommand plannedPulse;
  float commandVelocity;
} PulseControllerDebugInfo;

// Prints one instance's debug_info to stdout -- see PRINT_DPULSE_DEBUG above
// and PulseControllerBank.lf's controllers.debug_info
// reaction, its only caller.
void PulseController_PrintDebugInfo(int index, PulseControllerDebugInfo info);

#endif // PULSE_CONTROLLER_H
