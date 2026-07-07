#ifndef SMALL_DELTAP_PULSE_CONTROLLER_H
#define SMALL_DELTAP_PULSE_CONTROLLER_H

#include "pulse_motor_model.h"
#include <stdbool.h>

// Diagnostics-only snapshot of a Small_DeltaP_Pulse_Controller instance's
// status -- this design's analogue of DposPulseMPCDebugInfo
// (dpos_pulse_mpc.h)/SVCDebugInfo (pulse_mpc.h). Kept in its own tiny
// header (rather than folded into pulse_motor_model.h, which is
// deliberately controller-agnostic) so a test harness/CSV logger can
// reference this type without pulling in controller-specific planning
// logic -- there is no such logic to pull in anyway; it all lives inline
// in Small_DeltaP_Pulse_Controller.lf's own reactions (a reactor, not a
// separate .c/.h implementation), so this header holds only the data
// shape, not any behavior.

// Set to 1 to populate debug_info in Small_DeltaP_Pulse_Controller.lf's
// control_tick reaction -- 0 by default to avoid the overhead of copying
// this struct out every control_period tick during normal operation, same
// role as PRINT_DPOS/PRINT_SVC for the older controllers.
#define PRINT_DPULSE 0

typedef struct {
  MotorState state;
  bool passThrough;
  float remainingError;
  PulseCommand plannedPulse;
  float commandVelocity;
} SmallDeltaPPulseControllerDebugInfo;

#endif // SMALL_DELTAP_PULSE_CONTROLLER_H
