#include "pulse_controller.h"
#include <stdio.h>

void PulseController_PrintDebugInfo(int index, PulseControllerDebugInfo info) {
  const char *state_str;
  switch (info.state) {
    case MOTOR_STOPPED:  state_str = "STOPPED";  break;
    case MOTOR_STARTING: state_str = "STARTING"; break;
    case MOTOR_RUNNING:  state_str = "RUNNING";  break;
    case MOTOR_STOPPING: state_str = "STOPPING"; break;
    default:              state_str = "?";        break;
  }
  // still-planning: actively pulsing toward remainingError, as opposed to
  // passthrough (following desired_velocity_ directly) or converged (close
  // enough that no further pulse will be planned) -- see converged's
  // comment on PulseControllerDebugInfo.
  const char *mode_str = info.passThrough ? "PASSTHRU" : (info.converged ? "CONVERGED" : "PLANNING");
  printf(
      "DPULSE[%d]: %-8s mode=%-9s remErr=%6d dir=%+d runDur=%6d cmd=%6d (milli-rad, milli-rad/s, milli-s)\r\n",
      index, state_str, mode_str,
      (int) (info.remainingError * 1000.0f), (int) info.plannedPulse.dir,
      (int) (info.plannedPulse.run_duration * 1000.0f),
      (int) (info.commandVelocity * 1000.0f));
}
