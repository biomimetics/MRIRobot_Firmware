// Standalone sanity check for dpos_pulse_mpc.c -- pure C, no LF reactor, no
// cmake integration. Not part of the firmware build (nothing includes or
// compiles this file automatically); compile and run it directly:
//
//   cd vel_control_mri_arm/src_c
//   gcc -o /tmp/dpos_test dpos_pulse_mpc_test.c dpos_pulse_mpc.c pulse_motor_model.c -lm
//   /tmp/dpos_test
//
// Hand-sets a PulseMotorModel + DposPulseMPC and checks PlanBestPulse's chosen
// direction/run_duration against a few fixed remainingError values, before
// any reactor (Small_DeltaP_MPC_Controller.lf) is involved -- see
// src/lib/SmallDeltaPMPCControl/TODO.md's Stage 1 checklist.
#include "dpos_pulse_mpc.h"
#include <stdio.h>
#include <math.h>

static DposPulseMPC MakeController(void) {
  DposPulseMPC c = {0};
  c.model.V_min_cmd = 0.2f;
  c.model.gain = 0.9f;
  c.model.T_start = 0.025f;
  c.model.T_stop = 0.001f;
  c.model.minimumPulseWidth = 0.02f;

  c.W_error = 1.0f;
  c.k_overshoot = 10.0f;
  c.C_overshoot = 0.01f;
  c.sigma0 = 0.01f; // rad
  c.lookahead_depth = 2;
  c.grid_step_sigmas = 1.0f;
  c.filterAlpha = 0.1f;
  c.engagementGapThreshold = 0.3f;

  c.pulseState.state = MOTOR_STOPPED;
  c.pulseState.dir = 1.0f;
  return c;
}

static void CheckCase(const char *label, float remainingError) {
  DposPulseMPC c = MakeController();
  PulseCommand best = DposPulseMPC_PlanBestPulse(&c, remainingError, c.lookahead_depth);
  float predicted = PulseMotorModel_PredictDelta(&c.model, best);

  printf(
      "%-28s remErr=%+7.4f -> dir=%+.0f run_duration=%7.4fs predicted_delta=%+7.4f\n",
      label, (double) remainingError, (double) best.dir, (double) best.run_duration,
      (double) predicted);
}

int main(void) {
  printf("---- DposPulseMPC_PlanBestPulse sanity check ----\n");

  CheckCase("small positive gap", 0.02f);
  CheckCase("small negative gap", -0.02f);
  CheckCase("larger positive gap", 0.15f);
  CheckCase("larger negative gap", -0.15f);
  CheckCase("tiny gap (near floor)", 0.002f);

  // Direction sanity: positive remainingError should always plan dir > 0,
  // negative should always plan dir < 0.
  int ok = 1;
  {
    DposPulseMPC c = MakeController();
    PulseCommand p = DposPulseMPC_PlanBestPulse(&c, 0.05f, c.lookahead_depth);
    if (p.dir <= 0.0f) {
      printf("FAIL: expected positive dir for positive remainingError, got %+.0f\n", (double) p.dir);
      ok = 0;
    }
  }
  {
    DposPulseMPC c = MakeController();
    PulseCommand p = DposPulseMPC_PlanBestPulse(&c, -0.05f, c.lookahead_depth);
    if (p.dir >= 0.0f) {
      printf("FAIL: expected negative dir for negative remainingError, got %+.0f\n", (double) p.dir);
      ok = 0;
    }
  }
  // run_duration sanity: never below the motor's minimumPulseWidth floor.
  {
    DposPulseMPC c = MakeController();
    PulseCommand p = DposPulseMPC_PlanBestPulse(&c, 0.05f, c.lookahead_depth);
    if (p.run_duration < c.model.minimumPulseWidth) {
      printf("FAIL: run_duration %f below minimumPulseWidth %f\n",
          (double) p.run_duration, (double) c.model.minimumPulseWidth);
      ok = 0;
    }
  }

  // DposPulseMPC_Update sanity: a small remaining error with near-zero
  // desiredVelocity should engage small-velocity mode, plan and execute a
  // full STARTING -> RUNNING -> STOPPING -> STOPPED pulse cycle, and land
  // close to zero remaining error.
  {
    DposPulseMPC c = MakeController();
    float dt = 0.001f;
    float remainingError = 0.05f;
    float simulatedPosition = 0.0f;

    for (int i = 0; i < 5000; i++) {
      float gapLeft = remainingError - simulatedPosition;
      DposPulseMPC_Update(&c, 0.0f, gapLeft, dt);
      if (c.pulseState.state == MOTOR_RUNNING) {
        simulatedPosition += c.pulseState.commandVelocity * c.model.gain * dt;
      }
    }

    float finalGap = remainingError - simulatedPosition;
    printf("Update() full-cycle test: finalGap=%+7.4f (started at %+7.4f)\n",
        (double) finalGap, (double) remainingError);
    if (fabsf(finalGap) > 0.01f) {
      printf("FAIL: expected Update() to converge close to zero remaining error\n");
      ok = 0;
    }
  }

  printf(ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
