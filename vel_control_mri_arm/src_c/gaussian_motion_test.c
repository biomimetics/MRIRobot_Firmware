// Standalone sanity check + demo for gaussian_motion.c -- pure C, no LF
// reactor, no cmake integration. Not part of the firmware build (nothing
// includes or compiles this file automatically); compile and run it
// directly:
//
//   cd vel_control_mri_arm/src_c
//   gcc -o /tmp/gaussian_motion_test gaussian_motion_test.c gaussian_motion.c -lm
//   /tmp/gaussian_motion_test
#include "gaussian_motion.h"
#include <math.h>
#include <stdio.h>

int main(void) {
  int ok = 1;

  GaussianRV velocity = {.mean = 5.0f, .variance = 0.4f * 0.4f};

  // Add/Scale: exact linear propagation, checked directly against the
  // textbook formulas rather than just re-running the implementation.
  {
    GaussianRV a = {.mean = 2.0f, .variance = 1.0f};
    GaussianRV b = {.mean = 3.0f, .variance = 4.0f};
    GaussianRV sum = GaussianMotion_Add(a, b);
    if (fabsf(sum.mean - 5.0f) > 1e-6f || fabsf(sum.variance - 5.0f) > 1e-6f) {
      printf("FAIL: Add gave mean=%.6f variance=%.6f, expected 5.0/5.0\n", (double) sum.mean, (double) sum.variance);
      ok = 0;
    }

    GaussianRV scaled = GaussianMotion_Scale(a, 3.0f);
    if (fabsf(scaled.mean - 6.0f) > 1e-6f || fabsf(scaled.variance - 9.0f) > 1e-6f) {
      printf("FAIL: Scale gave mean=%.6f variance=%.6f, expected 6.0/9.0\n", (double) scaled.mean, (double) scaled.variance);
      ok = 0;
    }
  }

  // Second-order reciprocal's mean-bias correction should push mean(1/V)
  // above the first-order estimate for mean(V) > 0, variance(V) > 0.
  {
    GaussianRV linearized = GaussianMotion_ReciprocalLinearized(velocity);
    GaussianRV second_order = GaussianMotion_ReciprocalSecondOrder(velocity);
    if (!(second_order.mean > linearized.mean)) {
      printf("FAIL: expected second-order reciprocal mean (%.6f) > linearized (%.6f)\n",
          (double) second_order.mean, (double) linearized.mean);
      ok = 0;
    }
  }

  // ArrivalTimePdf should integrate to ~1 over a wide enough window --
  // coarse Riemann sum, so kept loose (within 5%).
  {
    float distance = 20.0f;
    float total = 0.0f;
    float step = 0.01f;
    for (float t = step; t < 30.0f; t += step) {
      total += GaussianMotion_ArrivalTimePdf(t, distance, velocity) * step;
    }
    if (fabsf(total - 1.0f) > 0.05f) {
      printf("FAIL: ArrivalTimePdf integrated to %.4f, expected ~1.0\n", (double) total);
      ok = 0;
    }
  }

  // Cdf/Cdf_Tail/InvCdf/InvCdf_Tail should all agree with each other: the
  // CDF at the mean is 0.5, Cdf+Cdf_Tail sum to 1, and Cdf(InvCdf(p)) round-
  // trips back to p. Tolerances are looser than the double version of this
  // test would use -- float epsilon, not the ~1e-9 the underlying rational
  // approximation could achieve in exact arithmetic.
  {
    if (fabsf(GaussianMotion_Cdf(velocity.mean, velocity) - 0.5f) > 1e-6f) {
      printf("FAIL: Cdf at mean expected 0.5, got %.9f\n", (double) GaussianMotion_Cdf(velocity.mean, velocity));
      ok = 0;
    }

    float x = 5.3f;
    float cdf_sum = GaussianMotion_Cdf(x, velocity) + GaussianMotion_Cdf_Tail(x, velocity);
    if (fabsf(cdf_sum - 1.0f) > 1e-6f) {
      printf("FAIL: Cdf(x) + Cdf_Tail(x) = %.9f, expected 1.0\n", (double) cdf_sum);
      ok = 0;
    }

    float p = 0.9f;
    float quantile = GaussianMotion_InvCdf(p, velocity);
    float roundtrip = GaussianMotion_Cdf(quantile, velocity);
    if (fabsf(roundtrip - p) > 1e-4f) {
      printf("FAIL: Cdf(InvCdf(%.2f)) = %.9f, expected %.2f\n", (double) p, (double) roundtrip, (double) p);
      ok = 0;
    }

    float tail_quantile = GaussianMotion_InvCdf_Tail(p, velocity);
    float tail_roundtrip = GaussianMotion_Cdf_Tail(tail_quantile, velocity);
    if (fabsf(tail_roundtrip - p) > 1e-4f) {
      printf("FAIL: Cdf_Tail(InvCdf_Tail(%.2f)) = %.9f, expected %.2f\n", (double) p, (double) tail_roundtrip, (double) p);
      ok = 0;
    }
  }

  float x0 = 0.0f;
  float t = 3.0f;
  float distance = 20.0f;

  GaussianRV position = GaussianMotion_PositionDistribution(x0, t, velocity);
  GaussianRV arrival1 = GaussianMotion_ArrivalTimeLinearized(distance, velocity);
  GaussianRV arrival2 = GaussianMotion_ArrivalTimeSecondOrder(distance, velocity);

  GaussianMotion_Print("Velocity", velocity);
  GaussianMotion_Print("Position", position);
  GaussianMotion_Print("Arrival (Linearized)", arrival1);
  GaussianMotion_Print("Arrival (Second Order)", arrival2);

  printf("Arrival PDF at t = 4.0 s : %.10f\n",
      (double) GaussianMotion_ArrivalTimePdf(4.0f, distance, velocity));

  printf(ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
