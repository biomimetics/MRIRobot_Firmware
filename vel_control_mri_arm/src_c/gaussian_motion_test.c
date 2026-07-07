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
#include <stddef.h>
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

  // ArrivalTimeCdf should agree with a Riemann sum of ArrivalTimePdf, and
  // should stay well-behaved (monotonic, bounded in [0, 1]) even when
  // velocity.mean sits at zero -- the regime the removed reciprocal-moment
  // approximations couldn't handle.
  {
    float distance = 20.0f;
    float pdf_integral = 0.0f;
    float step = 0.01f;
    for (float t = step; t <= 4.0f; t += step) {
      pdf_integral += GaussianMotion_ArrivalTimePdf(t, distance, velocity) * step;
    }
    float cdf_at_4 = GaussianMotion_ArrivalTimeCdf(4.0f, distance, velocity);
    if (fabsf(cdf_at_4 - pdf_integral) > 0.01f) {
      printf("FAIL: ArrivalTimeCdf(4.0) = %.6f, PDF integral to 4.0 = %.6f\n",
          (double) cdf_at_4, (double) pdf_integral);
      ok = 0;
    }

    if (GaussianMotion_ArrivalTimeCdf(-1.0f, distance, velocity) != 0.0f) {
      printf("FAIL: ArrivalTimeCdf should be 0 for arrival_time <= 0\n");
      ok = 0;
    }

    float prev = 0.0f;
    int monotonic = 1;
    for (float t = 0.5f; t <= 20.0f; t += 0.5f) {
      float cur = GaussianMotion_ArrivalTimeCdf(t, distance, velocity);
      if (cur < prev - 1e-6f) {
        monotonic = 0;
      }
      prev = cur;
    }
    if (!monotonic) {
      printf("FAIL: ArrivalTimeCdf is not monotonic in arrival_time\n");
      ok = 0;
    }

    GaussianRV zero_mean_velocity = {.mean = 0.0f, .variance = 0.4f * 0.4f};
    float cdf_zero_mean = GaussianMotion_ArrivalTimeCdf(3.0f, distance, zero_mean_velocity);
    if (!(cdf_zero_mean >= 0.0f && cdf_zero_mean <= 1.0f)) {
      printf("FAIL: ArrivalTimeCdf with zero-mean velocity gave out-of-range %.6f\n",
          (double) cdf_zero_mean);
      ok = 0;
    }
  }

  // ArrivalTimeCdf_Tail should be the complement of ArrivalTimeCdf at every
  // arrival_time (including <= 0, where Cdf is 0 and Cdf_Tail is 1), and
  // should stay well-behaved for near-zero-mean velocity same as above.
  {
    float distance = 20.0f;
    float sample_times[] = {-1.0f, 0.5f, 4.0f, 10.0f, 20.0f};
    for (size_t i = 0; i < sizeof(sample_times) / sizeof(sample_times[0]); i++) {
      float t = sample_times[i];
      float sum = GaussianMotion_ArrivalTimeCdf(t, distance, velocity) +
                  GaussianMotion_ArrivalTimeCdf_Tail(t, distance, velocity);
      if (fabsf(sum - 1.0f) > 1e-6f) {
        printf("FAIL: ArrivalTimeCdf(%.2f) + ArrivalTimeCdf_Tail(%.2f) = %.9f, expected 1.0\n",
            (double) t, (double) t, (double) sum);
        ok = 0;
      }
    }

    GaussianRV zero_mean_velocity = {.mean = 0.0f, .variance = 0.4f * 0.4f};
    float tail_zero_mean = GaussianMotion_ArrivalTimeCdf_Tail(3.0f, distance, zero_mean_velocity);
    if (!(tail_zero_mean >= 0.0f && tail_zero_mean <= 1.0f)) {
      printf("FAIL: ArrivalTimeCdf_Tail with zero-mean velocity gave out-of-range %.6f\n",
          (double) tail_zero_mean);
      ok = 0;
    }
  }

  // ArrivalTimeInvCdf/InvCdf_Tail should round-trip through
  // ArrivalTimeCdf/Cdf_Tail, same shape of check as the plain Cdf/InvCdf
  // round-trip above -- including for near-zero-mean velocity, where they
  // should stay finite and satisfy the round-trip instead of blowing up.
  {
    float distance = 20.0f;
    float p = 0.05f;

    float deadline = GaussianMotion_ArrivalTimeInvCdf(p, distance, velocity);
    float roundtrip = GaussianMotion_ArrivalTimeCdf(deadline, distance, velocity);
    if (fabsf(roundtrip - p) > 1e-3f) {
      printf("FAIL: ArrivalTimeCdf(ArrivalTimeInvCdf(%.2f)) = %.9f, expected %.2f\n",
          (double) p, (double) roundtrip, (double) p);
      ok = 0;
    }

    float tail_deadline = GaussianMotion_ArrivalTimeInvCdf_Tail(p, distance, velocity);
    float tail_roundtrip = GaussianMotion_ArrivalTimeCdf_Tail(tail_deadline, distance, velocity);
    if (fabsf(tail_roundtrip - p) > 1e-3f) {
      printf("FAIL: ArrivalTimeCdf_Tail(ArrivalTimeInvCdf_Tail(%.2f)) = %.9f, expected %.2f\n",
          (double) p, (double) tail_roundtrip, (double) p);
      ok = 0;
    }

    // Zero-mean velocity: P(V >= 0) = 0.5, so any p <= 0.5 asks for a
    // velocity threshold >= 0 -- still finite. p > 0.5 asks for a threshold
    // < 0, which should come back as +INFINITY (see the header comment).
    GaussianRV zero_mean_velocity = {.mean = 0.0f, .variance = 0.4f * 0.4f};
    float finite_deadline = GaussianMotion_ArrivalTimeInvCdf(0.4f, distance, zero_mean_velocity);
    if (!(isfinite(finite_deadline) && finite_deadline > 0.0f)) {
      printf("FAIL: ArrivalTimeInvCdf(0.4) with zero-mean velocity expected finite positive, got %.6f\n",
          (double) finite_deadline);
      ok = 0;
    }

    float infinite_deadline = GaussianMotion_ArrivalTimeInvCdf(0.6f, distance, zero_mean_velocity);
    if (infinite_deadline != INFINITY) {
      printf("FAIL: ArrivalTimeInvCdf(0.6) with zero-mean velocity expected +INFINITY, got %.6f\n",
          (double) infinite_deadline);
      ok = 0;
    }
  }

  float x0 = 0.0f;
  float t = 3.0f;
  float distance = 20.0f;

  GaussianRV position = GaussianMotion_PositionDistribution(x0, t, velocity);

  GaussianMotion_Print("Velocity", velocity);
  GaussianMotion_Print("Position", position);

  printf("Arrival PDF at t = 4.0 s : %.10f\n",
      (double) GaussianMotion_ArrivalTimePdf(4.0f, distance, velocity));
  printf("Arrival CDF at t = 4.0 s : %.10f\n",
      (double) GaussianMotion_ArrivalTimeCdf(4.0f, distance, velocity));
  printf("Arrival CDF Tail at t = 4.0 s : %.10f\n",
      (double) GaussianMotion_ArrivalTimeCdf_Tail(4.0f, distance, velocity));

  printf(ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
