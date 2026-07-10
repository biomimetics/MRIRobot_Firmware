// Standalone sanity check + demo for lognormal_motion.c -- pure C, no LF
// reactor, no cmake integration. Not part of the firmware build. Compile
// and run directly:
//
//   cd vel_control_mri_arm/src_c
//   gcc -o /tmp/lognormal_motion_test lognormal_motion_test.c lognormal_motion.c stats.c -lm
//   /tmp/lognormal_motion_test
#include "lognormal_motion.h"
#include <math.h>
#include <stdio.h>

int main(void) {
  int ok = 1;

  // Distance: estimated 20.0, +-2.0 (10% relative stddev) -- the "small,
  // roughly constant relative error, shrinks near zero" regime discussed
  // for this model.
  LogNormalRV distance = LogNormalRV_FromMeanStdDev(20.0f, 2.0f);
  // Velocity: estimated 5.0, +-0.4 -- confidently away from zero, unlike
  // the small-pulse regime gaussian_motion.c has to handle.
  LogNormalRV velocity = LogNormalRV_FromMeanStdDev(5.0f, 0.4f);

  // LogNormalRV_FromMeanStdDev should round-trip through LogNormalRV_Mean/
  // StdDev.
  {
    if (fabsf(LogNormalRV_Mean(distance) - 20.0f) > 1e-3f ||
        fabsf(LogNormalRV_StdDev(distance) - 2.0f) > 1e-3f) {
      printf("FAIL: distance mean/stddev round-trip got mean=%.6f stddev=%.6f, expected 20.0/2.0\n",
          (double) LogNormalRV_Mean(distance), (double) LogNormalRV_StdDev(distance));
      ok = 0;
    }
  }

  // ArrivalTimePdf should integrate to ~1 -- unlike GaussianMotion's
  // reciprocal-Gaussian transform, this one is a proper (non-defective)
  // distribution, so this should hold tightly, not just approximately.
  {
    float total = 0.0f;
    float step = 0.001f;
    for (float t = step; t < 20.0f; t += step) {
      total += LogNormalMotion_ArrivalTimePdf(t, distance, velocity) * step;
    }
    if (fabsf(total - 1.0f) > 0.02f) {
      printf("FAIL: ArrivalTimePdf integrated to %.4f, expected ~1.0\n", (double) total);
      ok = 0;
    }
  }

  // Cdf + Cdf_Tail should sum to exactly 1 at every arrival_time (proper
  // distribution, no point mass at infinity the way GaussianMotion's does).
  {
    float sample_times[] = {0.5f, 2.0f, 4.0f, 10.0f, 50.0f};
    for (size_t i = 0; i < sizeof(sample_times) / sizeof(sample_times[0]); i++) {
      float t = sample_times[i];
      float sum = LogNormalMotion_ArrivalTimeCdf(t, distance, velocity) +
                  LogNormalMotion_ArrivalTimeCdf_Tail(t, distance, velocity);
      if (fabsf(sum - 1.0f) > 1e-5f) {
        printf("FAIL: ArrivalTimeCdf(%.2f) + ArrivalTimeCdf_Tail(%.2f) = %.9f, expected 1.0\n",
            (double) t, (double) t, (double) sum);
        ok = 0;
      }
    }

    // As arrival_time -> infinity, Cdf -> 1 (contrast with
    // GaussianMotion_ArrivalTimeCdf, which only approaches P(V>=0) < 1).
    float cdf_far = LogNormalMotion_ArrivalTimeCdf(1e6f, distance, velocity);
    if (fabsf(cdf_far - 1.0f) > 1e-4f) {
      printf("FAIL: ArrivalTimeCdf(1e6) = %.9f, expected ~1.0\n", (double) cdf_far);
      ok = 0;
    }
  }

  // InvCdf/InvCdf_Tail should round-trip through Cdf/Cdf_Tail.
  {
    float p = 0.05f;

    float deadline = LogNormalMotion_ArrivalTimeInvCdf(p, distance, velocity);
    float roundtrip = LogNormalMotion_ArrivalTimeCdf(deadline, distance, velocity);
    if (fabsf(roundtrip - p) > 1e-3f) {
      printf("FAIL: ArrivalTimeCdf(ArrivalTimeInvCdf(%.2f)) = %.9f, expected %.2f\n",
          (double) p, (double) roundtrip, (double) p);
      ok = 0;
    }

    float tail_deadline = LogNormalMotion_ArrivalTimeInvCdf_Tail(p, distance, velocity);
    float tail_roundtrip = LogNormalMotion_ArrivalTimeCdf_Tail(tail_deadline, distance, velocity);
    if (fabsf(tail_roundtrip - p) > 1e-3f) {
      printf("FAIL: ArrivalTimeCdf_Tail(ArrivalTimeInvCdf_Tail(%.2f)) = %.9f, expected %.2f\n",
          (double) p, (double) tail_roundtrip, (double) p);
      ok = 0;
    }
  }

  // LogNormalRV_Multiply/Divide sanity, checked directly against the
  // textbook log-space formulas rather than re-running the implementation.
  // (Note: (X/velocity)*velocity does NOT recover X here -- velocity is an
  // independent draw each time it's used, so variance accumulates on both
  // the divide and the multiply rather than canceling, same as it would
  // for any two independent random variables.)
  {
    LogNormalRV ratio = LogNormalRV_Divide(distance, velocity);
    float expected_mu = distance.mu - velocity.mu;
    float expected_sigma2 = distance.sigma2 + velocity.sigma2;
    if (fabsf(ratio.mu - expected_mu) > 1e-5f || fabsf(ratio.sigma2 - expected_sigma2) > 1e-5f) {
      printf("FAIL: Divide gave mu=%.6f sigma2=%.6f, expected %.6f/%.6f\n",
          (double) ratio.mu, (double) ratio.sigma2, (double) expected_mu, (double) expected_sigma2);
      ok = 0;
    }

    LogNormalRV product = LogNormalRV_Multiply(distance, velocity);
    float expected_mu_p = distance.mu + velocity.mu;
    if (fabsf(product.mu - expected_mu_p) > 1e-5f || fabsf(product.sigma2 - expected_sigma2) > 1e-5f) {
      printf("FAIL: Multiply gave mu=%.6f sigma2=%.6f, expected %.6f/%.6f\n",
          (double) product.mu, (double) product.sigma2, (double) expected_mu_p, (double) expected_sigma2);
      ok = 0;
    }
  }

  LogNormalRV arrival = LogNormalMotion_ArrivalTimeDistribution(distance, velocity);

  LogNormalMotion_Print("Distance", distance);
  LogNormalMotion_Print("Velocity", velocity);
  LogNormalMotion_Print("Arrival time (lognormal model)", arrival);

  printf("Arrival PDF at t = 4.0 s : %.10f\n",
      (double) LogNormalMotion_ArrivalTimePdf(4.0f, distance, velocity));
  printf("Arrival CDF at t = 4.0 s : %.10f\n",
      (double) LogNormalMotion_ArrivalTimeCdf(4.0f, distance, velocity));
  printf("Arrival CDF Tail at t = 4.0 s : %.10f\n",
      (double) LogNormalMotion_ArrivalTimeCdf_Tail(4.0f, distance, velocity));

  printf(ok ? "PASS\n" : "FAIL\n");
  return ok ? 0 : 1;
}
