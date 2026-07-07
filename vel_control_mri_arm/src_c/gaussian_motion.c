#include "gaussian_motion.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

float GaussianMotion_StdDev(GaussianRV g) {
  return sqrtf(g.variance);
}

static float StandardNormalPdf(float x, float mean, float stddev) {
  float z = (x - mean) / stddev;
  return expf(-0.5f * z * z) / (stddev * sqrtf(2.0f * (float) M_PI));
}

float GaussianMotion_Pdf(float x, GaussianRV g) {
  return StandardNormalPdf(x, g.mean, GaussianMotion_StdDev(g));
}

// Inverse CDF (quantile) of the standard normal distribution, via Acklam's
// rational approximation (~1e-9 relative accuracy over most of the domain
// in exact arithmetic -- see GaussianMotion_InvCdf's comment for why this
// file's float arithmetic doesn't actually achieve that).
// -INFINITY for p == 0, INFINITY for p == 1, NAN outside [0, 1].
static float StandardNormalInvCdf(float p) {
  if (p <= 0.0f) {
    return (p == 0.0f) ? -INFINITY : NAN;
  }
  if (p >= 1.0f) {
    return (p == 1.0f) ? INFINITY : NAN;
  }

  static const float a[] = {
    -3.969683028665376e+01f, 2.209460984245205e+02f, -2.759285104469687e+02f,
     1.383577518672690e+02f, -3.066479806614716e+01f, 2.506628277459239e+00f
  };
  static const float b[] = {
    -5.447609879822406e+01f, 1.615858368580409e+02f, -1.556989798598866e+02f,
     6.680131188771972e+01f, -1.328068155288572e+01f
  };
  static const float c[] = {
    -7.784894002430293e-03f, -3.223964580411365e-01f, -2.400758277161838e+00f,
    -2.549732539343734e+00f,  4.374664141464968e+00f,  2.938163982698783e+00f
  };
  static const float d[] = {
     7.784695709041462e-03f, 3.224671290700398e-01f, 2.445134137142996e+00f,
     3.754408661907416e+00f
  };

  const float plow = 0.02425f;
  const float phigh = 1.0f - plow;
  float q, r;

  if (p < plow) {
    q = sqrtf(-2.0f * logf(p));
    return (((((c[0]*q + c[1])*q + c[2])*q + c[3])*q + c[4])*q + c[5]) /
           ((((d[0]*q + d[1])*q + d[2])*q + d[3])*q + 1.0f);
  }

  if (p > phigh) {
    q = sqrtf(-2.0f * logf(1.0f - p));
    return -(((((c[0]*q + c[1])*q + c[2])*q + c[3])*q + c[4])*q + c[5]) /
             ((((d[0]*q + d[1])*q + d[2])*q + d[3])*q + 1.0f);
  }

  q = p - 0.5f;
  r = q * q;
  return (((((a[0]*r + a[1])*r + a[2])*r + a[3])*r + a[4])*r + a[5]) * q /
         (((((b[0]*r + b[1])*r + b[2])*r + b[3])*r + b[4])*r + 1.0f);
}

float GaussianMotion_InvCdf(float p, GaussianRV g) {
  return g.mean + sqrtf(g.variance) * StandardNormalInvCdf(p);
}

float GaussianMotion_InvCdf_Tail(float p, GaussianRV g) {
  return g.mean + sqrtf(g.variance) * StandardNormalInvCdf(1.0f - p);
}

// Standard normal CDF via erfc rather than erf: erfc keeps precision in the
// far tails (where 1+erf(x) loses it to cancellation), which is exactly
// where Cdf/Cdf_Tail below matter most.
static float StandardNormalCdf(float z) {
  return 0.5f * erfcf(-z / sqrtf(2.0f));
}

// P(X <= x) for X ~ g.
float GaussianMotion_Cdf(float x, GaussianRV g) {
  float z = (x - g.mean) / GaussianMotion_StdDev(g);
  return StandardNormalCdf(z);
}

// P(X >= x) for X ~ g, i.e. 1 - Cdf(x, g) -- computed directly via the
// opposite-sign erfc tail rather than as a subtraction, for the same
// tail-precision reason as StandardNormalCdf itself.
float GaussianMotion_Cdf_Tail(float x, GaussianRV g) {
  float z = (x - g.mean) / GaussianMotion_StdDev(g);
  return StandardNormalCdf(-z);
}

GaussianRV GaussianMotion_Add(GaussianRV a, GaussianRV b) {
  GaussianRV out;
  out.mean = a.mean + b.mean;
  out.variance = a.variance + b.variance;
  return out;
}

GaussianRV GaussianMotion_Subtract(GaussianRV a, GaussianRV b) {
  GaussianRV out;
  out.mean = a.mean - b.mean;
  out.variance = a.variance + b.variance;
  return out;
}

GaussianRV GaussianMotion_Scale(GaussianRV x, float k) {
  GaussianRV out;
  out.mean = k * x.mean;
  out.variance = k * k * x.variance;
  return out;
}

GaussianRV GaussianMotion_Product(GaussianRV a, GaussianRV b) {
  GaussianRV out;
  out.mean = a.mean * b.mean;
  out.variance = a.mean * a.mean * b.variance
               + b.mean * b.mean * a.variance
               + a.variance * b.variance;
  return out;
}

GaussianRV GaussianMotion_PositionDistribution(float x0, float time, GaussianRV velocity) {
  GaussianRV out;
  out.mean = x0 + velocity.mean * time;
  out.variance = velocity.variance * time * time;
  return out;
}

float GaussianMotion_ArrivalTimePdf(float arrival_time, float distance, GaussianRV velocity) {
  if (arrival_time <= 0.0f) {
    return 0.0f;
  }
  float v = distance / arrival_time;
  return (distance / (arrival_time * arrival_time)) * GaussianMotion_Pdf(v, velocity);
}

float GaussianMotion_ArrivalTimeCdf(float arrival_time, float distance, GaussianRV velocity) {
  if (arrival_time <= 0.0f) {
    return 0.0f;
  }
  return GaussianMotion_Cdf_Tail(distance / arrival_time, velocity);
}

float GaussianMotion_ArrivalTimeCdf_Tail(float arrival_time, float distance, GaussianRV velocity) {
  if (arrival_time <= 0.0f) {
    return 1.0f;
  }
  return GaussianMotion_Cdf(distance / arrival_time, velocity);
}

float GaussianMotion_ArrivalTimeInvCdf(float p, float distance, GaussianRV velocity) {
  float v_threshold = GaussianMotion_InvCdf_Tail(p, velocity);
  if (v_threshold <= 0.0f) {
    return INFINITY;
  }
  return distance / v_threshold;
}

float GaussianMotion_ArrivalTimeInvCdf_Tail(float p, float distance, GaussianRV velocity) {
  float v_threshold = GaussianMotion_InvCdf(p, velocity);
  if (v_threshold <= 0.0f) {
    return INFINITY;
  }
  return distance / v_threshold;
}

void GaussianMotion_Print(const char *name, GaussianRV g) {
  printf("%s\n", name);
  printf("  mean     : %.6f\n", (double) g.mean);
  printf("  variance : %.6f\n", (double) g.variance);
  printf("  stddev   : %.6f\n\n", (double) GaussianMotion_StdDev(g));
}
