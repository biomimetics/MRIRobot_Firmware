/*
 * stats.c
 *
 * Simple online statistics utilities for embedded systems.
 *
 * Uses Welford's online algorithm for numerically stable
 * running mean and variance estimation.
 */

#include "stats.h"
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/*==========================================================
 * Gaussian random variable -- basic statistics
 *
 * Moved from gaussian_motion.c: this is generic Gaussian
 * distribution/propagation math with no motion-specific meaning.
 * gaussian_motion.c keeps only the arrival-time/position-distribution
 * functions that actually interpret a GaussianRV as a velocity or position.
 *==========================================================*/

float GaussianRV_StdDev(GaussianRV g) {
  return sqrtf(g.variance);
}

static float StandardNormalPdf(float x, float mean, float stddev) {
  float z = (x - mean) / stddev;
  return expf(-0.5f * z * z) / (stddev * sqrtf(2.0f * (float) M_PI));
}

float GaussianRV_Pdf(float x, GaussianRV g) {
  return StandardNormalPdf(x, g.mean, GaussianRV_StdDev(g));
}

// Inverse CDF (quantile) of the standard normal distribution, via Acklam's
// rational approximation (~1e-9 relative accuracy over most of the domain
// in exact arithmetic -- see GaussianRV_InvCdf's comment for why this
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

float GaussianRV_InvCdf(float p, GaussianRV g) {
  return g.mean + sqrtf(g.variance) * StandardNormalInvCdf(p);
}

float GaussianRV_InvCdf_Tail(float p, GaussianRV g) {
  return g.mean + sqrtf(g.variance) * StandardNormalInvCdf(1.0f - p);
}

// Standard normal CDF via erfc rather than erf: erfc keeps precision in the
// far tails (where 1+erf(x) loses it to cancellation), which is exactly
// where Cdf/Cdf_Tail below matter most.
static float StandardNormalCdf(float z) {
  return 0.5f * erfcf(-z / sqrtf(2.0f));
}

// P(X <= x) for X ~ g.
float GaussianRV_Cdf(float x, GaussianRV g) {
  float z = (x - g.mean) / GaussianRV_StdDev(g);
  return StandardNormalCdf(z);
}

// P(X >= x) for X ~ g, i.e. 1 - Cdf(x, g) -- computed directly via the
// opposite-sign erfc tail rather than as a subtraction, for the same
// tail-precision reason as StandardNormalCdf itself.
float GaussianRV_Cdf_Tail(float x, GaussianRV g) {
  float z = (x - g.mean) / GaussianRV_StdDev(g);
  return StandardNormalCdf(-z);
}

GaussianRV GaussianRV_Add(GaussianRV a, GaussianRV b) {
  GaussianRV out;
  out.mean = a.mean + b.mean;
  out.variance = a.variance + b.variance;
  return out;
}

GaussianRV GaussianRV_Subtract(GaussianRV a, GaussianRV b) {
  GaussianRV out;
  out.mean = a.mean - b.mean;
  out.variance = a.variance + b.variance;
  return out;
}

GaussianRV GaussianRV_Scale(GaussianRV x, float k) {
  GaussianRV out;
  out.mean = k * x.mean;
  out.variance = k * k * x.variance;
  return out;
}

GaussianRV GaussianRV_Product(GaussianRV a, GaussianRV b) {
  GaussianRV out;
  out.mean = a.mean * b.mean;
  out.variance = a.mean * a.mean * b.variance
               + b.mean * b.mean * a.variance
               + a.variance * b.variance;
  return out;
}


/*----------------------------------------------------------
 * Initialization
 *----------------------------------------------------------*/

void SampleStats_Init(SampleStats *s)
{
    s->n = 0;
    s->mean = 0.0f;
    s->M2 = 0.0f;
}


/*----------------------------------------------------------
 * Add one observation
 *----------------------------------------------------------*/

void SampleStats_Add(SampleStats *s, float x)
{
    s->n++;

    float delta = x - s->mean;

    s->mean += delta / (float)s->n;

    float delta2 = x - s->mean;

    s->M2 += delta * delta2;
}


/*----------------------------------------------------------
 * Accessors
 *----------------------------------------------------------*/

uint32_t SampleStats_Count(const SampleStats *s)
{
    return s->n;
}

float SampleStats_Mean(const SampleStats *s)
{
    return s->mean;
}

float SampleStats_SampleVariance(const SampleStats *s)
{
    if (s->n < 2)
        return 0.0f;

    return s->M2 / (float)(s->n - 1);
}

float SampleStats_PopulationVariance(const SampleStats *s)
{
    if (s->n == 0)
        return 0.0f;

    return s->M2 / (float)s->n;
}

float SampleStats_StandardDeviation(const SampleStats *s)
{
    return sqrtf(SampleStats_SampleVariance(s));
}


/*----------------------------------------------------------
 * Hypothesis-test-shaped helpers (see stats.h)
 *----------------------------------------------------------*/

bool SampleStats_MeanDiffersFrom(const SampleStats *s, float reference, float z_threshold, uint32_t min_count)
{
    if (s->n < min_count)
        return false;

    float standard_error =
        SampleStats_StandardDeviation(s) / sqrtf((float)s->n);

    return fabsf(SampleStats_Mean(s) - reference) >
           z_threshold * standard_error;
}

bool SampleStats_IsOutlier(const SampleStats *s, float x, float sigma_threshold, uint32_t min_count)
{
    if (s->n < min_count)
        return false;

    return fabsf(x - SampleStats_Mean(s)) >
           sigma_threshold * SampleStats_StandardDeviation(s);
}


/*==========================================================
 * Distribution conversions
 *==========================================================*/

/*
 * Treat the observed samples as Gaussian.
 */
GaussianRV SampleStats_ToGaussian(const SampleStats *s)
{
    GaussianRV rv;

    rv.mean = SampleStats_Mean(s);
    rv.variance = SampleStats_SampleVariance(s);

    return rv;
}


/*
 * Treat the observed samples as log(x).
 *
 * Feed SampleStats_Add(stats, logf(x))
 * to estimate these parameters.
 */
LogNormalRV SampleStats_ToLogNormal(const SampleStats *s)
{
    LogNormalRV rv;

    rv.mu = SampleStats_Mean(s);
    rv.sigma2 = SampleStats_SampleVariance(s);

    return rv;
}


/*
 * Moment-matched Gaussian approximation
 * of a lognormal distribution.
 */
GaussianRV LogNormal_ToGaussian(const LogNormalRV *ln)
{
    GaussianRV rv;

    float mu = ln->mu;
    float sigma2 = ln->sigma2;

    rv.mean =
        expf(mu + 0.5f * sigma2);

    rv.variance =
        (expf(sigma2) - 1.0f) *
        expf(2.0f * mu + sigma2);

    return rv;
}


/*==========================================================
 * LogNormalRV -- basic statistics and algebra
 *==========================================================*/

float LogNormalRV_Mean(LogNormalRV ln) {
  return expf(ln.mu + 0.5f * ln.sigma2);
}

float LogNormalRV_Variance(LogNormalRV ln) {
  return (expf(ln.sigma2) - 1.0f) * expf(2.0f * ln.mu + ln.sigma2);
}

float LogNormalRV_StdDev(LogNormalRV ln) {
  return sqrtf(LogNormalRV_Variance(ln));
}

LogNormalRV LogNormalRV_FromMeanStdDev(float mean, float stddev) {
  LogNormalRV ln;
  float variance = stddev * stddev;
  ln.sigma2 = logf(1.0f + variance / (mean * mean));
  ln.mu = logf(mean) - 0.5f * ln.sigma2;
  return ln;
}

float LogNormalRV_Pdf(float x, LogNormalRV ln) {
  if (x <= 0.0f) {
    return 0.0f;
  }
  float sigma = sqrtf(ln.sigma2);
  float z = (logf(x) - ln.mu) / sigma;
  return expf(-0.5f * z * z) / (x * sigma * sqrtf(2.0f * (float) M_PI));
}

float LogNormalRV_Cdf(float x, LogNormalRV ln) {
  if (x <= 0.0f) {
    return 0.0f;
  }
  return StandardNormalCdf((logf(x) - ln.mu) / sqrtf(ln.sigma2));
}

float LogNormalRV_Cdf_Tail(float x, LogNormalRV ln) {
  if (x <= 0.0f) {
    return 1.0f;
  }
  return StandardNormalCdf(-(logf(x) - ln.mu) / sqrtf(ln.sigma2));
}

float LogNormalRV_InvCdf(float p, LogNormalRV ln) {
  return expf(ln.mu + sqrtf(ln.sigma2) * StandardNormalInvCdf(p));
}

float LogNormalRV_InvCdf_Tail(float p, LogNormalRV ln) {
  return expf(ln.mu + sqrtf(ln.sigma2) * StandardNormalInvCdf(1.0f - p));
}

LogNormalRV LogNormalRV_Multiply(LogNormalRV a, LogNormalRV b) {
  LogNormalRV out;
  out.mu = a.mu + b.mu;
  out.sigma2 = a.sigma2 + b.sigma2;
  return out;
}

LogNormalRV LogNormalRV_Divide(LogNormalRV a, LogNormalRV b) {
  LogNormalRV out;
  out.mu = a.mu - b.mu;
  out.sigma2 = a.sigma2 + b.sigma2;
  return out;
}


/*==========================================================
 * Convenience helper
 *
 * Updates log-space statistics directly.
 *
 * Returns 0 if x <= 0.
 *==========================================================*/

int SampleStats_AddLogObservation(SampleStats *s, float x)
{
    if (x <= 0.0f)
        return 0;

    SampleStats_Add(s, logf(x));

    return 1;
}


