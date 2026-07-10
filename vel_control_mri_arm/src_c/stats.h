#ifndef STATS_H
#define STATS_H

#include <stdbool.h>
#include <stdint.h>

/*==========================================================
 * Gaussian random variable
 *==========================================================*/

typedef struct
{
    float mean;
    float variance;
} GaussianRV;

// ---- Basic statistics ----

float GaussianRV_StdDev(GaussianRV g);

// Gaussian density of g, evaluated at x.
float GaussianRV_Pdf(float x, GaussianRV g);

// Standard normal (Phi^-1) quantile function, scaled/shifted to g: returns x
// such that P(X <= x) = p for X ~ g. Domain 0 < p < 1 (p == 0/1 map to
// +-INFINITY, outside [0,1] returns NAN). The underlying rational
// approximation (Acklam's) is accurate to ~1e-9 relative error in exact
// arithmetic, but computed here in float -- so actual accuracy is capped at
// float epsilon (~1e-7), not the ~1e-9 the coefficients could otherwise
// deliver.
float GaussianRV_InvCdf(float p, GaussianRV g);

// Same as InvCdf, but for the upper tail: returns x such that P(X >= x) = p
float GaussianRV_InvCdf_Tail(float p, GaussianRV g);

// P(X <= x) for X ~ g.
float GaussianRV_Cdf(float x, GaussianRV g);

// P(X >= x) for X ~ g (== 1 - Cdf(x, g), but computed directly for tail
// precision -- see the .c file).
float GaussianRV_Cdf_Tail(float x, GaussianRV g);

// ---- Generic Gaussian propagation (assumes independence throughout) ----

// Z = X + Y
GaussianRV GaussianRV_Add(GaussianRV a, GaussianRV b);

// Z = X - Y
GaussianRV GaussianRV_Subtract(GaussianRV a, GaussianRV b);

// Y = k*X
GaussianRV GaussianRV_Scale(GaussianRV x, float k);

// Z = X*Y. Mean and variance are both exact given independence --
// Var(XY) = Var(X)*Var(Y) + mean(X)^2*Var(Y) + mean(Y)^2*Var(X). Note the
// true distribution of a product of two Gaussians is not itself Gaussian --
// these are its first two moments, not a distributional claim.
GaussianRV GaussianRV_Product(GaussianRV a, GaussianRV b);


/*==========================================================
 * Lognormal distribution
 *
 * If X ~ LogNormal(mu, sigma²), then
 *
 *      ln(X) ~ Normal(mu, sigma²)
 *
 *==========================================================*/

typedef struct
{
    float mu;
    float sigma2;
} LogNormalRV;

// ---- Basic statistics ----

// Linear-space mean/variance of ln, via moment matching (same formulas as
// LogNormal_ToGaussian, exposed as plain accessors rather than requiring
// callers to build a throwaway GaussianRV).
float LogNormalRV_Mean(LogNormalRV ln);
float LogNormalRV_Variance(LogNormalRV ln);
float LogNormalRV_StdDev(LogNormalRV ln);

// Builds a LogNormalRV whose linear-space mean/stddev match the given
// values -- the inverse direction of LogNormalRV_Mean/StdDev. Lets a caller
// specify a lognormal prior in intuitive linear-space terms (e.g. "distance
// = 5cm, +-1mm") instead of computing mu/sigma2 by hand. mean must be > 0;
// stddev must be >= 0.
LogNormalRV LogNormalRV_FromMeanStdDev(float mean, float stddev);

// Lognormal density of ln, evaluated at x. 0 for x <= 0 (lognormal has no
// support there).
float LogNormalRV_Pdf(float x, LogNormalRV ln);

// P(X <= x) for X ~ ln. 0 for x <= 0.
float LogNormalRV_Cdf(float x, LogNormalRV ln);

// P(X >= x) for X ~ ln (== 1 - Cdf(x, ln), computed directly via the
// opposite-sign erfc tail for the same tail-precision reason as
// GaussianRV_Cdf_Tail). 1 for x <= 0.
float LogNormalRV_Cdf_Tail(float x, LogNormalRV ln);

// Quantile function: returns x such that P(X <= x) = p for X ~ ln. Domain
// 0 < p < 1 (p == 0 -> 0, p == 1 -> +INFINITY, outside [0,1] -> NAN), same
// convention as GaussianRV_InvCdf.
float LogNormalRV_InvCdf(float p, LogNormalRV ln);

// Same as InvCdf, but for the upper tail: returns x such that P(X >= x) = p.
float LogNormalRV_InvCdf_Tail(float p, LogNormalRV ln);

// ---- Generic lognormal propagation (assumes independence throughout) ----
//
// If X ~ LogNormal(mu_x, sigma_x^2) and Y ~ LogNormal(mu_y, sigma_y^2) are
// independent, then X*Y and X/Y are themselves exactly lognormal (their
// logs are the sum/difference of two independent normals) -- unlike
// GaussianRV_Product, these are exact distributional results, not just
// moment-matched approximations.

// Z = X*Y
LogNormalRV LogNormalRV_Multiply(LogNormalRV a, LogNormalRV b);

// Z = X/Y
LogNormalRV LogNormalRV_Divide(LogNormalRV a, LogNormalRV b);


/*==========================================================
 * Running sample statistics
 *==========================================================*/

typedef struct
{
    uint32_t n;      /* Number of samples */
    float mean;      /* Running mean */
    float M2;        /* Sum of squared deviations */
} SampleStats;


/*----------------------------------------------------------
 * Initialization
 *----------------------------------------------------------*/

void SampleStats_Init(SampleStats *s);


/*----------------------------------------------------------
 * Add one observation
 *----------------------------------------------------------*/

void SampleStats_Add(SampleStats *s, float x);


/*----------------------------------------------------------
 * Accessors
 *----------------------------------------------------------*/

uint32_t SampleStats_Count(const SampleStats *s);

float SampleStats_Mean(const SampleStats *s);

float SampleStats_SampleVariance(const SampleStats *s);

float SampleStats_PopulationVariance(const SampleStats *s);

float SampleStats_StandardDeviation(const SampleStats *s);


/*----------------------------------------------------------
 * Hypothesis-test-shaped helpers
 *----------------------------------------------------------*/

/*
 * One-sample z-test: is the sample mean statistically
 * significantly different from `reference`?
 *
 * True iff n >= min_count AND
 *
 *     |mean - reference| > z_threshold * stddev / sqrt(n)
 *
 * (i.e. reference lies outside the mean's z_threshold-sigma
 * confidence interval). z_threshold ~2.0 for the usual ~95%
 * level. A z-test rather than a proper t-test: at the
 * min_count values callers use (>= ~20) the two differ by a
 * few percent on the threshold, not worth carrying a
 * t-distribution table for.
 *
 * A zero-variance sample (all n observations identical) with
 * mean != reference tests as different -- consistent with the
 * formula (the confidence interval has zero width).
 */
bool SampleStats_MeanDiffersFrom(const SampleStats *s, float reference, float z_threshold, uint32_t min_count);

/*
 * True iff n >= min_count AND x lies more than
 * sigma_threshold sample standard deviations from the sample
 * mean. Below min_count this always returns false -- with too
 * few samples the mean/stddev aren't trustworthy enough to
 * reject anything against.
 */
bool SampleStats_IsOutlier(const SampleStats *s, float x, float sigma_threshold, uint32_t min_count);


/*==========================================================
 * Distribution conversions
 *==========================================================*/

/*
 * Treat the observed samples as Gaussian.
 */
GaussianRV SampleStats_ToGaussian(const SampleStats *s);

/*
 * Treat the observed samples as log(x).
 *
 * Feed SampleStats_Add(stats, logf(x))
 * to estimate these parameters.
 */
LogNormalRV SampleStats_ToLogNormal(const SampleStats *s);

/*
 * Moment-matched Gaussian approximation
 * of a lognormal distribution.
 */
GaussianRV LogNormal_ToGaussian(const LogNormalRV *ln);


/*==========================================================
 * Convenience helper
 *
 * Updates log-space statistics directly.
 *
 * Returns 0 if x <= 0.
 *==========================================================*/

int SampleStats_AddLogObservation(SampleStats *s, float x);

#endif // STATS_H
