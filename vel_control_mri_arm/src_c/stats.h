#ifndef STATS_H
#define STATS_H

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
