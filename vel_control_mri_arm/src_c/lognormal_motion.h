#ifndef LOGNORMAL_MOTION_H
#define LOGNORMAL_MOTION_H

#include "stats.h"

// EXPERIMENTAL, not wired into any controller yet -- see gaussian_motion.h
// for the Gaussian-velocity arrival-time model this is meant to be compared
// against, not (yet) replace.
//
// Alternative arrival-time model to gaussian_motion.h's: both remaining
// distance D and velocity V are modeled as independent, strictly-positive
// LogNormalRV's rather than distance being a fixed float and velocity being
// Gaussian. Because the ratio of two independent lognormals is itself
// exactly lognormal (LogNormalRV_Divide in stats.h), the resulting
// arrival-time distribution T = D/V is a proper distribution -- it
// integrates to exactly 1, unlike GaussianMotion_ArrivalTimeCdf's defective
// distribution (whose mass approaches P(V>=0) < 1 as t->infinity, see that
// file's comments) -- and has none of the near-zero-mean pathology of
// dividing by a Gaussian.
//
// The tradeoff: lognormal has no way to represent a velocity (or distance)
// whose mean is at or near zero, so this model is NOT a drop-in replacement
// for the small-velocity/pulsing regime gaussian_motion.c targets. It's
// meant for regimes where both distance and velocity are confidently
// bounded away from zero -- e.g. sustained continuous motion where the
// dominant uncertainty is measurement noise on the remaining distance
// (which itself tends to look lognormal: roughly constant *relative*
// error, so absolute variance shrinks as distance shrinks, rather than the
// fixed absolute variance a Gaussian distance model would imply).
//
// Uses float throughout, same rationale as gaussian_motion.h.

// T = D/V. Exact: ln(T) = ln(D) - ln(V) ~ Normal(mu_d - mu_v, sigma_d^2 +
// sigma_v^2), independent D and V.
LogNormalRV LogNormalMotion_ArrivalTimeDistribution(LogNormalRV distance, LogNormalRV velocity);

// Density of T = D/V at `arrival_time`.
float LogNormalMotion_ArrivalTimePdf(float arrival_time, LogNormalRV distance, LogNormalRV velocity);

// P(T <= arrival_time).
float LogNormalMotion_ArrivalTimeCdf(float arrival_time, LogNormalRV distance, LogNormalRV velocity);

// P(T >= arrival_time) (== 1 - ArrivalTimeCdf, computed directly for tail
// precision, same reasoning as GaussianRV_Cdf_Tail).
float LogNormalMotion_ArrivalTimeCdf_Tail(float arrival_time, LogNormalRV distance, LogNormalRV velocity);

// Inverse of ArrivalTimeCdf: returns arrival_time such that P(T <=
// arrival_time) = p. Same "overshoot deadline" interpretation as
// GaussianMotion_ArrivalTimeInvCdf. Domain 0 < p < 1.
float LogNormalMotion_ArrivalTimeInvCdf(float p, LogNormalRV distance, LogNormalRV velocity);

// Inverse of ArrivalTimeCdf_Tail: returns arrival_time such that P(T >=
// arrival_time) = p.
float LogNormalMotion_ArrivalTimeInvCdf_Tail(float p, LogNormalRV distance, LogNormalRV velocity);

// Prints mu/sigma2/mean/stddev of ln under `name`, for diagnostics.
void LogNormalMotion_Print(const char *name, LogNormalRV ln);

#endif // LOGNORMAL_MOTION_H
