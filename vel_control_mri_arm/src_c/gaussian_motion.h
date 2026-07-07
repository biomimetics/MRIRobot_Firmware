#ifndef GAUSSIAN_MOTION_H
#define GAUSSIAN_MOTION_H

// 1-D Gaussian uncertainty propagation for a constant-velocity motion model:
// V ~ N(mean, variance), held constant over the interval of interest.
// Provides generic Gaussian random-variable algebra (add/subtract/scale/
// product/reciprocal) plus two motion-specific applications built on top of
// it: the position distribution after time t, and the arrival-time
// distribution/likelihood for covering a fixed distance at velocity V.
//
// Uses float rather than double throughout -- trades away some numerical
// accuracy (see GaussianMotion_InvCdf's comment for how much) for speed and
// for consistency with the rest of this codebase's control-loop math
// (MotorModel, PulseCommand, etc.), which is float end to end.
//
// NOTE: arrival-time-by-reciprocal-moments (mean(1/V), Var(1/V) via Taylor
// expansion) is deliberately not provided here. It implicitly assumes
// velocity.mean is bounded well away from zero relative to its stddev (a
// near-zero-mean reciprocal Gaussian has no finite mean/variance in the
// exact sense), which does NOT hold in this codebase's small-velocity/
// pulsing regimes (see dpos_pulse_mpc.c). ArrivalTimePdf/ArrivalTimeCdf
// below sidestep this: they're exact change-of-variables transforms of
// velocity's own density/CDF, valid for any velocity.mean (including near
// zero), as long as velocity is physically one-signed over the interval.

typedef struct {
  float mean;
  float variance;
} GaussianRV;

// ---- Basic statistics ----

float GaussianMotion_StdDev(GaussianRV g);

// Gaussian density of g, evaluated at x.
float GaussianMotion_Pdf(float x, GaussianRV g);

// Standard normal (Phi^-1) quantile function, scaled/shifted to g: returns x
// such that P(X <= x) = p for X ~ g. Domain 0 < p < 1 (p == 0/1 map to
// +-INFINITY, outside [0,1] returns NAN). The underlying rational
// approximation (Acklam's) is accurate to ~1e-9 relative error in exact
// arithmetic, but computed here in float -- so actual accuracy is capped at
// float epsilon (~1e-7), not the ~1e-9 the coefficients could otherwise
// deliver.
float GaussianMotion_InvCdf(float p, GaussianRV g);

// Same as InvCdf, but for the upper tail: returns x such that P(X >= x) = p
float GaussianMotion_InvCdf_Tail(float p, GaussianRV g);

// P(X <= x) for X ~ g.
float GaussianMotion_Cdf(float x, GaussianRV g);

// P(X >= x) for X ~ g (== 1 - Cdf(x, g), but computed directly for tail
// precision -- see the .c file).
float GaussianMotion_Cdf_Tail(float x, GaussianRV g);

// ---- Generic Gaussian propagation (assumes independence throughout) ----

// Z = X + Y
GaussianRV GaussianMotion_Add(GaussianRV a, GaussianRV b);

// Z = X - Y
GaussianRV GaussianMotion_Subtract(GaussianRV a, GaussianRV b);

// Y = k*X
GaussianRV GaussianMotion_Scale(GaussianRV x, float k);

// Z = X*Y. Mean and variance are both exact given independence --
// Var(XY) = Var(X)*Var(Y) + mean(X)^2*Var(Y) + mean(Y)^2*Var(X). Note the
// true distribution of a product of two Gaussians is not itself Gaussian --
// these are its first two moments, not a distributional claim.
GaussianRV GaussianMotion_Product(GaussianRV a, GaussianRV b);

// ---- Motion models: constant velocity V ~ N(mean, variance) over [0, t] ----

// Position after time `time`, starting from x0: X = x0 + V*time.
GaussianRV GaussianMotion_PositionDistribution(float x0, float time, GaussianRV velocity);

// Density of T = distance/V at `arrival_time`: f_T(arrival_time), i.e. the
// PDF of arrival time, not a cumulative probability. Computed exactly via
// change-of-variables from velocity's own Gaussian density: f_T(t) =
// (distance/t^2) * f_V(distance/t). Only the positive-t/positive-v branch
// of the transform: correct as long as distance > 0 and velocity is
// physically one-signed over the interval. Returns 0 for arrival_time <= 0.
float GaussianMotion_ArrivalTimePdf(float arrival_time, float distance, GaussianRV velocity);

// P(T <= arrival_time) for T = distance/V -- probability that arrival
// happens at or before `arrival_time` (cumulative mass on [0, arrival_time]
// in T-space, i.e. the "arrived in time" probability). Computed as
// P(V >= distance/arrival_time) -- the equivalent flipped-tail condition in
// V-space, via velocity's own Cdf_Tail -- rather than any moment
// approximation of T, so unlike a Gaussian fit to T's mean/variance, this
// stays valid even when velocity.mean sits near zero (the small-velocity/
// pulsing regime -- see the file-level NOTE). Same one-signed-velocity/
// positive-distance caveat as ArrivalTimePdf. Returns 0 for
// arrival_time <= 0.
float GaussianMotion_ArrivalTimeCdf(float arrival_time, float distance, GaussianRV velocity);

// P(T >= arrival_time) for T = distance/V -- probability that arrival has
// NOT yet happened by `arrival_time` (cumulative mass on [arrival_time,
// infinity) in T-space, i.e. the "still in transit"/"hasn't arrived yet"
// probability). Equivalent to 1 - ArrivalTimeCdf(...), but computed
// directly as P(V <= distance/arrival_time) via velocity's own (non-tail)
// Cdf, for the same tail-precision reason Cdf/Cdf_Tail are both provided
// above. Returns 1 for arrival_time <= 0 (arrival cannot have happened
// yet).
float GaussianMotion_ArrivalTimeCdf_Tail(float arrival_time, float distance, GaussianRV velocity);

// Inverse of ArrivalTimeCdf: returns arrival_time such that
// P(T <= arrival_time) = p, i.e. the arrival_time by which there's exactly
// probability p that arrival has already happened -- the "overshoot
// deadline" a single-pulse planner wants directly (plan a pulse no longer
// than this and the chance you've already covered `distance`, and are now
// overshooting, is bounded by p). Computed as distance / (the velocity
// threshold v such that P(V >= v) = p), via GaussianMotion_InvCdf_Tail --
// same exact-transform approach as ArrivalTimeCdf, so this too stays valid
// for velocity.mean near zero. Domain 0 < p < 1, same as InvCdf_Tail.
// Returns +INFINITY if that velocity threshold is <= 0 (including the
// p == 1 edge case) -- physically, no finite arrival_time bounds P(T <=
// arrival_time) at that probability when velocity that slow or reversed is
// itself plausible enough. Returns 0 for p == 0 (P(T <= 0) = 0 always).
float GaussianMotion_ArrivalTimeInvCdf(float p, float distance, GaussianRV velocity);

// Inverse of ArrivalTimeCdf_Tail: returns arrival_time such that
// P(T >= arrival_time) = p, i.e. the arrival_time by which there's exactly
// probability p that arrival has NOT yet happened. Computed as distance /
// (the velocity threshold v such that P(V <= v) = p), via
// GaussianMotion_InvCdf. Same domain/+INFINITY/near-zero-mean notes as
// ArrivalTimeInvCdf.
float GaussianMotion_ArrivalTimeInvCdf_Tail(float p, float distance, GaussianRV velocity);

// Prints mean/variance/stddev of g under `name`, for diagnostics.
void GaussianMotion_Print(const char *name, GaussianRV g);

#endif // GAUSSIAN_MOTION_H
