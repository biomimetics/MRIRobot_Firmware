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
// NOTE: every reciprocal-based function below (ReciprocalLinearized/
// SecondOrder, ArrivalTimeLinearized/SecondOrder, ArrivalTimePdf) divides by
// velocity, implicitly assuming velocity.mean is bounded well away from zero
// relative to its stddev (a near-zero-mean reciprocal Gaussian has no finite
// mean/variance in the exact sense). That assumption does NOT generally
// hold in this codebase's own small-velocity/pulsing regimes (see
// dpos_pulse_mpc.c), where commanded velocity legitimately sits near zero --
// don't reach for these functions there without re-deriving for that
// regime.

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

// Y = 1/X via a first-order Taylor expansion of f(x)=1/x about mean(X):
// mean(Y) ~= 1/mean(X), Var(Y) ~= Var(X)/mean(X)^4. Only valid when
// mean(X) is large relative to sqrt(Var(X)) -- see the file-level NOTE.
GaussianRV GaussianMotion_ReciprocalLinearized(GaussianRV x);

// Same as ReciprocalLinearized, but mean(Y) also carries the standard
// second-order bias correction from f''(mean(X)): mean(Y) ~=
// (1/mean(X)) * (1 + Var(X)/mean(X)^2). Variance is left at the same
// first-order estimate -- no corresponding correction derived here.
GaussianRV GaussianMotion_ReciprocalSecondOrder(GaussianRV x);

// ---- Motion models: constant velocity V ~ N(mean, variance) over [0, t] ----

// Position after time `time`, starting from x0: X = x0 + V*time.
GaussianRV GaussianMotion_PositionDistribution(float x0, float time, GaussianRV velocity);

// Arrival time T = distance/V, via ReciprocalLinearized(velocity) scaled by
// distance.
GaussianRV GaussianMotion_ArrivalTimeLinearized(float distance, GaussianRV velocity);

// Same, via ReciprocalSecondOrder(velocity).
GaussianRV GaussianMotion_ArrivalTimeSecondOrder(float distance, GaussianRV velocity);

// Exact PDF of T = distance/V, via change-of-variables from velocity's own
// Gaussian density: f_T(t) = (distance/t^2) * f_V(distance/t). Not an
// approximation, unlike the two functions above -- but only the positive-t/
// positive-v branch of the transform: correct as long as velocity is
// physically one-signed (see the file-level NOTE for why that matters
// here). Returns 0 for arrival_time <= 0.
float GaussianMotion_ArrivalTimePdf(float arrival_time, float distance, GaussianRV velocity);

// Prints mean/variance/stddev of g under `name`, for diagnostics.
void GaussianMotion_Print(const char *name, GaussianRV g);

#endif // GAUSSIAN_MOTION_H
