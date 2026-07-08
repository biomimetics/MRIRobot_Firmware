#include "gaussian_motion.h"
#include <math.h>
#include <stdio.h>

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
  return (distance / (arrival_time * arrival_time)) * GaussianRV_Pdf(v, velocity);
}

float GaussianMotion_ArrivalTimeCdf(float arrival_time, float distance, GaussianRV velocity) {
  if (arrival_time <= 0.0f) {
    return 0.0f;
  }
  return GaussianRV_Cdf_Tail(distance / arrival_time, velocity);
}

float GaussianMotion_ArrivalTimeCdf_Tail(float arrival_time, float distance, GaussianRV velocity) {
  if (arrival_time <= 0.0f) {
    return 1.0f;
  }
  return GaussianRV_Cdf(distance / arrival_time, velocity);
}

float GaussianMotion_ArrivalTimeInvCdf(float p, float distance, GaussianRV velocity) {
  float v_threshold = GaussianRV_InvCdf_Tail(p, velocity);
  if (v_threshold <= 0.0f) {
    return INFINITY;
  }
  return distance / v_threshold;
}

float GaussianMotion_ArrivalTimeInvCdf_Tail(float p, float distance, GaussianRV velocity) {
  float v_threshold = GaussianRV_InvCdf(p, velocity);
  if (v_threshold <= 0.0f) {
    return INFINITY;
  }
  return distance / v_threshold;
}

void GaussianMotion_Print(const char *name, GaussianRV g) {
  printf("%s\n", name);
  printf("  mean     : %.6f\n", (double) g.mean);
  printf("  variance : %.6f\n", (double) g.variance);
  printf("  stddev   : %.6f\n\n", (double) GaussianRV_StdDev(g));
}
