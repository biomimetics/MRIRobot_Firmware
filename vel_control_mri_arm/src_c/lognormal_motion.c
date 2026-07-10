#include "lognormal_motion.h"
#include <math.h>
#include <stdio.h>

LogNormalRV LogNormalMotion_ArrivalTimeDistribution(LogNormalRV distance, LogNormalRV velocity) {
  return LogNormalRV_Divide(distance, velocity);
}

float LogNormalMotion_ArrivalTimePdf(float arrival_time, LogNormalRV distance, LogNormalRV velocity) {
  return LogNormalRV_Pdf(arrival_time, LogNormalMotion_ArrivalTimeDistribution(distance, velocity));
}

float LogNormalMotion_ArrivalTimeCdf(float arrival_time, LogNormalRV distance, LogNormalRV velocity) {
  return LogNormalRV_Cdf(arrival_time, LogNormalMotion_ArrivalTimeDistribution(distance, velocity));
}

float LogNormalMotion_ArrivalTimeCdf_Tail(float arrival_time, LogNormalRV distance, LogNormalRV velocity) {
  return LogNormalRV_Cdf_Tail(arrival_time, LogNormalMotion_ArrivalTimeDistribution(distance, velocity));
}

float LogNormalMotion_ArrivalTimeInvCdf(float p, LogNormalRV distance, LogNormalRV velocity) {
  return LogNormalRV_InvCdf(p, LogNormalMotion_ArrivalTimeDistribution(distance, velocity));
}

float LogNormalMotion_ArrivalTimeInvCdf_Tail(float p, LogNormalRV distance, LogNormalRV velocity) {
  return LogNormalRV_InvCdf_Tail(p, LogNormalMotion_ArrivalTimeDistribution(distance, velocity));
}

void LogNormalMotion_Print(const char *name, LogNormalRV ln) {
  printf("%s\n", name);
  printf("  mu       : %.6f\n", (double) ln.mu);
  printf("  sigma2   : %.6f\n", (double) ln.sigma2);
  printf("  mean     : %.6f\n", (double) LogNormalRV_Mean(ln));
  printf("  stddev   : %.6f\n\n", (double) LogNormalRV_StdDev(ln));
}
