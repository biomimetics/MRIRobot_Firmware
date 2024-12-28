// #ifndef TEST_H
// #define TEST_H

#include <zephyr.h>
#include "motor.h"
#include "encoder.h"

void runSpeedTest(double speed);
void runForceTest(double freq, int mag);
void runImpedanceTest(double k, double a, double kp, double kd);

void printVals(double degree, double target, uint64_t time, double freq, double mag, uint64_t maxtime, double speed);
uint64_t determineSimTime(double freq);

// #endif