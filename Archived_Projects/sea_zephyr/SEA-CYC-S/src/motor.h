#ifndef MOTOR_H
#define MOTOR_H

#include <zephyr.h>
#include "stm32_std.h"

// setup function
int setupMotors();

// tsetsing function
void testMotor();

// function definitions
void setMotorSpeed(double speed);

#endif