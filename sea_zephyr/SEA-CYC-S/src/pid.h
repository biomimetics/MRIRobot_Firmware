#ifndef PID_H
#define PID_H

#include <zephyr.h>
#include <math.h>

void set_target(int target);
int get_pid();

#endif