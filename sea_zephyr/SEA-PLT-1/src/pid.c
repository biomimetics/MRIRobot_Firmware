#include "pid.h"
#include <math.h>
#include <sys/util.h>
#include <timing/timing.h>
#include "stm32_std.h"
#include "motor.h"
#include "encoder.h"

int target_pos = 0;
int last_pos = 0;
long last_time = 0;
float error_i = 0;

float Kp = 1;
float Ki = 0.01;
float Kd = 0.1;
float Kw = 0.9;

long long int dt;
long long int time;

void setup_pid() {
    dt = k_uptime_get();        // get initial time marker
    time = k_uptime_delta(&dt); // get current time in milliseconds
    error_i = 0;
}

void set_target(int target) {
    target_pos = target;

}

int get_pid() {
    long dt = k_uptime_delta(&dt);
    double current_pos = read_deg();

    // Calculate PID errors
    float error_p = target_pos - current_pos;
    float error_d = (current_pos - last_pos) / (dt);
    error_i = (error_p) + (Kw * error_i);

    // Update previosu timestep's values
    last_pos = current_pos;
    // last_time = current_time;

    // get PID control value
    int PID_val = (int)(Kp * error_p + Ki * error_i + Kd * error_d);

    // Set values
    return PID_val;
}