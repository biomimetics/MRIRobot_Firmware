#include <stdio.h>
#include "common.h"

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H


typedef struct { /* __MOTOR_CONFIG_H */
    // Base Info
    int dir;                // Motor direction - maps motor direction to output sproket direction. Affected by gear trains.
    float max_speed;          // max motor speed in rad/s

    // Encoder Into
    float qdec_cpr;         // motor encoder count per rotation
    float sea_cpr;          // sea encoder counts per inch of linear travel (linear encoder, not counts/revolution)
    float sea_radius;       // lever-arm radius (inches) from the SEA's linear travel to the joint's angular deflection
    float sea_offset;       // sea reset offset
    float pwm_rad_per_sec_max; // max rad/s value to use for calculating duty cycles (dependent on external encoder cpr and expected encoder ratio)
    float command_bias;     // rad/s -- known per-motor MAGNITUDE-domain command CORRECTION, direction-agnostic: positive means this motor runs slower than commanded and the command should go UP by this much (new_command ~= |commanded| + command_bias), NOT the raw measured offset |actual|-|commanded| (that's the opposite sign -- see MotorCommandBiasEstimator.lf). Used to warm-start MotorCommandBiasEstimator.lf's online estimate instead of starting from 0 every boot; that reactor's live estimate is expected to refine away from whatever's set here. CommandSafetyFilter.lf applies the commanded direction's sign to the (warm-started/learned) total and gates the whole correction off when commanded velocity is exactly 0 -- see those reactors. 0.0 until bench-characterized per motor.
   } Motor_Config;

// --------------------------------------- Base joint ---------------------------------------
static Motor_Config motor0_config = {
    1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    40000,      // qdec_cpr         - motor encoder count per rotation // 10,000 base cpr * 4 scaling = 40,000
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    2.79528,    // sea_radius       - lever-arm radius, inches
    -0.5,       // sea_offset       - sea reset offset // UNUSED
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    0.00,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};


static Motor_Config motor1_config = {
    1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    40000,      // qdec_cpr         - motor encoder count per rotation // (40000 base cpr, 40k * 1 = 40k after x4 counts scaling.)
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    2.79528,    // sea_radius       - lever-arm radius, inches
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    0.010,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};


static Motor_Config motor2_config = {
    1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    40000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    2.79528,    // sea_radius       - lever-arm radius, inches
    -0.15,      // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    -0.075,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};
// --------------------------------------- Base joint ---------------------------------------




// --------------------------------------- Elbow joint ---------------------------------------
static Motor_Config motor3_config = {
    -1, //1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    8000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    1.775591,   // sea_radius       - lever-arm radius, inches
    2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    0.020,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};


static Motor_Config motor4_config = {
    -1, //1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    8000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    1.775591,   // sea_radius       - lever-arm radius, inches
    0,         // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    0.200,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};
// --------------------------------------- Elbow joint ---------------------------------------




// --------------------------------------- Wrist joint ---------------------------------------
static Motor_Config motor5_config = {
    -1,         // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    8000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    1.775591,   // sea_radius       - lever-arm radius, inches
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    0.46,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};


static Motor_Config motor6_config = {
    -1,         // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    8000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    1.775591,   // sea_radius       - lever-arm radius, inches
    -0.2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    0.46,        // command_bias      - rad/s, warm-start for MotorCommandBiasEstimator.lf
};
// --------------------------------------- Wrist joint ---------------------------------------





/*
    We want to make an array of pointers to all the hard-coded encoder configs.
        This allows us to easily access cofings as needed
*/
static Motor_Config* motor_configs[7] = {
    &motor0_config,
    &motor1_config,
    &motor2_config,
    &motor3_config,
    &motor4_config,
    &motor5_config,
    &motor6_config
    };

#endif /* __MOTOR_CONFIG_H */
