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
};


static Motor_Config motor1_config = {
    1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    40000,      // qdec_cpr         - motor encoder count per rotation // (40000 base cpr, 40k * 1 = 40k after x4 counts scaling.)
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    2.79528,    // sea_radius       - lever-arm radius, inches
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
};


static Motor_Config motor2_config = {
    1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    40000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    2.79528,    // sea_radius       - lever-arm radius, inches
    -0.15,      // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
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
};


static Motor_Config motor4_config = {
    -1, //1,          // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    8000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    1.775591,   // sea_radius       - lever-arm radius, inches
    0,         // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
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
};


static Motor_Config motor6_config = {
    -1,         // dir              - Motor direction
    12.566,       // max_speed        - max motor speed in rad/s, 2 rotations/sec (4*pi rad/s)
    8000,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder counts per inch of linear travel
    1.775591,   // sea_radius       - lever-arm radius, inches
    -0.2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
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
