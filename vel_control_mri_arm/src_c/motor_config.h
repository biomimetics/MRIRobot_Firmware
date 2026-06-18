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
    float sea_cpr;          // motor encoder count per rotation
    float sea_gear_ratio;   // sea motor gear ratio
    float sea_offset;       // sea reset offset
    float pwm_rad_per_sec_max; // max rad/s value to use for calculating duty cycles (dependent on external encoder cpr and expected encoder ratio)

    // Movement info
    float speed_ratio;      // motor speed ratio
    float gear_ratio;       // motor gear box gear ratio
   } Motor_Config;

// --------------------------------------- Base joint --------------------------------------- 
static Motor_Config motor0_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s // APPLY THESE BEFORE GEAR REDUCTION
    40000,      // qdec_cpr         - motor encoder count per rotation // 10,000 base cpr * 4 scaling = 40,000
    31614,       // sea_cpr          - sea encoder count per rotation // see sea encoder scratch spreadsheet for derivation, basically 8000 (lines/inch) / 25.4(mm/inch) = 314.96 lines/mm --> 314.96 lines/mm * 1000 mm/meter / 0.2826 circ/meter = 1,114,508.138 lines/circ which is counts per revolution. I'm sleep deprvied so hopefully that's not wrong, check it soon.
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.5,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor1_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s
    40000,      // qdec_cpr         - motor encoder count per rotation // (2000 base cpr, 2k * 4 = 8k after x4 counts scaling.)
    31614,       // sea_cpr          - sea encoder count per rotation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor2_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s
    40000,      // qdec_cpr         - motor encoder count per rotation
    31614,       // sea_cpr          - sea encoder count per rotation
    20.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.15,      // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Base joint --------------------------------------- 




// --------------------------------------- Elbow joint --------------------------------------- 
static Motor_Config motor3_config = {
    -1, //1,          // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22251,       // sea_cpr          - sea encoder count per rotation
    13.0,       // sea_gear_ratio   - sea motor gear ratio
    2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor4_config = {
    -1, //1,          // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22251,       // sea_cpr          - sea encoder count per rotation
    15.0,       // sea_gear_ratio   - sea motor gear ratio
    0,         // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Elbow joint --------------------------------------- 




// --------------------------------------- Wrist joint --------------------------------------- 
static Motor_Config motor5_config = {
    -1,         // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22251,       // sea_cpr          - sea encoder count per rotati--1---on
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor6_config = {
    -1,         // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22251,       // sea_cpr          - sea encoder count per rotation
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    -0.2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
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


// OLD - KEEPING FOR BACKUPS
/*
// --------------------------------------- Base joint --------------------------------------- 
static Motor_Config motor0_config = {
    1,          // dir              - Motor direction
    300,        // max_speed        - max motor speed // what unit??? Looks like it was in CCR, so 300/1000 = 30% duty cycle!
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // qdec_cpr         - motor encoder count per rotation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.5,       // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor1_config = {
    1,          // dir              - Motor direction
    300,        // max_speed        - max motor speed
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder count per rotation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor2_config = {
    1,          // dir              - Motor direction
    300,        // max_speed        - max motor speed
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder count per rotation
    20.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.15,      // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Base joint --------------------------------------- 




// --------------------------------------- Elbow joint --------------------------------------- 
static Motor_Config motor3_config = {
    1,          // dir              - Motor direction
    150,        // max_speed        - max motor speed
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder count per rotation
    13.0,       // sea_gear_ratio   - sea motor gear ratio
    2,       // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor4_config = {
    1,          // dir              - Motor direction
    150,        // max_speed        - max motor speed
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder count per rotation
    15.0,       // sea_gear_ratio   - sea motor gear ratio
    0,         // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Elbow joint --------------------------------------- 




// --------------------------------------- Wrist joint --------------------------------------- 
static Motor_Config motor5_config = {
    -1,         // dir              - Motor direction
    150,        // max_speed        - max motor speed
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder count per rotati--1---on
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor6_config = {
    -1,         // dir              - Motor direction
    150,        // max_speed        - max motor speed
    23040,      // qdec_cpr         - motor encoder count per rotation
    8000,       // sea_cpr          - sea encoder count per rotation
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    -0.2,       // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Wrist joint --------------------------------------- 

/* other set of backup values, before I triple checked the sea counts code again
typedef struct {
    // Base Info
    int dir;                // Motor direction - maps motor direction to output sproket direction. Affected by gear trains.
    float max_speed;          // max motor speed in rad/s

    // Encoder Into
    float qdec_cpr;         // motor encoder count per rotation
    float sea_cpr;          // motor encoder count per rotation
    float sea_gear_ratio;   // sea motor gear ratio
    float sea_offset;       // sea reset offset
    float pwm_rad_per_sec_max; // max rad/s value to use for calculating duty cycles (dependent on external encoder cpr and expected encoder ratio)

    // Movement info
    float speed_ratio;      // motor speed ratio
    float gear_ratio;       // motor gear box gear ratio
   } Motor_Config;

// --------------------------------------- Base joint --------------------------------------- 
static Motor_Config motor0_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s // APPLY THESE BEFORE GEAR REDUCTION
    40000,      // qdec_cpr         - motor encoder count per rotation // 10,000 base cpr * 4 scaling = 40,000
    22252,       // sea_cpr          - sea encoder count per rotation // see sea encoder scratch spreadsheet for derivation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.5,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor1_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s
    40000,      // qdec_cpr         - motor encoder count per rotation // (2000 base cpr, 2k * 4 = 8k after x4 counts scaling.)
    22252,       // sea_cpr          - sea encoder count per rotation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor2_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s
    40000,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    20.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.15,      // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX_BASE,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Base joint --------------------------------------- 




// --------------------------------------- Elbow joint --------------------------------------- 
static Motor_Config motor3_config = {
    -1, //1,          // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    13.0,       // sea_gear_ratio   - sea motor gear ratio
    2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor4_config = {
    -1, //1,          // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    15.0,       // sea_gear_ratio   - sea motor gear ratio
    0,         // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Elbow joint --------------------------------------- 




// --------------------------------------- Wrist joint --------------------------------------- 
static Motor_Config motor5_config = {
    -1,         // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotati--1---on
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor6_config = {
    -1,         // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    8000,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    -0.2,       // sea_offset       - sea reset offset
    PWM_RAD_PER_SEC_MAX,            // pwm_rad_per_sec_max
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Wrist joint --------------------------------------- 


*/