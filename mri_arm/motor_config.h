#include <stdio.h>

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H
typedef struct { /* __MOTOR_CONFIG_H */
    // Base Info
    int dir;                // Motor direction
    float max_speed;          // max motor speed in rad/s

    // Encoder Into
    float qdec_cpr;         // motor encoder count per rotation
    float sea_cpr;          // motor encoder count per rotation
    float sea_gear_ratio;   // sea motor gear ratio
    float sea_offset;       // sea reset offset

    // Movement info
    float speed_ratio;      // motor speed ratio
    float gear_ratio;       // motor gear box gear ratio
   } Motor_Config;



// --------------------------------------- Base joint --------------------------------------- 
static Motor_Config motor0_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s // APPLY THESE BEFORE GEAR REDUCTION
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation // see sea encoder scratch spreadsheet for derivation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.5,       // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor1_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    22.0,       // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor2_config = {
    1,          // dir              - Motor direction
    0.52,       // max_speed        - max motor speed in rad/s, about 30 deg/s
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    20.0,       // sea_gear_ratio   - sea motor gear ratio
    -0.15,      // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    21.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Base joint --------------------------------------- 




// --------------------------------------- Elbow joint --------------------------------------- 
static Motor_Config motor3_config = {
    1,          // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    13.0,       // sea_gear_ratio   - sea motor gear ratio
    2,       // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor4_config = {
    1,          // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    15.0,       // sea_gear_ratio   - sea motor gear ratio
    0,         // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    12.0,       // gear_ratio       - motor gear box gear ratio
};
// --------------------------------------- Elbow joint --------------------------------------- 




// --------------------------------------- Wrist joint --------------------------------------- 
static Motor_Config motor5_config = {
    -1,         // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotati--1---on
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    0.3,        // sea_offset       - sea reset offset
    1.0,        // speed_ratio      - motor speed ration 
    52.0/16.0,  // gear_ratio       - motor gear box gear ratio
};


static Motor_Config motor6_config = {
    -1,         // dir              - Motor direction
    0.26,       // max_speed        - max motor speed in rad/s, about 15 deg/s
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
    36.0/16.0,  // sea_gear_ratio   - sea motor gear ratio
    -0.2,       // sea_offset       - sea reset offset
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
*/