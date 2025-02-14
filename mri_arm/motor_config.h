#include <stdio.h>

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H
typedef struct { /* __MOTOR_CONFIG_H */
    // Base Info
    int dir;                // Motor direction
    int max_speed;          // max motor speed

    // Encoder Into
    float qdec_cpr;         // motor encoder count per rotation
    float sea_cpr;          // motor encoder count per rotation
    float sea_offset;       // sea reset offset

    // Movement info
    float speed_ratio;      // motor speed ratio
    float gear_ratio;       // motor gear box gear ratio
   } Motor_Config;



// Define config for motor 0
static Motor_Config motor0_config = {
    1,          // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // qdec_cpr     - motor encoder count per rotation
    0.0,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};


static Motor_Config motor1_config = {
    1,          // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // sea_cpr      - sea encoder count per rotation
    0.0,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};


static Motor_Config motor2_config = {
    1,          // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // sea_cpr      - sea encoder count per rotation
    0.0,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};


static Motor_Config motor3_config = {
    1,          // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // sea_cpr      - sea encoder count per rotation
    12,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};


static Motor_Config motor4_config = {
    1,          // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // sea_cpr      - sea encoder count per rotation
    160,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};


static Motor_Config motor5_config = {
    -1,         // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // sea_cpr      - sea encoder count per rotation
    20,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};


static Motor_Config motor6_config = {
    -1,         // dir          - Motor direction
    500,        // max_speed    - max motor speed
    23040,      // qdec_cpr     - motor encoder count per rotation
    400,        // sea_cpr      - sea encoder count per rotation
    100,        // sea_offset   - sea reset offset
    1.0,        // speed_ratio  - motor speed ration 
    1.0         // gear_ratio   - motor gear box gear ratio
};





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