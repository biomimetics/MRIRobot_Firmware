#include <stdio.h>

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

// ######
// ### Gloabl config settings for all motors

  // DEBUGGING
  #define PRINT_USM 0
  #define PRINT_ENCODER 0

  // PWM CONFIGS
  #define ARR_PERIOD 2000 // 16-bit timer period for ARR (auto reload register). Can go up to 65535 but the RC filters on the board aren't suited for values above 2000
  #define PWM_RAD_PER_SEC_MAX 26.17993875 // 250 RPM = 26.1799 rad/s, this scales the input rad/s down to [0,1] range for converting to duty cycle
  
  // LIMITS
  #define USM_MAX_DUTY_CYCLE 0.30 // max allowable duty cycle. If above this, we force it to saturate. 
  #define USM_MIN_DUTY_CYCLE 0.005 // If below this amount, apply a deadband to keep motors from jittering around low values.
  
  // CONSTANTS AND CONVERSION
  #define TWO_PI 6.28318
  #define NSEC_TO_SEC 0.000000001
  #define MSEC_TO_SEC 0.001

  #define RAD_PER_SEC_TO_DEG_PER_SEC 57.2958 // only used for printing
  #define RPM_TO_DEG_PER_SEC 6.0

//
typedef struct { // __MOTOR_CONFIG_H

    // Encoder Into
    float qdec_cpr;         // motor encoder count per rotation
    float sea_cpr;          // sea encoder count per rotation

   } Motor_Config;
   

// --------------------------------------- Base joint --------------------------------------- 
static Motor_Config motor0_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation // see sea encoder scratch spreadsheet for derivation
};


static Motor_Config motor1_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
};


static Motor_Config motor2_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
};
// --------------------------------------- Base joint --------------------------------------- 


// --------------------------------------- Elbow joint --------------------------------------- 
static Motor_Config motor3_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
};


static Motor_Config motor4_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
};
// --------------------------------------- Elbow joint --------------------------------------- 




// --------------------------------------- Wrist joint --------------------------------------- 
static Motor_Config motor5_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotati--1---on
};


static Motor_Config motor6_config = {
    23040,      // qdec_cpr         - motor encoder count per rotation
    22252,       // sea_cpr          - sea encoder count per rotation
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

/*
// old motor_config structure
typedef struct { // __MOTOR_CONFIG_H 
    // Base Info
    int dir;                // Motor direction - maps motor direction to output sproket direction. Affected by gear trains.
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
*/
