#include <stdio.h>

#ifndef __CONTROL_CONFIG_H
#define __CONTROL_CONFIG_H

typedef struct {
    float Kp;                     // proportional control gain
    float Kd;                     // differential control gain
    float Ki;                     // integral control gain
    float integral_clamping_max;  // maximum intergral control signal
    float control_signal_max;     // maximum control signal
   } PID_Config;


// ----------------------------------- Position Controllers ---------------------------------

// --------------------------------------- Base joint --------------------------------------- 
static PID_Config pos_pid_config_0 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };
// --------------------------------------- Base joint --------------------------------------- 


// --------------------------------------- Shoulder joint -----------------------------------
static PID_Config pos_pid_config_1 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };

static PID_Config pos_pid_config_2 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };

// --------------------------------------- Shoulder joint -----------------------------------


// --------------------------------------- Elbow joint --------------------------------------- 
static PID_Config pos_pid_config_3 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };

static PID_Config pos_pid_config_4 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };
// --------------------------------------- Elbow joint --------------------------------------- 



// --------------------------------------- Wrist joint --------------------------------------- 
static PID_Config pos_pid_config_5 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };

static PID_Config pos_pid_config_6 = {
    10.0, // kp // was 10.0
    0.10, // kd // was 0.10
    0.00, // ki // was 0.01
    10.0,
    10.0 
  };
// --------------------------------------- Wrist joint --------------------------------------- 


/*
    We want to make an array of pointers to all the hard-coded encoder configs. 
        This allows us to easily access cofings as needed
*/
static PID_Config* pos_pid_configs[7] = {
    &pos_pid_config_0, 
    &pos_pid_config_1, 
    &pos_pid_config_2, 
    &pos_pid_config_3, 
    &pos_pid_config_4, 
    &pos_pid_config_5, 
    &pos_pid_config_6
    };

// ----------------------------------- Position Controllers ---------------------------------

// ----------------------------------- Velocity Controllers ---------------------------------

// --------------------------------------- Base joint --------------------------------------- 
static PID_Config vel_pid_config_0 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };
// --------------------------------------- Base joint --------------------------------------- 


// --------------------------------------- Shoulder joint -----------------------------------
static PID_Config vel_pid_config_1 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };

static PID_Config vel_pid_config_2 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };

// --------------------------------------- Shoulder joint -----------------------------------


// --------------------------------------- Elbow joint --------------------------------------- 
static PID_Config vel_pid_config_3 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };

static PID_Config vel_pid_config_4 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };
// --------------------------------------- Elbow joint --------------------------------------- 



// --------------------------------------- Wrist joint --------------------------------------- 
static PID_Config vel_pid_config_5 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };

static PID_Config vel_pid_config_6 = {
    0.001, // kp
    0.000, // kd
    0.000, // ki
    10.0,
    10.0 
  };
// --------------------------------------- Wrist joint --------------------------------------- 

/*
    We want to make an array of pointers to all the hard-coded encoder configs. 
        This allows us to easily access cofings as needed
*/
static PID_Config* vel_pid_configs[7] = {
    &vel_pid_config_0, 
    &vel_pid_config_1, 
    &vel_pid_config_2, 
    &vel_pid_config_3, 
    &vel_pid_config_4, 
    &vel_pid_config_5, 
    &vel_pid_config_6
    };

// ----------------------------------- Position Controllers ---------------------------------

#endif /* __CONTROL_CONFIG_H */