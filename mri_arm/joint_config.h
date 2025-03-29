#include <stdio.h>

#ifndef __JOINT_CONFIG_H
#define __JOINT_CONFIG_H
typedef struct { /* __JOINT_CONFIG_H */
    float Kp;               // proportional control constant
    float Kd;               // differential control constant
    float lim;              // SEA limit
   } Joint_Config;




// --------------------------------------- Base joint --------------------------------------- 
static Joint_Config config0 = {
    40.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    0.3         // lim              - SEA limit
};

static Joint_Config config1 = {
    40.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    1.2         // lim              - SEA limit
};

static Joint_Config config2 = {
    40.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    0.8         // lim              - SEA limit
};
// --------------------------------------- Base joint --------------------------------------- 




// --------------------------------------- Elbow joint --------------------------------------- 
static Joint_Config config3 = {
    30.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    1.0         // lim              - SEA limit
};

static Joint_Config config4 = {
    30.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    0.6         // lim              - SEA limit
};
// --------------------------------------- Elbow joint --------------------------------------- 



// --------------------------------------- Wrist joint --------------------------------------- 
static Joint_Config config5 = {
    -10.0,      // Kp               - proportional control constant
    0.0,       // Kd               - differential control constant
    0.6         // lim              - SEA limit
};

static Joint_Config config6 = {
    -10.0,      // Kp               - proportional control constant
    0.0,       // Kd               - differential control constant
    0.3         // lim              - SEA limit
};
// --------------------------------------- Wrist joint --------------------------------------- 



/*
    We want to make an array of pointers to all the hard-coded encoder configs. 
        This allows us to easily access cofings as needed
*/
static Joint_Config* joint_configs[7] = {
    &config0, 
    &config1, 
    &config2, 
    &config3, 
    &config4, 
    &config5, 
    &config6
    };

#endif /* __JOINT_CONFIG_H */