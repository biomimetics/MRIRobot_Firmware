#include <stdio.h>

#ifndef __JOINT_CONFIG_H
#define __JOINT_CONFIG_H
typedef struct { /* __JOINT_CONFIG_H */
    float Kp;               // proportional control constant
    float Kd;               // differential control constant
    float lim;              // SEA limit
    float Ks;              // SEA limit
   } Joint_Config;




// --------------------------------------- Base joint --------------------------------------- 
static Joint_Config config0 = {
    40.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    3.0,        // lim              - SEA limit
    10          // Ks
};

static Joint_Config config1 = {
    40.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    2.2,        // lim              - SEA limit
    40          // Ks
};

static Joint_Config config2 = {
    40.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    2.0,        // lim              - SEA limit
    40          // Ks
};
// --------------------------------------- Base joint --------------------------------------- 




// --------------------------------------- Elbow joint --------------------------------------- 
static Joint_Config config3 = {
    20.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    1.8,        // lim              - SEA limit
    80          // Ks
};

static Joint_Config config4 = {
    20.0,       // Kp               - proportional control constant
    10.0,       // Kd               - differential control constant
    1.4,        // lim              - SEA limit
    80          // Ks
};
// --------------------------------------- Elbow joint --------------------------------------- 



// --------------------------------------- Wrist joint --------------------------------------- 
static Joint_Config config5 = {
    -10.0,      // Kp               - proportional control constant
    0.0,       // Kd               - differential control constant
    1.2,        // lim              - SEA limit
    40          // Ks
};

static Joint_Config config6 = {
    -10.0,      // Kp               - proportional control constant
    0.0,        // Kd               - differential control constant
    0.7,        // lim              - SEA limit
    40          // Ks
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