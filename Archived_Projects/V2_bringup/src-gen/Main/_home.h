#ifndef _HOME_H
#define _HOME_H
#include "include/core/reactor.h"
#include "_switches.h"
#ifndef TOP_LEVEL_PREAMBLE_1677590477_H
#define TOP_LEVEL_PREAMBLE_1677590477_H
#include "stm32f4xx_hal.h"
#include "stm32_startup.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32_startup.h"
#include <stdio.h>
#endif
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;
    #ifdef FEDERATED
    #ifdef FEDERATED_DECENTRALIZED
    tag_t intended_tag;
    #endif
    interval_t physical_time_of_arrival;
    #endif
} _home_current_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;
    #ifdef FEDERATED
    #ifdef FEDERATED_DECENTRALIZED
    tag_t intended_tag;
    #endif
    interval_t physical_time_of_arrival;
    #endif
} _home_sea_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;
    #ifdef FEDERATED
    #ifdef FEDERATED_DECENTRALIZED
    tag_t intended_tag;
    #endif
    interval_t physical_time_of_arrival;
    #endif
} _home_motor_speed_t;
typedef struct {
    struct self_base_t base;
    #line 17 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    interval_t sample_period;
#line 61 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_home.h"
    #line 25 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    int* is_home;
    #line 27 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    float* home_speed;
    #line 29 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    float* last_pos;
    #line 30 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    float* last_sea;
    #line 31 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    float* pos_diff;
    #line 33 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    int* diffzero_cnt;
    #line 35 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    int update_cnt;
#line 76 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_home.h"
    // Multiport input array will be malloc'd later.
    _home_current_pos_t** _lf_current_pos;
    int _lf_current_pos_width;
    // Default input (in case it does not get connected)
    _home_current_pos_t _lf_default__current_pos;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_current_pos__sparse;
    // Multiport input array will be malloc'd later.
    _home_sea_pos_t** _lf_sea_pos;
    int _lf_sea_pos_width;
    // Default input (in case it does not get connected)
    _home_sea_pos_t _lf_default__sea_pos;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_sea_pos__sparse;
    // Array of output ports.
    _home_motor_speed_t* _lf_motor_speed;
    int _lf_motor_speed_width;
    // An array of pointers to the individual ports. Useful
    // for the lf_set macros to work out-of-the-box for
    // multiports in the body of reactions because their 
    // value can be accessed via a -> operator (e.g.,foo[i]->value).
    // So we have to handle multiports specially here a construct that
    // array of pointers.
    _home_motor_speed_t** _lf_motor_speed_pointers;
    struct {
        _switches_read_t** read;
        int read_width;
        trigger_t read_trigger;
        reaction_t* read_reactions[1];
    } _lf_homing_switches;
    int _lf_homing_switches_width;
    reaction_t _lf__reaction_0;
    trigger_t _lf__trigger;
    trigger_t _lf__current_pos;
    reaction_t* _lf__current_pos_reactions[1];
    #ifdef FEDERATED
    trigger_t* _lf__current_pos_network_port_status;
    
    #endif // FEDERATED
    trigger_t _lf__sea_pos;
    reaction_t* _lf__sea_pos_reactions[1];
    #ifdef FEDERATED
    trigger_t* _lf__sea_pos_network_port_status;
    
    #endif // FEDERATED
} _home_self_t;
_home_self_t* new__home();
#endif // _HOME_H
