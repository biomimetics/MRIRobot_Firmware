#ifndef _SEACONTROLLER2_H
#define _SEACONTROLLER2_H
#include "include/core/reactor.h"
#include "_pidcontroller.h"
#include "_ffbcontroller.h"
#ifndef TOP_LEVEL_PREAMBLE_69160933_H
#define TOP_LEVEL_PREAMBLE_69160933_H
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"
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
} _seacontroller2_current_pos_t;
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
} _seacontroller2_sea_pos_t;
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
} _seacontroller2_target_pos_t;
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
} _seacontroller2_out_t;
typedef struct {
    struct self_base_t base;
    #line 18 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float Kp;
    #line 19 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float Ki;
    #line 20 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float Kd;
    #line 21 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float Ks;
    #line 22 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float SLIM;
    #line 23 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float SEA_LIM;
#line 86 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_seacontroller2.h"
    #line 33 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    int fsm_state;
    #line 34 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float last_target;
    #line 35 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float last_sea;
    #line 37 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    float last_out;
    #line 39 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    int cycle_count;
    #line 41 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA2.lf"
    bool print_data;
#line 99 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_seacontroller2.h"
    _seacontroller2_current_pos_t* _lf_current_pos;
    // width of -2 indicates that it is not a multiport.
    int _lf_current_pos_width;
    // Default input (in case it does not get connected)
    _seacontroller2_current_pos_t _lf_default__current_pos;
    _seacontroller2_sea_pos_t* _lf_sea_pos;
    // width of -2 indicates that it is not a multiport.
    int _lf_sea_pos_width;
    // Default input (in case it does not get connected)
    _seacontroller2_sea_pos_t _lf_default__sea_pos;
    _seacontroller2_target_pos_t* _lf_target_pos;
    // width of -2 indicates that it is not a multiport.
    int _lf_target_pos_width;
    // Default input (in case it does not get connected)
    _seacontroller2_target_pos_t _lf_default__target_pos;
    _seacontroller2_out_t _lf_out;
    int _lf_out_width;
    struct {
        _ffbcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_force_control;
    int _lf_force_control_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_pos_control;
    int _lf_pos_control_width;
    reaction_t _lf__reaction_0;
    reaction_t _lf__reaction_1;
    trigger_t _lf__current_pos;
    #ifdef FEDERATED
    
    #endif // FEDERATED
    trigger_t _lf__sea_pos;
    reaction_t* _lf__sea_pos_reactions[1];
    #ifdef FEDERATED
    
    #endif // FEDERATED
    trigger_t _lf__target_pos;
    reaction_t* _lf__target_pos_reactions[1];
    #ifdef FEDERATED
    
    #endif // FEDERATED
} _seacontroller2_self_t;
_seacontroller2_self_t* new__seacontroller2();
#endif // _SEACONTROLLER2_H
