#ifndef _FFBCONTROLLER_H
#define _FFBCONTROLLER_H
#include "include/core/reactor.h"
#ifndef TOP_LEVEL_PREAMBLE_1053695609_H
#define TOP_LEVEL_PREAMBLE_1053695609_H
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
} _ffbcontroller_current_pos_t;
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
} _ffbcontroller_sea_pos_t;
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
} _ffbcontroller_target_pos_t;
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
} _ffbcontroller_out_t;
typedef struct {
    struct self_base_t base;
    #line 14 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    float Kp;
    #line 14 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    float LIM;
    #line 14 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    float CHANGE_P;
    #line 14 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    float CHANGE_D;
#line 76 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ffbcontroller.h"
    #line 20 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    int fsm_state;
    #line 21 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    float last_sea;
#line 81 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ffbcontroller.h"
    _ffbcontroller_current_pos_t* _lf_current_pos;
    // width of -2 indicates that it is not a multiport.
    int _lf_current_pos_width;
    // Default input (in case it does not get connected)
    _ffbcontroller_current_pos_t _lf_default__current_pos;
    _ffbcontroller_sea_pos_t* _lf_sea_pos;
    // width of -2 indicates that it is not a multiport.
    int _lf_sea_pos_width;
    // Default input (in case it does not get connected)
    _ffbcontroller_sea_pos_t _lf_default__sea_pos;
    _ffbcontroller_target_pos_t* _lf_target_pos;
    // width of -2 indicates that it is not a multiport.
    int _lf_target_pos_width;
    // Default input (in case it does not get connected)
    _ffbcontroller_target_pos_t _lf_default__target_pos;
    _ffbcontroller_out_t _lf_out;
    int _lf_out_width;
    reaction_t _lf__reaction_0;
    trigger_t _lf__current_pos;
    reaction_t* _lf__current_pos_reactions[1];
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
} _ffbcontroller_self_t;
_ffbcontroller_self_t* new__ffbcontroller();
#endif // _FFBCONTROLLER_H
