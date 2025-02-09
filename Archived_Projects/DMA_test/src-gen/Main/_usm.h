#ifndef _USM_H
#define _USM_H
#include "include/core/reactor.h"
#ifndef TOP_LEVEL_PREAMBLE_1940749968_H
#define TOP_LEVEL_PREAMBLE_1940749968_H
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
} _usm_speed_t;
typedef struct {
    struct self_base_t base;
#line 26 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.h"
#line 27 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.h"
    // Multiport input array will be malloc'd later.
    _usm_speed_t** _lf_speed;
    int _lf_speed_width;
    // Default input (in case it does not get connected)
    _usm_speed_t _lf_default__speed;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_speed__sparse;
    reaction_t _lf__reaction_0;
    reaction_t _lf__reaction_1;
    reaction_t _lf__reaction_2;
    reaction_t _lf__reaction_3;
    reaction_t _lf__reaction_4;
    trigger_t _lf__motor_tick0;
    reaction_t* _lf__motor_tick0_reactions[1];
    trigger_t _lf__motor_tick1;
    reaction_t* _lf__motor_tick1_reactions[1];
    trigger_t _lf__motor_tick2;
    reaction_t* _lf__motor_tick2_reactions[1];
    trigger_t _lf__startup;
    reaction_t* _lf__startup_reactions[1];
    trigger_t _lf__speed;
    reaction_t* _lf__speed_reactions[1];
    #ifdef FEDERATED
    trigger_t* _lf__speed_network_port_status;
    
    #endif // FEDERATED
} _usm_self_t;
_usm_self_t* new__usm();
#endif // _USM_H
