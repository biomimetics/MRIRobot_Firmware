#ifndef _MOTORDRIVER_H
#define _MOTORDRIVER_H
#include "include/core/reactor.h"
#include "_pidcontroller.h"
#include "_usm.h"
#ifndef TOP_LEVEL_PREAMBLE_990226843_H
#define TOP_LEVEL_PREAMBLE_990226843_H
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32_startup.h"
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
    bool value;
    #ifdef FEDERATED
    #ifdef FEDERATED_DECENTRALIZED
    tag_t intended_tag;
    #endif
    interval_t physical_time_of_arrival;
    #endif
} _motordriver_target_sel_t;
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
} _motordriver_target_speed_t;
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
} _motordriver_target_pos_t;
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
} _motordriver_qdec_current_t;
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
} _motordriver_qdec_sea_t;
typedef struct {
    struct self_base_t base;
    #line 27 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src/lib/Motor_Driver.lf"
    interval_t refresh_period;
#line 91 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src-gen/Main/_motordriver.h"
    #line 35 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src/lib/Motor_Driver.lf"
    bool* sel;
    #line 36 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src/lib/Motor_Driver.lf"
    float* speed_spc;
    #line 37 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src/lib/Motor_Driver.lf"
    float* speed_posc;
#line 98 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src-gen/Main/_motordriver.h"
    // Multiport input array will be malloc'd later.
    _motordriver_target_sel_t** _lf_target_sel;
    int _lf_target_sel_width;
    // Default input (in case it does not get connected)
    _motordriver_target_sel_t _lf_default__target_sel;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_target_sel__sparse;
    // Multiport input array will be malloc'd later.
    _motordriver_target_speed_t** _lf_target_speed;
    int _lf_target_speed_width;
    // Default input (in case it does not get connected)
    _motordriver_target_speed_t _lf_default__target_speed;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_target_speed__sparse;
    // Multiport input array will be malloc'd later.
    _motordriver_target_pos_t** _lf_target_pos;
    int _lf_target_pos_width;
    // Default input (in case it does not get connected)
    _motordriver_target_pos_t _lf_default__target_pos;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_target_pos__sparse;
    // Multiport input array will be malloc'd later.
    _motordriver_qdec_current_t** _lf_qdec_current;
    int _lf_qdec_current_width;
    // Default input (in case it does not get connected)
    _motordriver_qdec_current_t _lf_default__qdec_current;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_qdec_current__sparse;
    // Multiport input array will be malloc'd later.
    _motordriver_qdec_sea_t** _lf_qdec_sea;
    int _lf_qdec_sea_width;
    // Default input (in case it does not get connected)
    _motordriver_qdec_sea_t _lf_default__qdec_sea;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_qdec_sea__sparse;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u0;
    int _lf_control_u0_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u1;
    int _lf_control_u1_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u2;
    int _lf_control_u2_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u3;
    int _lf_control_u3_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u4;
    int _lf_control_u4_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u5;
    int _lf_control_u5_width;
    struct {
        _pidcontroller_out_t* out;
        trigger_t out_trigger;
        reaction_t* out_reactions[1];
    } _lf_control_u6;
    int _lf_control_u6_width;
    struct {
        _usm_set_speed_0_t set_speed_0;
        _usm_set_speed_1_t set_speed_1;
        _usm_set_speed_2_t set_speed_2;
        _usm_set_speed_3_t set_speed_3;
        _usm_set_speed_4_t set_speed_4;
        _usm_set_speed_5_t set_speed_5;
        _usm_set_speed_6_t set_speed_6;
    } _lf_usm;
    int _lf_usm_width;
    reaction_t _lf__reaction_0;
    reaction_t _lf__reaction_1;
    reaction_t _lf__reaction_2;
    trigger_t _lf__trigger;
    reaction_t* _lf__trigger_reactions[1];
    trigger_t _lf__target_sel;
    reaction_t* _lf__target_sel_reactions[1];
    #ifdef FEDERATED
    trigger_t* _lf__target_sel_network_port_status;
    
    #endif // FEDERATED
    trigger_t _lf__target_speed;
    reaction_t* _lf__target_speed_reactions[1];
    #ifdef FEDERATED
    trigger_t* _lf__target_speed_network_port_status;
    
    #endif // FEDERATED
    trigger_t _lf__target_pos;
    #ifdef FEDERATED
    trigger_t* _lf__target_pos_network_port_status;
    
    #endif // FEDERATED
    trigger_t _lf__qdec_current;
    #ifdef FEDERATED
    trigger_t* _lf__qdec_current_network_port_status;
    
    #endif // FEDERATED
    trigger_t _lf__qdec_sea;
    #ifdef FEDERATED
    trigger_t* _lf__qdec_sea_network_port_status;
    
    #endif // FEDERATED
} _motordriver_self_t;
_motordriver_self_t* new__motordriver();
#endif // _MOTORDRIVER_H
