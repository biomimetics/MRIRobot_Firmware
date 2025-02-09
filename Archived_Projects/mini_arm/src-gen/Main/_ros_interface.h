#ifndef _ROS_INTERFACE_H
#define _ROS_INTERFACE_H
#include "include/core/reactor.h"
#ifndef TOP_LEVEL_PREAMBLE_19467337_H
#define TOP_LEVEL_PREAMBLE_19467337_H
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
} _ros_interface_current_pos_t;
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
} _ros_interface_sea_pos_t;
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
} _ros_interface_target_pos_t;
typedef struct {
    struct self_base_t base;
#line 55 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_ros_interface.h"
#line 56 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_ros_interface.h"
    // Multiport input array will be malloc'd later.
    _ros_interface_current_pos_t** _lf_current_pos;
    int _lf_current_pos_width;
    // Default input (in case it does not get connected)
    _ros_interface_current_pos_t _lf_default__current_pos;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_current_pos__sparse;
    // Multiport input array will be malloc'd later.
    _ros_interface_sea_pos_t** _lf_sea_pos;
    int _lf_sea_pos_width;
    // Default input (in case it does not get connected)
    _ros_interface_sea_pos_t _lf_default__sea_pos;
    // Struct to support efficiently reading sparse inputs.
    lf_sparse_io_record_t* _lf_sea_pos__sparse;
    // Array of output ports.
    _ros_interface_target_pos_t* _lf_target_pos;
    int _lf_target_pos_width;
    // An array of pointers to the individual ports. Useful
    // for the lf_set macros to work out-of-the-box for
    // multiports in the body of reactions because their 
    // value can be accessed via a -> operator (e.g.,foo[i]->value).
    // So we have to handle multiports specially here a construct that
    // array of pointers.
    _ros_interface_target_pos_t** _lf_target_pos_pointers;
    reaction_t _lf__reaction_0;
    reaction_t _lf__reaction_1;
    reaction_t _lf__reaction_2;
    reaction_t _lf__reaction_3;
    trigger_t _lf__trigger_send;
    reaction_t* _lf__trigger_send_reactions[1];
    trigger_t _lf__trigger_read;
    reaction_t* _lf__trigger_read_reactions[1];
    trigger_t _lf__startup;
    reaction_t* _lf__startup_reactions[1];
    trigger_t _lf__current_pos;
    reaction_t* _lf__current_pos_reactions[1];
    #ifdef FEDERATED
    trigger_t* _lf__current_pos_network_port_status;
    
    #endif // FEDERATED
    trigger_t _lf__sea_pos;
    #ifdef FEDERATED
    trigger_t* _lf__sea_pos_network_port_status;
    
    #endif // FEDERATED
} _ros_interface_self_t;
_ros_interface_self_t* new__ros_interface();
#endif // _ROS_INTERFACE_H
