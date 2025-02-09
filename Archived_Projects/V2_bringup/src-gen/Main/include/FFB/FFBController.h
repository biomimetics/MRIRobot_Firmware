#ifndef _ffbcontroller_H
#define _ffbcontroller_H
#ifndef _FFBCONTROLLER_H // necessary for arduino-cli, which automatically includes headers that are not used
#ifndef TOP_LEVEL_PREAMBLE_1413871034_H
#define TOP_LEVEL_PREAMBLE_1413871034_H
/*Correspondence: Range: [(9, 2), (10, 18)) -> Range: [(0, 0), (1, 18)) (verbatim=true; src=/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/FFB.lf)*/#include "stm32f4xx_hal.h"
#include <stdio.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif
#include "../include/api/schedule.h"
#include "../include/core/reactor.h"
#ifdef __cplusplus
}
#endif
typedef struct ffbcontroller_self_t{
    self_base_t base; // This field is only to be used by the runtime, not the user.
    float Ks;
    float SLIM;
    float D_LIM;
    int fsm_state;
    float last_sea;
    int end[0]; // placeholder; MSVC does not compile empty structs
} ffbcontroller_self_t;
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
} ffbcontroller_current_pos_t;
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
} ffbcontroller_sea_pos_t;
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
} ffbcontroller_target_pos_t;
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
} ffbcontroller_out_t;
#endif
#endif
