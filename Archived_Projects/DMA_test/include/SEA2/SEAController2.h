#ifndef _seacontroller2_H
#define _seacontroller2_H
#ifndef _SEACONTROLLER2_H // necessary for arduino-cli, which automatically includes headers that are not used
#ifndef TOP_LEVEL_PREAMBLE_165137662_H
#define TOP_LEVEL_PREAMBLE_165137662_H
/*Correspondence: Range: [(12, 2), (13, 18)) -> Range: [(0, 0), (1, 18)) (verbatim=true; src=/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/SEA2.lf)*/#include "stm32f4xx_hal.h"
#include <stdio.h>
/*Correspondence: Range: [(9, 2), (10, 18)) -> Range: [(0, 0), (1, 18)) (verbatim=true; src=/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/PID.lf)*/#include "stm32f4xx_hal.h"
#include <stdio.h>
/*Correspondence: Range: [(9, 2), (10, 18)) -> Range: [(0, 0), (1, 18)) (verbatim=true; src=/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf)*/#include "stm32f4xx_hal.h"
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
typedef struct seacontroller2_self_t{
    self_base_t base; // This field is only to be used by the runtime, not the user.
    float Kp;
    float Ki;
    float Kd;
    float Ks;
    float SEA_LIM_P;
    float SEA_LIM_D;
    float LIM;
    float CHANGE_P;
    float CHANGE_D;
    int fsm_state;
    float last_target;
    float last_sea;
    float last_out;
    int cycle_count;
    bool print_data;
    int end[0]; // placeholder; MSVC does not compile empty structs
} seacontroller2_self_t;
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
} seacontroller2_current_pos_t;
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
} seacontroller2_sea_pos_t;
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
} seacontroller2_target_pos_t;
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
} seacontroller2_out_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} ffbcontroller_current_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} ffbcontroller_sea_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} ffbcontroller_target_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} ffbcontroller_out_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} pidcontroller_current_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} pidcontroller_sea_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} pidcontroller_target_pos_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    float value;

} pidcontroller_out_t;
#endif
#endif
