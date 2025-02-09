#ifndef _home_H
#define _home_H
#ifndef _HOME_H // necessary for arduino-cli, which automatically includes headers that are not used
#ifndef TOP_LEVEL_PREAMBLE_1358169683_H
#define TOP_LEVEL_PREAMBLE_1358169683_H
/*Correspondence: Range: [(9, 2), (11, 18)) -> Range: [(0, 0), (2, 18)) (verbatim=true; src=/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/V3_bringup/src/lib/Home.lf)*/#include "stm32f4xx_hal.h"
#include "stm32_startup.h"
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
typedef struct home_self_t{
    self_base_t base; // This field is only to be used by the runtime, not the user.
    interval_t sample_period;
    float* home_speed;
    float* last_pos;
    float* last_sea;
    float* pos_diff;
    int* diffzero_cnt;
    int update_cnt;
    int end[0]; // placeholder; MSVC does not compile empty structs
} home_self_t;
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
} home_current_pos_t;
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
} home_sea_pos_t;
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
} home_motor_speed_t;
typedef struct {
    token_type_t type;
    lf_token_t* token;
    size_t length;
    bool is_present;
    lf_port_internal_t _base;
    int value;
    #ifdef FEDERATED
    #ifdef FEDERATED_DECENTRALIZED
    tag_t intended_tag;
    #endif
    interval_t physical_time_of_arrival;
    #endif
} home_is_home_t;
#endif
#endif
