#ifndef _SWITCHES_H
#define _SWITCHES_H
#include "include/core/reactor.h"
#ifndef TOP_LEVEL_PREAMBLE_173197870_H
#define TOP_LEVEL_PREAMBLE_173197870_H
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
    bool value;
    #ifdef FEDERATED
    #ifdef FEDERATED_DECENTRALIZED
    tag_t intended_tag;
    #endif
    interval_t physical_time_of_arrival;
    #endif
} _switches_read_t;
typedef struct {
    struct self_base_t base;
    #line 15 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Switches.lf"
    interval_t sample_period;
#line 29 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_switches.h"
#line 30 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_switches.h"
    // Array of output ports.
    _switches_read_t* _lf_read;
    int _lf_read_width;
    // An array of pointers to the individual ports. Useful
    // for the lf_set macros to work out-of-the-box for
    // multiports in the body of reactions because their 
    // value can be accessed via a -> operator (e.g.,foo[i]->value).
    // So we have to handle multiports specially here a construct that
    // array of pointers.
    _switches_read_t** _lf_read_pointers;
    reaction_t _lf__reaction_0;
    reaction_t _lf__reaction_1;
    trigger_t _lf__trigger;
    reaction_t* _lf__trigger_reactions[1];
    trigger_t _lf__startup;
    reaction_t* _lf__startup_reactions[1];
} _switches_self_t;
_switches_self_t* new__switches();
#endif // _SWITCHES_H
