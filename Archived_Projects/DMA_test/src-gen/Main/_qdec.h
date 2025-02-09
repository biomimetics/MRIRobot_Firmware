#ifndef _QDEC_H
#define _QDEC_H
#include "include/core/reactor.h"
#ifndef TOP_LEVEL_PREAMBLE_1778801068_H
#define TOP_LEVEL_PREAMBLE_1778801068_H
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
} _qdec_reset_qdec_t;
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
} _qdec_qdec_out_t;
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
} _qdec_sea_out_t;
typedef struct {
    struct self_base_t base;
    #line 14 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Encoder.lf"
    interval_t sample_period;
#line 56 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_qdec.h"
#line 57 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_qdec.h"
    _qdec_reset_qdec_t* _lf_reset_qdec;
    // width of -2 indicates that it is not a multiport.
    int _lf_reset_qdec_width;
    // Default input (in case it does not get connected)
    _qdec_reset_qdec_t _lf_default__reset_qdec;
    // Array of output ports.
    _qdec_qdec_out_t* _lf_qdec_out;
    int _lf_qdec_out_width;
    // An array of pointers to the individual ports. Useful
    // for the lf_set macros to work out-of-the-box for
    // multiports in the body of reactions because their 
    // value can be accessed via a -> operator (e.g.,foo[i]->value).
    // So we have to handle multiports specially here a construct that
    // array of pointers.
    _qdec_qdec_out_t** _lf_qdec_out_pointers;
    // Array of output ports.
    _qdec_sea_out_t* _lf_sea_out;
    int _lf_sea_out_width;
    // An array of pointers to the individual ports. Useful
    // for the lf_set macros to work out-of-the-box for
    // multiports in the body of reactions because their 
    // value can be accessed via a -> operator (e.g.,foo[i]->value).
    // So we have to handle multiports specially here a construct that
    // array of pointers.
    _qdec_sea_out_t** _lf_sea_out_pointers;
    reaction_t _lf__reaction_0;
    reaction_t _lf__reaction_1;
    reaction_t _lf__reaction_2;
    reaction_t _lf__reaction_3;
    trigger_t _lf__trigger;
    reaction_t* _lf__trigger_reactions[1];
    trigger_t _lf__enable;
    reaction_t* _lf__enable_reactions[1];
    trigger_t _lf__startup;
    reaction_t* _lf__startup_reactions[1];
    trigger_t _lf__reset_qdec;
    reaction_t* _lf__reset_qdec_reactions[1];
    #ifdef FEDERATED
    
    #endif // FEDERATED
} _qdec_self_t;
_qdec_self_t* new__qdec();
#endif // _QDEC_H
