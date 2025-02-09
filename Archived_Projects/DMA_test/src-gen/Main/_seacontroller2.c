#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/SEA2/SEAController2.h"
#include "_seacontroller2.h"
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _seacontroller2reaction_function_0(void* instance_args) {
    _seacontroller2_self_t* self = (_seacontroller2_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _seacontroller2_sea_pos_t* sea_pos = self->_lf_sea_pos;
    int sea_pos_width = self->_lf_sea_pos_width; SUPPRESS_UNUSED_WARNING(sea_pos_width);
    _seacontroller2_target_pos_t* target_pos = self->_lf_target_pos;
    int target_pos_width = self->_lf_target_pos_width; SUPPRESS_UNUSED_WARNING(target_pos_width);
    #line 64 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/SEA2.lf"
    float TRANS_CYCLES = 0;  // Number of cycles we stay in transient state
    
    // Any necessary operations we have here
    float sea_diff = (sea_pos->value - self->last_sea);
    float sea_error = sea_pos->value;
    self->last_sea = sea_pos->value;
    
    bool flag_p = (sea_error > self->SEA_LIM_P || sea_error < -self->SEA_LIM_P);
    bool flag_d = (sea_diff > self->SEA_LIM_D || sea_diff < -self->SEA_LIM_D);
    
    if (self->last_target != target_pos->value) {
      self->fsm_state = 0;
      self->cycle_count = 0;
    }
    
    /* SEA finite state machine:
        0: PID movement state
        1: Movement transient state (initial movement from new command)
        2: Force feedback state for when we encounter an obstacle */
    switch(self->fsm_state){
      case 0: // For PID state
        if (flag_p || flag_d) {
          self->fsm_state = 2; // We got to FFB state
          printf("triggered [%d, %d]\r\n", flag_p, (int)(sea_error*10000));
        }
        break;
    
      case 1: // For initial movement transient state  
        self->cycle_count++;
        if (flag_p){
          self->fsm_state = 2; // We go to FFB state
        } else if(self->cycle_count >= TRANS_CYCLES)
          self->fsm_state = 0; // We got to standard PID state
        break;
    
      case 2: // For Force Feedback state 
        // We stay in this state until we get a new command
        break;      
      }
    self->last_target = target_pos->value;
#line 56 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_seacontroller2.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _seacontroller2reaction_function_1(void* instance_args) {
    _seacontroller2_self_t* self = (_seacontroller2_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    struct force_control {
        _ffbcontroller_out_t* out;
    
    } force_control;
    struct pos_control {
        _pidcontroller_out_t* out;
    
    } pos_control;
    force_control.out = self->_lf_force_control.out;
    pos_control.out = self->_lf_pos_control.out;
    _seacontroller2_out_t* out = &self->_lf_out;
    #line 107 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/SEA2.lf"
    // Caps to decrease the rate of rise and fall
    float RISE_CAP = 20;
    float FALL_CAP = 20;
    
    switch(self->fsm_state){
      case 0: // For PID state
        float out_diff = pos_control.out->value - self->last_out;
        float out_val = pos_control.out->value;
    
        // Apply the rise/fall caps
        if (out_diff > RISE_CAP)
          out_val = self->last_out + RISE_CAP;
        if (out_diff < -FALL_CAP)
          out_val = self->last_out - FALL_CAP;
    
        lf_set(out, out_val);
        self->last_out = out_val;
        break;
    
      case 2: // For Force Feedback state 
        lf_set(out, force_control.out->value);
        self->last_out = 0;
        break;
    }
#line 98 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_seacontroller2.c"
}
#include "include/api/reaction_macros_undef.h"
_seacontroller2_self_t* new__seacontroller2() {
    _seacontroller2_self_t* self = (_seacontroller2_self_t*)lf_new_reactor(sizeof(_seacontroller2_self_t));
    // Set input by default to an always absent default input.
    self->_lf_current_pos = &self->_lf_default__current_pos;
    // Set the default source reactor pointer
    self->_lf_default__current_pos._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_sea_pos = &self->_lf_default__sea_pos;
    // Set the default source reactor pointer
    self->_lf_default__sea_pos._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_target_pos = &self->_lf_default__target_pos;
    // Set the default source reactor pointer
    self->_lf_default__target_pos._base.source_reactor = (self_base_t*)self;
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_force_control_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_force_control.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_force_control.out_reactions[0] = &self->_lf__reaction_1;
    self->_lf_force_control.out_trigger.reactions = self->_lf_force_control.out_reactions;
    self->_lf_force_control.out_trigger.last_tag = NEVER_TAG;
    self->_lf_force_control.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_force_control.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_pos_control_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_pos_control.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_pos_control.out_reactions[0] = &self->_lf__reaction_1;
    self->_lf_pos_control.out_trigger.reactions = self->_lf_pos_control.out_reactions;
    self->_lf_pos_control.out_trigger.last_tag = NEVER_TAG;
    self->_lf_pos_control.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_pos_control.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _seacontroller2reaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _seacontroller2reaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__current_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__current_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__current_pos.tmplt.type.element_size = sizeof(float);
    self->_lf__sea_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__sea_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__sea_pos_reactions[0] = &self->_lf__reaction_0;
    self->_lf__sea_pos.reactions = &self->_lf__sea_pos_reactions[0];
    self->_lf__sea_pos.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__sea_pos.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__sea_pos.tmplt.type.element_size = sizeof(float);
    self->_lf__target_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__target_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__target_pos_reactions[0] = &self->_lf__reaction_0;
    self->_lf__target_pos.reactions = &self->_lf__target_pos_reactions[0];
    self->_lf__target_pos.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__target_pos.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__target_pos.tmplt.type.element_size = sizeof(float);
    return self;
}
