#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/SEA5/SEAController5.h"
#include "_seacontroller5.h"
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _seacontroller5reaction_function_0(void* instance_args) {
    _seacontroller5_self_t* self = (_seacontroller5_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _seacontroller5_sea_pos_t* sea_pos = self->_lf_sea_pos;
    int sea_pos_width = self->_lf_sea_pos_width; SUPPRESS_UNUSED_WARNING(sea_pos_width);
    _seacontroller5_target_pos_t* target_pos = self->_lf_target_pos;
    int target_pos_width = self->_lf_target_pos_width; SUPPRESS_UNUSED_WARNING(target_pos_width);
    #line 55 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA5.lf"
    float D_LIM = 0.005;      // Force difference limit
    float TRANS_CYCLES = 70;  // Number of cycles we stay in thetransient state
    
    
    float sea_diff = (sea_pos->value - self->last_sea);
    float sea_error = sea_pos->value;
    self->last_sea = sea_pos->value;
    
    bool pos_flag = (sea_error > self->SEA_LIM || sea_error < -self->SEA_LIM);
    bool change_flag = (sea_diff > D_LIM || sea_diff < -D_LIM);
    
    if (self->last_target != target_pos->value) {
      self->fsm_state = 2;
      self->cycle_count = 0;
    } else  if(self->fsm_state == 1) {       // For normal PID movement
      if (change_flag) {
        if(self -> print_data) {
          printf("ch_f %d\n\r", (int)(sea_diff * 10000));
        }
        self->fsm_state = 0; // We got to FFB state
      } else if(pos_flag) {
        if(self -> print_data) {
          printf("pos_f %d\n\r", (int)(sea_error * 10000));
        }
        self->fsm_state = 0; // We got to FFB state
      }
    } else if(self->fsm_state == 2) {       // For initial movement transience
      self->cycle_count++;
      if (pos_flag) {
        if(self -> print_data) {
          printf("tr_pos: %d\n\r", (int)(sea_error * 1000));
        }
        self->fsm_state = 0; // We go to FFB state
      } else if(self->cycle_count >= TRANS_CYCLES) {
        self->fsm_state = 1; // We got to standard PID state
      }
    }
    self->last_target = target_pos->value;
#line 54 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_seacontroller5.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _seacontroller5reaction_function_1(void* instance_args) {
    _seacontroller5_self_t* self = (_seacontroller5_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    struct force_control {
        _ffbcontroller_out_t* out;
    
    } force_control;
    struct pos_control {
        _pidcontroller_out_t* out;
    
    } pos_control;
    force_control.out = self->_lf_force_control.out;
    pos_control.out = self->_lf_pos_control.out;
    _seacontroller5_out_t* out = &self->_lf_out;
    #line 96 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Joint_Controllers/SEA5.lf"
    float RISE_CAP = 1;
    float FALL_CAP = 5;
    if (self->fsm_state) {
    
      float out_diff = pos_control.out->value - self->last_out;
      float out_val = pos_control.out->value;
    
      if (out_diff > RISE_CAP) {
        out_val = self->last_out + RISE_CAP;
      }
      if (out_diff < -FALL_CAP) {
        out_val = self->last_out - FALL_CAP;
      }
      lf_set(out, out_val);
      self->last_out = out_val;
    
    } else {
      lf_set(out, force_control.out->value);
      self->last_out = 0;
    }
#line 92 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_seacontroller5.c"
}
#include "include/api/reaction_macros_undef.h"
_seacontroller5_self_t* new__seacontroller5() {
    _seacontroller5_self_t* self = (_seacontroller5_self_t*)lf_new_reactor(sizeof(_seacontroller5_self_t));
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
    self->_lf__reaction_0.function = _seacontroller5reaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _seacontroller5reaction_function_1;
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
