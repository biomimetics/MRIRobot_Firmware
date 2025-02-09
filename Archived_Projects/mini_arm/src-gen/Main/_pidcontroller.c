#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/PID/PIDController.h"
#include "_pidcontroller.h"
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _pidcontrollerreaction_function_0(void* instance_args) {
    _pidcontroller_self_t* self = (_pidcontroller_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _pidcontroller_target_pos_t* target_pos = self->_lf_target_pos;
    int target_pos_width = self->_lf_target_pos_width; SUPPRESS_UNUSED_WARNING(target_pos_width);
    _pidcontroller_current_pos_t* current_pos = self->_lf_current_pos;
    int current_pos_width = self->_lf_current_pos_width; SUPPRESS_UNUSED_WARNING(current_pos_width);
    _pidcontroller_out_t* out = &self->_lf_out;
    #line 27 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Joint_Controllers/PID.lf"
    instant_t now = lf_time_logical();
    interval_t interval = now - self->prev_time;
    
    float error_p = (target_pos->value) - (current_pos->value);
    float error_d = ((current_pos->value) - self->last_pos) / interval;
    self->error_i = error_p + (self->Kw * self->error_i);
    
    float pid_out = self->Kp * error_p + self->Kd * error_d;
    // float pid_out = self->Kp * error_p;
    
    if(pid_out > 100) {
      pid_out = 100;
    } else if(pid_out < -100) {
      pid_out = -100;
    } 
    lf_set(out, pid_out);
    
    self->last_pos = current_pos->value;
    self->prev_time = now;
#line 36 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_pidcontroller.c"
}
#include "include/api/reaction_macros_undef.h"
_pidcontroller_self_t* new__pidcontroller() {
    _pidcontroller_self_t* self = (_pidcontroller_self_t*)lf_new_reactor(sizeof(_pidcontroller_self_t));
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
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _pidcontrollerreaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__current_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__current_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__current_pos_reactions[0] = &self->_lf__reaction_0;
    self->_lf__current_pos.reactions = &self->_lf__current_pos_reactions[0];
    self->_lf__current_pos.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__current_pos.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__current_pos.tmplt.type.element_size = sizeof(float);
    self->_lf__sea_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__sea_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
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
