#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/FFB/FFBController.h"
#include "_ffbcontroller.h"
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _ffbcontrollerreaction_function_0(void* instance_args) {
    _ffbcontroller_self_t* self = (_ffbcontroller_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _ffbcontroller_target_pos_t* target_pos = self->_lf_target_pos;
    int target_pos_width = self->_lf_target_pos_width; SUPPRESS_UNUSED_WARNING(target_pos_width);
    _ffbcontroller_current_pos_t* current_pos = self->_lf_current_pos;
    int current_pos_width = self->_lf_current_pos_width; SUPPRESS_UNUSED_WARNING(current_pos_width);
    _ffbcontroller_sea_pos_t* sea_pos = self->_lf_sea_pos;
    int sea_pos_width = self->_lf_sea_pos_width; SUPPRESS_UNUSED_WARNING(sea_pos_width);
    _ffbcontroller_out_t* out = &self->_lf_out;
    #line 27 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Joint_Controllers/FFB.lf"
    float pid_out;
    
    float error_p = sea_pos->value;
    float sea_diff = (sea_pos->value - self->last_sea);
    self->last_sea = sea_pos->value;
    
    bool flag_p = (error_p > self->CHANGE_P || error_p < -self->CHANGE_P);
    bool flag_d = (sea_diff > self->CHANGE_D || sea_diff < -self->CHANGE_D);
    // FSM for control state.
    // 0: We are in the idle state
    // 1: We are moving
    switch(self->fsm_state){
      case 0: // For idle state
        pid_out = 0;
        self->fsm_state = flag_p || flag_d;
        break;
      case 1: // For movement state
        pid_out = self->Kp * error_p;
        self->fsm_state = (error_p > self->LIM || error_p < -self->LIM);
        break;
    
    }
    
    pid_out = pid_out < -100? -100: pid_out;
    pid_out = pid_out > 100? 100: pid_out;
    
    // printf("error: %d, out: %d\r\n", (int)(error_p * 100), (int)pid_out);
    lf_set(out, pid_out);
#line 47 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ffbcontroller.c"
}
#include "include/api/reaction_macros_undef.h"
_ffbcontroller_self_t* new__ffbcontroller() {
    _ffbcontroller_self_t* self = (_ffbcontroller_self_t*)lf_new_reactor(sizeof(_ffbcontroller_self_t));
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
    self->_lf__reaction_0.function = _ffbcontrollerreaction_function_0;
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
