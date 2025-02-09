#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/Home/Home.h"
#include "_home.h"
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _homereaction_function_0(void* instance_args) {
    _home_self_t* self = (_home_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    struct homing_switches {
        _switches_read_t** read;
    int read_width;
    
    } homing_switches;
    _home_current_pos_t** current_pos = self->_lf_current_pos;
    int current_pos_width = self->_lf_current_pos_width; SUPPRESS_UNUSED_WARNING(current_pos_width);
    _home_sea_pos_t** sea_pos = self->_lf_sea_pos;
    int sea_pos_width = self->_lf_sea_pos_width; SUPPRESS_UNUSED_WARNING(sea_pos_width);
    homing_switches.read = self->_lf_homing_switches.read;
    homing_switches.read_width = self->_lf_homing_switches.read_width;
    int motor_speed_width = self->_lf_motor_speed_width; SUPPRESS_UNUSED_WARNING(motor_speed_width);
    _home_motor_speed_t** motor_speed = self->_lf_motor_speed_pointers;
    #line 41 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Home.lf"
    for(int i=0; i<7; i++) {
      lf_set(motor_speed[i], 0);
    }
#line 28 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_home.c"
}
#include "include/api/reaction_macros_undef.h"
_home_self_t* new__home() {
    _home_self_t* self = (_home_self_t*)lf_new_reactor(sizeof(_home_self_t));
    // Set the default source reactor pointer
    self->_lf_default__current_pos._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__sea_pos._base.source_reactor = (self_base_t*)self;
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_homing_switches_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_homing_switches.read_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_homing_switches.read_reactions[0] = &self->_lf__reaction_0;
    self->_lf_homing_switches.read_trigger.reactions = self->_lf_homing_switches.read_reactions;
    self->_lf_homing_switches.read_trigger.last_tag = NEVER_TAG;
    self->_lf_homing_switches.read_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_homing_switches.read_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _homereaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__trigger.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
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
    return self;
}
