#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/Motor_Driver/MotorDriver.h"
#include "_motordriver.h"
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _motordriverreaction_function_0(void* instance_args) {
    _motordriver_self_t* self = (_motordriver_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    struct control_u0 {
        _seacontroller0_out_t* out;
    
    } control_u0;
    struct control_u1 {
        _seacontroller0_out_t* out;
    
    } control_u1;
    struct control_u2 {
        _seacontroller0_out_t* out;
    
    } control_u2;
    struct control_u3 {
        _seacontroller1_out_t* out;
    
    } control_u3;
    struct control_u4 {
        _seacontroller1_out_t* out;
    
    } control_u4;
    struct control_u5 {
        _seacontroller2_out_t* out;
    
    } control_u5;
    struct control_u6 {
        _seacontroller2_out_t* out;
    
    } control_u6;
    control_u0.out = self->_lf_control_u0.out;
    control_u1.out = self->_lf_control_u1.out;
    control_u2.out = self->_lf_control_u2.out;
    control_u3.out = self->_lf_control_u3.out;
    control_u4.out = self->_lf_control_u4.out;
    control_u5.out = self->_lf_control_u5.out;
    control_u6.out = self->_lf_control_u6.out;
    #line 103 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Driver.lf"
    self -> speed_posc[0] = control_u0.out->value;
    self -> speed_posc[1] = control_u1.out->value;
    self -> speed_posc[2] = control_u2.out->value;
    self -> speed_posc[3] = control_u3.out->value;
    self -> speed_posc[4] = control_u4.out->value;
    self -> speed_posc[5] = control_u5.out->value;
    self -> speed_posc[6] = control_u6.out->value;
#line 54 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_motordriver.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _motordriverreaction_function_1(void* instance_args) {
    _motordriver_self_t* self = (_motordriver_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _motordriver_target_sel_t** target_sel = self->_lf_target_sel;
    int target_sel_width = self->_lf_target_sel_width; SUPPRESS_UNUSED_WARNING(target_sel_width);
    _motordriver_target_speed_t** target_speed = self->_lf_target_speed;
    int target_speed_width = self->_lf_target_speed_width; SUPPRESS_UNUSED_WARNING(target_speed_width);
    #line 114 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Driver.lf"
    for(int i=0; i<7; i++) {
      self->sel[i] = target_sel[i]->value;
      self->speed_spc[i] = target_speed[i]->value;
    }
#line 69 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_motordriver.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _motordriverreaction_function_2(void* instance_args) {
    _motordriver_self_t* self = (_motordriver_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    struct usm {
        _usm_speed_t** speed;
    int speed_width;
    
    } usm;
    usm.speed = self->_lf_usm.speed;
    usm.speed_width = self->_lf_usm.speed_width;
    #line 121 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Driver.lf"
    // We use this to set either the speed or position of the motors
    // sel is used to determine which --> 0: speed control, 1: position control
    lf_set(usm.speed[0], self->sel[0]? self->speed_posc[0]: self->speed_spc[0]);
    lf_set(usm.speed[1], self->sel[1]? self->speed_posc[1]: self->speed_spc[1]);
    lf_set(usm.speed[2], self->sel[2]? self->speed_posc[2]: self->speed_spc[2]);
    lf_set(usm.speed[3], self->sel[3]? self->speed_posc[3]: self->speed_spc[3]);
    lf_set(usm.speed[4], self->sel[4]? self->speed_posc[4]: self->speed_spc[4]);
    lf_set(usm.speed[5], self->sel[5]? self->speed_posc[5]: self->speed_spc[5]);
    lf_set(usm.speed[6], self->sel[6]? self->speed_posc[6]: self->speed_spc[6]);
#line 92 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_motordriver.c"
}
#include "include/api/reaction_macros_undef.h"
_motordriver_self_t* new__motordriver() {
    _motordriver_self_t* self = (_motordriver_self_t*)lf_new_reactor(sizeof(_motordriver_self_t));
    // Set the default source reactor pointer
    self->_lf_default__target_sel._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__target_speed._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__target_pos._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__qdec_current._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__qdec_sea._base.source_reactor = (self_base_t*)self;
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u0_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u0.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u0.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u0.out_trigger.reactions = self->_lf_control_u0.out_reactions;
    self->_lf_control_u0.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u0.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u0.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u1_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u1.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u1.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u1.out_trigger.reactions = self->_lf_control_u1.out_reactions;
    self->_lf_control_u1.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u1.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u1.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u2_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u2.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u2.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u2.out_trigger.reactions = self->_lf_control_u2.out_reactions;
    self->_lf_control_u2.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u2.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u2.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u3_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u3.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u3.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u3.out_trigger.reactions = self->_lf_control_u3.out_reactions;
    self->_lf_control_u3.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u3.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u3.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u4_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u4.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u4.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u4.out_trigger.reactions = self->_lf_control_u4.out_reactions;
    self->_lf_control_u4.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u4.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u4.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u5_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u5.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u5.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u5.out_trigger.reactions = self->_lf_control_u5.out_reactions;
    self->_lf_control_u5.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u5.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u5.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_control_u6_width = -2;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf_control_u6.out_trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf_control_u6.out_reactions[0] = &self->_lf__reaction_0;
    self->_lf_control_u6.out_trigger.reactions = self->_lf_control_u6.out_reactions;
    self->_lf_control_u6.out_trigger.last_tag = NEVER_TAG;
    self->_lf_control_u6.out_trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf_control_u6.out_trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    // Set the _width variable for all cases. This will be -2
    // if the reactor is not a bank of reactors.
    self->_lf_usm_width = -2;
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _motordriverreaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _motordriverreaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__reaction_2.number = 2;
    self->_lf__reaction_2.function = _motordriverreaction_function_2;
    self->_lf__reaction_2.self = self;
    self->_lf__reaction_2.deadline_violation_handler = NULL;
    self->_lf__reaction_2.STP_handler = NULL;
    self->_lf__reaction_2.name = "?";
    self->_lf__reaction_2.mode = NULL;
    self->_lf__trigger.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger_reactions[0] = &self->_lf__reaction_2;
    self->_lf__trigger.reactions = &self->_lf__trigger_reactions[0];
    self->_lf__trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__trigger.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__target_sel.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__target_sel.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__target_sel_reactions[0] = &self->_lf__reaction_1;
    self->_lf__target_sel.reactions = &self->_lf__target_sel_reactions[0];
    self->_lf__target_sel.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__target_sel.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__target_sel.tmplt.type.element_size = sizeof(bool);
    self->_lf__target_speed.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__target_speed.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__target_speed_reactions[0] = &self->_lf__reaction_1;
    self->_lf__target_speed.reactions = &self->_lf__target_speed_reactions[0];
    self->_lf__target_speed.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__target_speed.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__target_speed.tmplt.type.element_size = sizeof(float);
    self->_lf__target_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__target_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__target_pos.tmplt.type.element_size = sizeof(float);
    self->_lf__qdec_current.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__qdec_current.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__qdec_current.tmplt.type.element_size = sizeof(float);
    self->_lf__qdec_sea.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__qdec_sea.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__qdec_sea.tmplt.type.element_size = sizeof(float);
    return self;
}
