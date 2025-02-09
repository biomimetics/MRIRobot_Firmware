#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/Switches/Switches.h"
#include "_switches.h"
// *********** From the preamble, verbatim:
#line 20 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Switches.lf"
static void Switch_GPIO_Init(void) {
      GPIO_InitTypeDef GPIO_InitStruct = {0};

      __HAL_RCC_GPIOA_CLK_ENABLE();

      GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
#line 18 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_switches.c"

// *********** End of preamble.
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _switchesreaction_function_0(void* instance_args) {
    _switches_self_t* self = (_switches_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 33 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Switches.lf"
    Switch_GPIO_Init();
#line 29 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_switches.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _switchesreaction_function_1(void* instance_args) {
    _switches_self_t* self = (_switches_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    int read_width = self->_lf_read_width; SUPPRESS_UNUSED_WARNING(read_width);
    _switches_read_t** read = self->_lf_read_pointers;
    #line 38 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Switches.lf"
    lf_set(read[0], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4));
    lf_set(read[1], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
    lf_set(read[2], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
    lf_set(read[3], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7));
    lf_set(read[4], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));
    lf_set(read[5], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11));
    lf_set(read[6], HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12));
    // lf_set(read[5], HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4));
#line 46 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_switches.c"
}
#include "include/api/reaction_macros_undef.h"
_switches_self_t* new__switches() {
    _switches_self_t* self = (_switches_self_t*)lf_new_reactor(sizeof(_switches_self_t));
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _switchesreaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _switchesreaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__trigger.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger_reactions[0] = &self->_lf__reaction_1;
    self->_lf__trigger.reactions = &self->_lf__trigger_reactions[0];
    self->_lf__trigger.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__trigger.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__trigger.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__startup.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__startup_reactions[0] = &self->_lf__reaction_0;
    self->_lf__startup.last_tag = NEVER_TAG;
    self->_lf__startup.reactions = &self->_lf__startup_reactions[0];
    self->_lf__startup.number_of_reactions = 1;
    self->_lf__startup.is_timer = false;
    return self;
}
