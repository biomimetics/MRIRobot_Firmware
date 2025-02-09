#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/Home/Home.h"
#include "_home.h"
// *********** From the preamble, verbatim:
#line 41 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Home.lf"
int switch_val[7] = {0, 0, 0, 0, 0, 0, 0};
static int printVals = false;
static int printcnt = 0;
static int printcnt_lim = 20;

static int home_state = 0;
static int _is_home = 0;


static float ref_0 = 0;
static float ref_1 = 0;
static float ref_3 = 0;

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  //+----------------------------------------------------------------+
  //  Name,     
  //  home0,    PA4
  //  home1,    PA5
  //  home2,    PA6
  //  home3,    PA7
  //  home4,    PA8
  //  home5,    PA11
  //  home6,    PA12
  //+----------------------------------------------------------------+

  /* Configure Switch pins : PC1, PC2, PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static float get_speed(float curr, float target) {
  if (curr - target > 0.01) {
    return -5;
  } else if(curr - target < -0.01) {
    return 5;
  } else {
    return 0;
  }
}
#line 53 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_home.c"

// *********** End of preamble.
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _homereaction_function_0(void* instance_args) {
    _home_self_t* self = (_home_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 90 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Home.lf"
    switch_val[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
    switch_val[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
    switch_val[4] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    switch_val[5] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
    switch_val[6] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
    
    if (printVals && printcnt>=printcnt_lim) {
      printf("switch vals: %d, %d, %d, %d, %d, %d, %d\r\n", switch_val[0], switch_val[1], switch_val[2], switch_val[3], switch_val[4], switch_val[5], switch_val[6]);
    }
    printcnt = (printcnt >= printcnt_lim) ? 0 : printcnt+1;
#line 73 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_home.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _homereaction_function_1(void* instance_args) {
    _home_self_t* self = (_home_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _home_current_pos_t** current_pos = self->_lf_current_pos;
    int current_pos_width = self->_lf_current_pos_width; SUPPRESS_UNUSED_WARNING(current_pos_width);
    _home_sea_pos_t** sea_pos = self->_lf_sea_pos;
    int sea_pos_width = self->_lf_sea_pos_width; SUPPRESS_UNUSED_WARNING(sea_pos_width);
    int motor_speed_width = self->_lf_motor_speed_width; SUPPRESS_UNUSED_WARNING(motor_speed_width);
    _home_motor_speed_t** motor_speed = self->_lf_motor_speed_pointers;
    _home_is_home_t* is_home = &self->_lf_is_home;
    #line 104 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Home.lf"
    int spd = 5;
    float target_0 = 2.0944;
    float target_1 = 1.56;
    float target_3 = 0.34;
    
    if (_is_home == 0) {
      switch (home_state) {
        case 0:           // Go one direction until we hit the first switch
          self->home_speed[0] = 0;
          self->home_speed[5] = -spd;
          self->home_speed[4] = spd*0.8;
          if (switch_val[3] == 1) {
            ref_1 = current_pos[5]->value;
            printf("[HOME] going to state 1 - ref[%d]\r\n", (int)(ref_1*100));
            home_state = 1;
          }
          break;
    
    
        case 1:           // Go the second direction until we hit the switch
          self->home_speed[0] = 0;
          self->home_speed[5] = spd*0.8;
          self->home_speed[4] = -spd;
          if (switch_val[4] == 1) {
            ref_3 = current_pos[4]->value;
            printf("[HOME] going to state 2 - ref[%d]\r\n", (int)(ref_3*100));
            home_state = 2;
          }
          break;
    
    
        case 2:           // point forward to better home R axis
          self->home_speed[0] = 0;
          self->home_speed[5] = get_speed((current_pos[5]->value - ref_1), 1.5708);
          self->home_speed[4] = get_speed((current_pos[4]->value - ref_3), 1.4);
          if (self->home_speed[5] == 0 && self->home_speed[4] == 0) {
            printf("[HOME] going to state 3\r\n");
            home_state = 3;
          }
          break;
    
    
        case 3:           // home the R axis
          self->home_speed[0] = -spd;
          self->home_speed[5] = 0;
          self->home_speed[4] = 0;
          if (switch_val[5] == 1) {
            ref_0 = current_pos[0]->value;
            printf("[HOME] going to state 4 - ref[%d]\r\n", (int)(ref_0*100));
            home_state = 4;
          }
          break;
    
    
        case 4:           // Home all three joints to their targets
          self->home_speed[0] = get_speed((current_pos[0]->value - ref_0), target_0);
          self->home_speed[5] = 0;
          self->home_speed[4] = 0;
          if ((self->home_speed[0] == 0) && (self->home_speed[5] == 0) && (self->home_speed[4] == 0)) {
            printf("[HOME] going to state 5\r\n");
            home_state = 5;
          }
          break;
    
    
        case 5:           // homing is complete
          self->home_speed[0] = 0;
          self->home_speed[5] = get_speed((current_pos[5]->value - ref_1), target_1);
          self->home_speed[4] = get_speed((current_pos[4]->value - ref_3), target_3);
          if ((self->home_speed[0] == 0) && (self->home_speed[5] == 0) && (self->home_speed[4] == 0)) {
            printf("[HOME] going to state 6\r\n");
            home_state = 6;
          }
          break;
    
    
        case 6:           // homing is complete
          printf("[HOME] Homing complete\r\n");
          _is_home = 1;
          break;
    
    
        default:
          self->home_speed[0] = 0;
          self->home_speed[5] = 0;
          self->home_speed[4] = 0;
          _is_home = 1;
      }
    
    
    
    
    } else {
      home_state = 0;
      for(int i=0; i<7; i++) {
        self->home_speed[i] = 0;
      }
    }
    
    for(int i=0; i<7; i++) {
      lf_set(motor_speed[i], self->home_speed[i]);
    }
    
    
    
    
    lf_set(is_home, _is_home);
#line 194 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_home.c"
}
#include "include/api/reaction_macros_undef.h"
_home_self_t* new__home() {
    _home_self_t* self = (_home_self_t*)lf_new_reactor(sizeof(_home_self_t));
    // Set the default source reactor pointer
    self->_lf_default__current_pos._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__sea_pos._base.source_reactor = (self_base_t*)self;
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _homereaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _homereaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__trigger.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__switch_update.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__switch_update.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__switch_update_reactions[0] = &self->_lf__reaction_0;
    self->_lf__switch_update.reactions = &self->_lf__switch_update_reactions[0];
    self->_lf__switch_update.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__switch_update.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__switch_update.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__switch_update.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__current_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__current_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__current_pos_reactions[0] = &self->_lf__reaction_1;
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
    self->_lf__sea_pos_reactions[0] = &self->_lf__reaction_1;
    self->_lf__sea_pos.reactions = &self->_lf__sea_pos_reactions[0];
    self->_lf__sea_pos.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__sea_pos.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__sea_pos.tmplt.type.element_size = sizeof(float);
    return self;
}
