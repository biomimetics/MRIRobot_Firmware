#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/USM/USM.h"
#include "_usm.h"
// *********** From the preamble, verbatim:
#line 23 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Drivers/USM.lf"
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

int MAX_SPEED[7]        =    {500, 500, 500, 500, 500, 500, 500};
float SPEED_SCALE[7]    =    {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
float _speed[7]         =    {0, 0, 0, 0, 0, 0, 0};

float motor_dir[7]=   {1, 1, 1, -1, -1, -1, -1};

float USM_LIMIT = 0.06;
volatile int en_usm;

int state = 0;

static void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  ;HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim4);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_MspPostInit(&htim4);

}

static void USM_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  //+----------------------------------------------------------------+
  //  Name,     PWM     Direction,    Enable
  //  USM_0,    PB4,    PC13,         PB1
  //  USM_1,    PB5,    PC14,         PB2
  //  USM_2,    PC8,    PC15,         PB3
  //  USM_3,    PC9,    PC4,          PB9
  //  USM_4,    PB6,    PC5,          PB10
  //  USM_5,    PB7,    PB15,         PB12
  //  USM_6,    PB8,    PB14,         PB13
  //+----------------------------------------------------------------+

  // Write reset value pins


  // Configure output pins (DIR/EN)
  HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10|
    GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_9|GPIO_PIN_10|
                        GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC,  GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure input pins (ON/OFF Switch) PC0
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure output pin (Status LED)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static int min(int a, int b) {
  return a<b? a: b;
}

static int max(int a, int b) {
  return a>b? a: b;
}

static int conv_speed(float speed, int idx) {
  float USM_MIN_SPEED = 3;
  if (speed < 0)
    return  min((int)((1000 * max(USM_MIN_SPEED, -speed) * SPEED_SCALE[idx])/100), MAX_SPEED[idx]);
  else
    return  min((int)((1000 * max(USM_MIN_SPEED, speed) * SPEED_SCALE[idx])/100), MAX_SPEED[idx]);
}
#line 161 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.c"

// *********** End of preamble.
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _usmreaction_function_0(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 181 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Drivers/USM.lf"
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    
    MX_TIM3_Init();
    MX_TIM4_Init();
    USM_GPIO_Init();
    
    // Initially set all enable values to 0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);    // for USM0
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);    // for USM1
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);    // for USM2
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);    // for USM3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);   // for USM4
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);   // for USM5
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);   // for USM5
    
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // USM_0, PB4
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // USM_1, PB5
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   // USM_2, PC8
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   // USM_3, PC9
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);   // USM_4, PB6
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);   // USM_5, PB7
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);   // USM_5, PB8
#line 196 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _usmreaction_function_1(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _usm_speed_t** speed = self->_lf_speed;
    int speed_width = self->_lf_speed_width; SUPPRESS_UNUSED_WARNING(speed_width);
    #line 209 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Drivers/USM.lf"
    en_usm = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, en_usm);
    
    for (int i=0; i<7; i++) {
      _speed[i] = en_usm? speed[i]->value * motor_dir[i] : 0;
    }
#line 211 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _usmreaction_function_2(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 218 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Drivers/USM.lf"
    // Set USM direction values
    // For some reason, setting the motor direction doesnt work unless we add a small delay in between
    // Update: It seems like moving the direction set above the enable pins resolves this issue
    //      Though I am still not sure why this is happening
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,   _speed[0] >= 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,    _speed[1] >= 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,    _speed[2] >= 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,    _speed[3] >= 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,    _speed[4] >= 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,   _speed[5] >= 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,   _speed[6] >= 0);
#line 230 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _usmreaction_function_3(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 234 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Drivers/USM.lf"
    // Sometimes it doesnt properly register if we dont add a delay here. 
    //    I think its an issue with the motor controller getting too many commands at the same time
    // HAL_Delay(1);
    TIM3->CCR1 = conv_speed(_speed[0], 0);  // Set period for USM0
    TIM3->CCR2 = conv_speed(_speed[1], 1);  // Set period for USM1
    TIM3->CCR3 = conv_speed(_speed[2], 2);  // Set period for USM2
    TIM3->CCR4 = conv_speed(_speed[3], 3);  // Set period for USM3
    TIM4->CCR1 = conv_speed(_speed[4], 4);  // Set period for USM4
    TIM4->CCR2 = conv_speed(_speed[5], 5);  // Set period for USM5
    TIM4->CCR3 = conv_speed(_speed[6], 6);  // Set period for USM6
#line 248 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _usmreaction_function_4(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 248 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/Motor_Drivers/USM.lf"
    // Set USM enable values
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,    _speed[0] > USM_LIMIT || _speed[0] < -USM_LIMIT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,    _speed[1] > USM_LIMIT || _speed[1] < -USM_LIMIT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,    _speed[2] > USM_LIMIT || _speed[2] < -USM_LIMIT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,    _speed[3] > USM_LIMIT || _speed[3] < -USM_LIMIT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,   _speed[4] > USM_LIMIT || _speed[4] < -USM_LIMIT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,   _speed[5] > USM_LIMIT || _speed[5] < -USM_LIMIT);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,   _speed[6] > USM_LIMIT || _speed[6] < -USM_LIMIT);
#line 264 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
_usm_self_t* new__usm() {
    _usm_self_t* self = (_usm_self_t*)lf_new_reactor(sizeof(_usm_self_t));
    // Set the default source reactor pointer
    self->_lf_default__speed._base.source_reactor = (self_base_t*)self;
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _usmreaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _usmreaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__reaction_2.number = 2;
    self->_lf__reaction_2.function = _usmreaction_function_2;
    self->_lf__reaction_2.self = self;
    self->_lf__reaction_2.deadline_violation_handler = NULL;
    self->_lf__reaction_2.STP_handler = NULL;
    self->_lf__reaction_2.name = "?";
    self->_lf__reaction_2.mode = NULL;
    self->_lf__reaction_3.number = 3;
    self->_lf__reaction_3.function = _usmreaction_function_3;
    self->_lf__reaction_3.self = self;
    self->_lf__reaction_3.deadline_violation_handler = NULL;
    self->_lf__reaction_3.STP_handler = NULL;
    self->_lf__reaction_3.name = "?";
    self->_lf__reaction_3.mode = NULL;
    self->_lf__reaction_4.number = 4;
    self->_lf__reaction_4.function = _usmreaction_function_4;
    self->_lf__reaction_4.self = self;
    self->_lf__reaction_4.deadline_violation_handler = NULL;
    self->_lf__reaction_4.STP_handler = NULL;
    self->_lf__reaction_4.name = "?";
    self->_lf__reaction_4.mode = NULL;
    self->_lf__motor_tick0.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__motor_tick0.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__motor_tick0_reactions[0] = &self->_lf__reaction_2;
    self->_lf__motor_tick0.reactions = &self->_lf__motor_tick0_reactions[0];
    self->_lf__motor_tick0.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__motor_tick0.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__motor_tick0.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__motor_tick0.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__motor_tick1.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__motor_tick1.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__motor_tick1_reactions[0] = &self->_lf__reaction_3;
    self->_lf__motor_tick1.reactions = &self->_lf__motor_tick1_reactions[0];
    self->_lf__motor_tick1.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__motor_tick1.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__motor_tick1.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__motor_tick1.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__motor_tick2.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__motor_tick2.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__motor_tick2_reactions[0] = &self->_lf__reaction_4;
    self->_lf__motor_tick2.reactions = &self->_lf__motor_tick2_reactions[0];
    self->_lf__motor_tick2.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__motor_tick2.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__motor_tick2.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__motor_tick2.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__startup.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__startup_reactions[0] = &self->_lf__reaction_0;
    self->_lf__startup.last_tag = NEVER_TAG;
    self->_lf__startup.reactions = &self->_lf__startup_reactions[0];
    self->_lf__startup.number_of_reactions = 1;
    self->_lf__startup.is_timer = false;
    self->_lf__speed.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__speed.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__speed_reactions[0] = &self->_lf__reaction_1;
    self->_lf__speed.reactions = &self->_lf__speed_reactions[0];
    self->_lf__speed.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__speed.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__speed.tmplt.type.element_size = sizeof(float);
    return self;
}
