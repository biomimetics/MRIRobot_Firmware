#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/USM/USM.h"
#include "_usm.h"
// *********** From the preamble, verbatim:
#line 24 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Motor_Drivers/USM.lf"
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

int MAX_SPEED[7]        =    {500, 500, 500, 500, 500, 500, 500};
float SPEED_SCALE[7]    =    {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};


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
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    Error_Handler();

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
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
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
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
  int pwm_val;
  float USM_MIN_SPEED = 2.1;

  if (speed < 0)
    pwm_val =  min((int)((1000 * max(USM_MIN_SPEED, -speed) * SPEED_SCALE[idx])/100), MAX_SPEED[idx]);
  else
    pwm_val = min((int)((1000 * max(USM_MIN_SPEED, speed) * SPEED_SCALE[idx])/100), MAX_SPEED[idx]);
  return  pwm_val;
}
#line 178 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_usm.c"

// *********** End of preamble.
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _usmreaction_function_0(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 197 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Motor_Drivers/USM.lf"
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
#line 213 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _usmreaction_function_1(void* instance_args) {
    _usm_self_t* self = (_usm_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _usm_set_speed_0_t* set_speed_0 = self->_lf_set_speed_0;
    int set_speed_0_width = self->_lf_set_speed_0_width; SUPPRESS_UNUSED_WARNING(set_speed_0_width);
    _usm_set_speed_1_t* set_speed_1 = self->_lf_set_speed_1;
    int set_speed_1_width = self->_lf_set_speed_1_width; SUPPRESS_UNUSED_WARNING(set_speed_1_width);
    _usm_set_speed_2_t* set_speed_2 = self->_lf_set_speed_2;
    int set_speed_2_width = self->_lf_set_speed_2_width; SUPPRESS_UNUSED_WARNING(set_speed_2_width);
    _usm_set_speed_3_t* set_speed_3 = self->_lf_set_speed_3;
    int set_speed_3_width = self->_lf_set_speed_3_width; SUPPRESS_UNUSED_WARNING(set_speed_3_width);
    _usm_set_speed_4_t* set_speed_4 = self->_lf_set_speed_4;
    int set_speed_4_width = self->_lf_set_speed_4_width; SUPPRESS_UNUSED_WARNING(set_speed_4_width);
    _usm_set_speed_5_t* set_speed_5 = self->_lf_set_speed_5;
    int set_speed_5_width = self->_lf_set_speed_5_width; SUPPRESS_UNUSED_WARNING(set_speed_5_width);
    _usm_set_speed_6_t* set_speed_6 = self->_lf_set_speed_6;
    int set_speed_6_width = self->_lf_set_speed_6_width; SUPPRESS_UNUSED_WARNING(set_speed_6_width);
    #line 232 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src/lib/Motor_Drivers/USM.lf"
    // Set USM direction values
    // For some reason, setting the motor direction doesnt work unless we add a small delay in between
    // Update: It seems like moving the direction set above the enable pins resolves this issue
    //      Though I am still not sure why this is happening
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,   set_speed_0->value >= 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,   set_speed_1->value >= 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,   set_speed_2->value >= 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,   set_speed_1->value >= 0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,   set_speed_2->value >= 0);
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,    set_speed_3->value >= 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,    set_speed_4->value >= 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,   set_speed_5->value >= 0);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,   set_speed_5->value >= 0);
    
    int en_usm = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) && !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);;
    float USM_LIMIT = 0.06;
    
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, en_usm);
    
    // Set USM enable values
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,    (set_speed_0->value > USM_LIMIT || set_speed_0->value < -USM_LIMIT) && en_usm);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,    (set_speed_1->value > USM_LIMIT || set_speed_1->value < -USM_LIMIT) && en_usm);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,    (set_speed_2->value > USM_LIMIT || set_speed_2->value < -USM_LIMIT) && en_usm);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,    (set_speed_3->value > USM_LIMIT || set_speed_3->value < -USM_LIMIT) && en_usm);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,   (set_speed_4->value > USM_LIMIT || set_speed_4->value < -USM_LIMIT) && en_usm);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,   (set_speed_5->value > USM_LIMIT || set_speed_5->value < -USM_LIMIT) && en_usm);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,   (set_speed_6->value > USM_LIMIT || set_speed_6->value < -USM_LIMIT) && en_usm);
    
    TIM3->CCR1 = conv_speed(set_speed_0->value, 0);  // Set period for USM0
    TIM3->CCR2 = conv_speed(set_speed_1->value, 1);  // Set period for USM1
    TIM3->CCR3 = conv_speed(set_speed_2->value, 2);  // Set period for USM2
    TIM3->CCR4 = conv_speed(set_speed_3->value, 3);  // Set period for USM3
    TIM4->CCR1 = conv_speed(set_speed_4->value, 4);  // Set period for USM4
    TIM4->CCR2 = conv_speed(set_speed_5->value, 5);  // Set period for USM5
    TIM4->CCR3 = conv_speed(set_speed_6->value, 6);  // Set period for USM6
#line 271 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/mini_arm/src-gen/Main/_usm.c"
}
#include "include/api/reaction_macros_undef.h"
_usm_self_t* new__usm() {
    _usm_self_t* self = (_usm_self_t*)lf_new_reactor(sizeof(_usm_self_t));
    // Set input by default to an always absent default input.
    self->_lf_set_speed_0 = &self->_lf_default__set_speed_0;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_0._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_set_speed_1 = &self->_lf_default__set_speed_1;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_1._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_set_speed_2 = &self->_lf_default__set_speed_2;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_2._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_set_speed_3 = &self->_lf_default__set_speed_3;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_3._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_set_speed_4 = &self->_lf_default__set_speed_4;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_4._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_set_speed_5 = &self->_lf_default__set_speed_5;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_5._base.source_reactor = (self_base_t*)self;
    // Set input by default to an always absent default input.
    self->_lf_set_speed_6 = &self->_lf_default__set_speed_6;
    // Set the default source reactor pointer
    self->_lf_default__set_speed_6._base.source_reactor = (self_base_t*)self;
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
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__startup.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__startup_reactions[0] = &self->_lf__reaction_0;
    self->_lf__startup.last_tag = NEVER_TAG;
    self->_lf__startup.reactions = &self->_lf__startup_reactions[0];
    self->_lf__startup.number_of_reactions = 1;
    self->_lf__startup.is_timer = false;
    self->_lf__set_speed_0.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_0.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_0_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_0.reactions = &self->_lf__set_speed_0_reactions[0];
    self->_lf__set_speed_0.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_0.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_0.tmplt.type.element_size = sizeof(float);
    self->_lf__set_speed_1.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_1.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_1_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_1.reactions = &self->_lf__set_speed_1_reactions[0];
    self->_lf__set_speed_1.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_1.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_1.tmplt.type.element_size = sizeof(float);
    self->_lf__set_speed_2.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_2.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_2_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_2.reactions = &self->_lf__set_speed_2_reactions[0];
    self->_lf__set_speed_2.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_2.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_2.tmplt.type.element_size = sizeof(float);
    self->_lf__set_speed_3.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_3.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_3_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_3.reactions = &self->_lf__set_speed_3_reactions[0];
    self->_lf__set_speed_3.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_3.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_3.tmplt.type.element_size = sizeof(float);
    self->_lf__set_speed_4.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_4.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_4_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_4.reactions = &self->_lf__set_speed_4_reactions[0];
    self->_lf__set_speed_4.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_4.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_4.tmplt.type.element_size = sizeof(float);
    self->_lf__set_speed_5.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_5.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_5_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_5.reactions = &self->_lf__set_speed_5_reactions[0];
    self->_lf__set_speed_5.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_5.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_5.tmplt.type.element_size = sizeof(float);
    self->_lf__set_speed_6.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__set_speed_6.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__set_speed_6_reactions[0] = &self->_lf__reaction_1;
    self->_lf__set_speed_6.reactions = &self->_lf__set_speed_6_reactions[0];
    self->_lf__set_speed_6.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__set_speed_6.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__set_speed_6.tmplt.type.element_size = sizeof(float);
    return self;
}
