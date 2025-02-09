#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/Encoder/QDEC.h"
#include "_qdec.h"
// *********** From the preamble, verbatim:
#line 23 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Encoder.lf"
UART_HandleTypeDef huart3;
static int printVals = true;
static int printcnt = 0;
static int printcnt_lim = 100;

uint8_t qdec_reset[1]= {0x01};
uint8_t qdec_disable[1]= {0x02};
uint8_t qdec_enable[1]= {0x03};

uint8_t req_qdec[2]=  {0x04, 0x05};


float cpr_qdec[7]=   {52000, 52000, 52000, 52000, 52000, 52000, 52000};
float cpr_sea[7]=     {40000, 40000, 40000, 40000, 40000, 40000, 40000};
float pi = 3.14159;

uint8_t data[100];
long count[7];

static void MX_UART3_Init(void) {
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}


static void get_ticks(char msg) {
  HAL_UART_Transmit(&huart3, &(msg), 1, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart3, data, 49, HAL_MAX_DELAY);

  for(int i=0; i<7; i++) {
    count[i] = data[0 + (i*7)] + (data[1 + (i*7)]<<8) + (data[2 + (i*7)]<<16) + (data[3 + (i*7)]<<24) + (data[4 + (i*7)]<<32) + (data[5 + (i*7)]<<40) + (data[6 + (i*7)]<<48);
    if (printVals && printcnt>=printcnt_lim) {
      printf("%ld  ", count[i]);
    }
  }
}
const int LEN = 20;
static int hist[20] ={0, 0, 0, 0, 0, 0, 0};
static int med_ind = 0;

static int get_avg_r(uint16_t value) {
  hist[med_ind] = (int)value;
  med_ind = (med_ind >= LEN)? 0 : med_ind+1;

  int sum = 0;
  for (int i=0; i<LEN; i++) {
    sum += hist[i];
  }
  return sum/LEN;
}
#line 68 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_qdec.c"

// *********** End of preamble.
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _qdecreaction_function_0(void* instance_args) {
    _qdec_self_t* self = (_qdec_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 86 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Encoder.lf"
    MX_UART3_Init();
    
    HAL_UART_Transmit(&huart3, qdec_reset, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, qdec_enable, 1, HAL_MAX_DELAY);
#line 82 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_qdec.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _qdecreaction_function_1(void* instance_args) {
    _qdec_self_t* self = (_qdec_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _qdec_reset_qdec_t* reset_qdec = self->_lf_reset_qdec;
    int reset_qdec_width = self->_lf_reset_qdec_width; SUPPRESS_UNUSED_WARNING(reset_qdec_width);
    #line 93 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Encoder.lf"
    HAL_UART_Transmit(&huart3, qdec_reset, 1, HAL_MAX_DELAY);
#line 92 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_qdec.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _qdecreaction_function_2(void* instance_args) {
    _qdec_self_t* self = (_qdec_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    int qdec_out_width = self->_lf_qdec_out_width; SUPPRESS_UNUSED_WARNING(qdec_out_width);
    _qdec_qdec_out_t** qdec_out = self->_lf_qdec_out_pointers;
    int sea_out_width = self->_lf_sea_out_width; SUPPRESS_UNUSED_WARNING(sea_out_width);
    _qdec_sea_out_t** sea_out = self->_lf_sea_out_pointers;
    #line 99 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src/lib/Encoder.lf"
    long start = HAL_GetTick();
    
    // Get values for SEA encoders
    get_ticks(req_qdec[0]);
    for (int i=0; i<7; i++) {
      lf_set(sea_out[i],  2*pi* ((float) (count[i]) / cpr_sea[i]));
    }
    
    // Get values for USM encoders
    get_ticks(req_qdec[1]);
    for (int i=0; i<7; i++) {
      lf_set(qdec_out[i],  2*pi* ((float) (count[i]) / cpr_qdec[i]));
    }
    
    long end = HAL_GetTick();
    if (printVals && printcnt>=printcnt_lim) {
      printf("[%ld]", end-start);
      printf("| \r\n");
    }
    
    if (printcnt > printcnt_lim) {
      printcnt = 0;
    } else {
      printcnt ++;
    }
#line 128 "/Users/naichenzhao/Desktop/MRIRobotProject/MRIRobot/Firmware/V2_bringup/src-gen/Main/_qdec.c"
}
#include "include/api/reaction_macros_undef.h"
_qdec_self_t* new__qdec() {
    _qdec_self_t* self = (_qdec_self_t*)lf_new_reactor(sizeof(_qdec_self_t));
    // Set input by default to an always absent default input.
    self->_lf_reset_qdec = &self->_lf_default__reset_qdec;
    // Set the default source reactor pointer
    self->_lf_default__reset_qdec._base.source_reactor = (self_base_t*)self;
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _qdecreaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _qdecreaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__reaction_2.number = 2;
    self->_lf__reaction_2.function = _qdecreaction_function_2;
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
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__startup.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__startup_reactions[0] = &self->_lf__reaction_0;
    self->_lf__startup.last_tag = NEVER_TAG;
    self->_lf__startup.reactions = &self->_lf__startup_reactions[0];
    self->_lf__startup.number_of_reactions = 1;
    self->_lf__startup.is_timer = false;
    self->_lf__reset_qdec.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__reset_qdec.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__reset_qdec_reactions[0] = &self->_lf__reaction_1;
    self->_lf__reset_qdec.reactions = &self->_lf__reset_qdec_reactions[0];
    self->_lf__reset_qdec.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__reset_qdec.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__reset_qdec.tmplt.type.element_size = sizeof(bool);
    return self;
}
