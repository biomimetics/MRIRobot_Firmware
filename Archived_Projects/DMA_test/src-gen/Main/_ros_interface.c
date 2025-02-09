#include "include/api/schedule.h"
#include "low_level_platform/api/low_level_platform.h"
#include "include/ROS_Interface/ROS_Interface.h"
#include "_ros_interface.h"
// *********** From the preamble, verbatim:
#line 27 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/ROS_Interface.lf"
UART_HandleTypeDef huart1;        // usb-c tied urart for computer communication
extern UART_HandleTypeDef huart2;

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

// Setup Rx and Tx buffers
static uint8_t rx_buffer[50];
static uint8_t tx_buffer[50];

int enc_data[7];
int sea_data[7];
int16_t joint_data[7];
float scaled_joint_data[7];

uint8_t UART1_rxBuffer2[20] = "THEREHELLO\r\n";

// We have an inFlight indicator if the DMA is currently sending data
static int inFlight = 0;

static float get_target(uint8_t* data) {
  float count = data[0] + (data[1]<<8) + (data[2]<<16) + (data[3]<<24) + (data[4]<<32);
  return count;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  inFlight = 0;
}

static void MX_UART1_Init(void) {
  huart1.Instance = USART1;
  // huart1.Init.BaudRate = 921600;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;\
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

static void MX_DMA_Init(void) {
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}
#line 63 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ros_interface.c"

// *********** End of preamble.
// ***** Start of method declarations.
// ***** End of method declarations.
#include "include/api/reaction_macros.h"
void _ros_interfacereaction_function_0(void* instance_args) {
    _ros_interface_self_t* self = (_ros_interface_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 87 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/ROS_Interface.lf"
    // Prep the send indicator
    tx_buffer[0] = 'e';
    
    // Prep UART interfaces
    MX_DMA_Init();
    MX_UART1_Init();
#line 79 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ros_interface.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _ros_interfacereaction_function_1(void* instance_args) {
    _ros_interface_self_t* self = (_ros_interface_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    
    #line 97 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/ROS_Interface.lf"
    printf("TRYSEND\r\n");
    if (! inFlight) {
      printf("SENDED\r\n");
      // HAL_UART_Transmit_DMA(&huart1, UART1_rxBuffer2, 12);
      HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, UART1_rxBuffer2, 12);
      if (status == HAL_OK) {
        printf("OK\r\n");
      } else if (status == HAL_BUSY) {
        printf("BUSY\r\n");
      } else {
        printf("FUCK\r\n");
      }
    }
#line 100 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ros_interface.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _ros_interfacereaction_function_2(void* instance_args) {
    _ros_interface_self_t* self = (_ros_interface_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    int target_pos_width = self->_lf_target_pos_width; SUPPRESS_UNUSED_WARNING(target_pos_width);
    _ros_interface_target_pos_t** target_pos = self->_lf_target_pos_pointers;
    #line 113 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/ROS_Interface.lf"
    if (HAL_UART_Receive(&huart1, rx_buffer, 1, 0) == HAL_OK && (rx_buffer[0] == 'p')) {
      if (HAL_UART_Receive(&huart1, rx_buffer, 14, 100) == HAL_OK) {
        for (int i=0; i<7; i++) {
          joint_data[i] = rx_buffer[(i*2)+1]  + (rx_buffer[(i*2)]<<8);
        }
    
        scaled_joint_data[0] = ((float) joint_data[0])/1000;
        scaled_joint_data[1] = ((float) joint_data[1])/1000;
        scaled_joint_data[2] = ((float) joint_data[2])/1000;
        scaled_joint_data[3] = ((float) joint_data[3])/1000;
        scaled_joint_data[4] = ((float) joint_data[3])/1000;
        scaled_joint_data[5] = ((float) joint_data[1])/1000;
        scaled_joint_data[6] = ((float) joint_data[6])/1000;
    
        for (int i=0; i<7; i++) {
          lf_set(target_pos[i], scaled_joint_data[i]);
        }
        printf("got: %d,%d,%d,%d,%d,%d,%d\n", (int)joint_data[0], (int)joint_data[1], (int)joint_data[2], (int)joint_data[3], (int)joint_data[4], (int)joint_data[5], (int)joint_data[6]);
      }
    
    }
#line 130 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ros_interface.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _ros_interfacereaction_function_3(void* instance_args) {
    _ros_interface_self_t* self = (_ros_interface_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _ros_interface_current_pos_t** current_pos = self->_lf_current_pos;
    int current_pos_width = self->_lf_current_pos_width; SUPPRESS_UNUSED_WARNING(current_pos_width);
    #line 138 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/ROS_Interface.lf"
    enc_data[0] = (int)(current_pos[0]->value * 1000);
    enc_data[1] = (int)(current_pos[1]->value * 1000);
    enc_data[2] = (int)(current_pos[2]->value * 1000);
    enc_data[3] = (int)(current_pos[3]->value * 1000);
    enc_data[4] = (int)(current_pos[4]->value * 1000);
    enc_data[5] = (int)(current_pos[5]->value * 1000);
    enc_data[6] = (int)(current_pos[6]->value * 1000);
#line 146 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ros_interface.c"
}
#include "include/api/reaction_macros_undef.h"
#include "include/api/reaction_macros.h"
void _ros_interfacereaction_function_4(void* instance_args) {
    _ros_interface_self_t* self = (_ros_interface_self_t*)instance_args; SUPPRESS_UNUSED_WARNING(self);
    _ros_interface_sea_pos_t** sea_pos = self->_lf_sea_pos;
    int sea_pos_width = self->_lf_sea_pos_width; SUPPRESS_UNUSED_WARNING(sea_pos_width);
    #line 148 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src/lib/ROS_Interface.lf"
    sea_data[0] = (int)(sea_pos[0]->value * 1000);
    sea_data[1] = (int)(sea_pos[1]->value * 1000);
    sea_data[2] = (int)(sea_pos[2]->value * 1000);
    sea_data[3] = (int)(sea_pos[3]->value * 1000);
    sea_data[4] = (int)(sea_pos[4]->value * 1000);
    sea_data[5] = (int)(sea_pos[5]->value * 1000);
    sea_data[6] = (int)(sea_pos[6]->value * 1000);
#line 162 "/Users/naichenzhao/Desktop/BML/MRIRobotProject/MRIRobot/Firmware/DMA_test/src-gen/Main/_ros_interface.c"
}
#include "include/api/reaction_macros_undef.h"
_ros_interface_self_t* new__ros_interface() {
    _ros_interface_self_t* self = (_ros_interface_self_t*)lf_new_reactor(sizeof(_ros_interface_self_t));
    // Set the default source reactor pointer
    self->_lf_default__current_pos._base.source_reactor = (self_base_t*)self;
    // Set the default source reactor pointer
    self->_lf_default__sea_pos._base.source_reactor = (self_base_t*)self;
    self->_lf__reaction_0.number = 0;
    self->_lf__reaction_0.function = _ros_interfacereaction_function_0;
    self->_lf__reaction_0.self = self;
    self->_lf__reaction_0.deadline_violation_handler = NULL;
    self->_lf__reaction_0.STP_handler = NULL;
    self->_lf__reaction_0.name = "?";
    self->_lf__reaction_0.mode = NULL;
    self->_lf__reaction_1.number = 1;
    self->_lf__reaction_1.function = _ros_interfacereaction_function_1;
    self->_lf__reaction_1.self = self;
    self->_lf__reaction_1.deadline_violation_handler = NULL;
    self->_lf__reaction_1.STP_handler = NULL;
    self->_lf__reaction_1.name = "?";
    self->_lf__reaction_1.mode = NULL;
    self->_lf__reaction_2.number = 2;
    self->_lf__reaction_2.function = _ros_interfacereaction_function_2;
    self->_lf__reaction_2.self = self;
    self->_lf__reaction_2.deadline_violation_handler = NULL;
    self->_lf__reaction_2.STP_handler = NULL;
    self->_lf__reaction_2.name = "?";
    self->_lf__reaction_2.mode = NULL;
    self->_lf__reaction_3.number = 3;
    self->_lf__reaction_3.function = _ros_interfacereaction_function_3;
    self->_lf__reaction_3.self = self;
    self->_lf__reaction_3.deadline_violation_handler = NULL;
    self->_lf__reaction_3.STP_handler = NULL;
    self->_lf__reaction_3.name = "?";
    self->_lf__reaction_3.mode = NULL;
    self->_lf__reaction_4.number = 4;
    self->_lf__reaction_4.function = _ros_interfacereaction_function_4;
    self->_lf__reaction_4.self = self;
    self->_lf__reaction_4.deadline_violation_handler = NULL;
    self->_lf__reaction_4.STP_handler = NULL;
    self->_lf__reaction_4.name = "?";
    self->_lf__reaction_4.mode = NULL;
    self->_lf__trigger_send.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger_send.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger_send_reactions[0] = &self->_lf__reaction_1;
    self->_lf__trigger_send.reactions = &self->_lf__trigger_send_reactions[0];
    self->_lf__trigger_send.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__trigger_send.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__trigger_send.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger_send.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger_read.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger_read.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__trigger_read_reactions[0] = &self->_lf__reaction_2;
    self->_lf__trigger_read.reactions = &self->_lf__trigger_read_reactions[0];
    self->_lf__trigger_read.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__trigger_read.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__trigger_read.is_timer = true;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__trigger_read.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__startup.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__startup_reactions[0] = &self->_lf__reaction_0;
    self->_lf__startup.last_tag = NEVER_TAG;
    self->_lf__startup.reactions = &self->_lf__startup_reactions[0];
    self->_lf__startup.number_of_reactions = 1;
    self->_lf__startup.is_timer = false;
    self->_lf__current_pos.last_tag = NEVER_TAG;
    #ifdef FEDERATED_DECENTRALIZED
    self->_lf__current_pos.intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};
    #endif // FEDERATED_DECENTRALIZED
    self->_lf__current_pos_reactions[0] = &self->_lf__reaction_3;
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
    self->_lf__sea_pos_reactions[0] = &self->_lf__reaction_4;
    self->_lf__sea_pos.reactions = &self->_lf__sea_pos_reactions[0];
    self->_lf__sea_pos.number_of_reactions = 1;
    #ifdef FEDERATED
    self->_lf__sea_pos.physical_time_of_arrival = NEVER;
    #endif // FEDERATED
    self->_lf__sea_pos.tmplt.type.element_size = sizeof(float);
    return self;
}
