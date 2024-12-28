
#ifndef STM32_STD_H
#define STM32_STD_H

/* common definitions for controller code */
#include <zephyr.h>
// change this for longer lines in log file (allocated at compile time)
#define TEXT_LINE_LENGTH 128 // allow 128 characters per log message
#define INPUT_CMD_LENGTH 8 // allow 7 characters + '\0' for text cmd+value

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1
// initial value for delay between state updates
#define PRINTWAIT 5000


/* for debugging printk messages*/
#define DEBUG_PRINT
// #define DEBUG_PRINT1 // more detailed debugging

// structure for commands coming from UART interface
struct cmd_struct_def {
    float time_stamp;
    char cmd[INPUT_CMD_LENGTH];
    int value;
};  

struct state_data_t {
    float time_stamp;
    uint16_t adc[8];  // 8 adc channels 8..15
    
    int32_t hx711;
    int32_t qdec3; // quadrature decoder using timer 3
    int32_t qdec5; // quadrature decoder using timer 5
};



/* GPIO Port B pin definitions
GPIOB Pin 13, and 14 are direction for each motor
* GPIOB Pin 15 is used for stepping motor free (unenergized=1)
*/
#define HX711_DOUT LL_GPIO_PIN_5
#define StepMotor1DirPin LL_GPIO_PIN_13
#define StepMotor2DirPin LL_GPIO_PIN_14
#define StepMotorFreePin LL_GPIO_PIN_15

#endif