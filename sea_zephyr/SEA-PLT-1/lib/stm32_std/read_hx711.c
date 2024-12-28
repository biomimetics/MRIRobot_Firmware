/* driver for load cell HX711 using TIM2 to make ~40 us pulse clock 
* 
* PB2 	TIM2_CH4	CN10-22	output used for HX711 PD_SCK
* PB5	GPIOB_5  	CN10-29 input used for HX711 #1 DOUT
*ChatGPT: User
* write code to generate 40 us pulse for nucleo-f446re for timer 2 channel 4 using low level library
* 
*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include <sys/printk.h>
#include <zephyr.h>
#include "stm32_std.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// Define the pulse duration in microseconds
#define PULSE_DURATION_US 40
// use prescale to get 1 us clock
#define PRESCALE (SystemCoreClock / 960000)  // works for 96 MHz clock
#define PULSE_COUNT 27 // 25==> gain 128, 26 ==> Gain 32, 27 ==> Gain 64
static int32_t hx711_buf;

/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority - larger = lower priority */
#define PRIORITY 90  // high number, low priority for input thread (blocking)

K_THREAD_STACK_DEFINE(hx711_thread_stack_area, STACKSIZE);
static struct k_thread hx711_thread_data;   // structure to hold kernel data about thread

extern void printq_add(char *);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void hx711_init(void);
//void hx711_thread(void *, void *, void *); 
int32_t read_hx711(void);
void start_hx711_thread(void);



// Function to configure the timer and generate the pulse
void hx711_init() {
    // Enable the timer clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    // Configure GPIO for the timer channel PB2 TIM2_CH4
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure the timer
    LL_TIM_InitTypeDef TIM_InitStruct;
    TIM_InitStruct.Prescaler = PRESCALE;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = PULSE_DURATION_US * 2 - 1;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);

    // Configure the timer channel
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;  // output low on match
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.CompareValue = PULSE_DURATION_US - 1;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);

    LL_TIM_SetOnePulseMode (TIM2, LL_TIM_ONEPULSEMODE_SINGLE);
    // Enable the timer channel output
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);

    // Enable the counter. This is reset in One Pulse Mode, and generates one pulse
    // LL_TIM_EnableCounter(TIM2);

    // Configure PB5 as input
    LL_GPIO_SetPinMode(GPIOB, HX711_DOUT, LL_GPIO_MODE_INPUT);
    #ifdef DEBUG_PRINT
    printk("# End HX711 init TIM2 Ch4 PB2 as PD_SCK and PB5 as DOUT input\n");
    #endif

}

// generate clock cycles and read data, use dummy pointer arguments for thread
void hx711_thread(void *a1, void *a2, void *a3)
{   int hx711_data=0;
    int i;
    char log[TEXT_LINE_LENGTH];
    snprintf(log, sizeof(log), "# Starting hx711_thread\n");
    printq_add(log);
    #ifdef DEBUG_PRINT1
    printk("# Starting reading load cell HX711 thread\n");
    #endif
    // Main program loop
    while(1)
    {
        // wait for DOUT (GPIOB5) to go low, signalling new data ready
        while(LL_GPIO_IsInputPinSet(GPIOB, HX711_DOUT))
        {  k_msleep(1); // let other threads execute while polling data in
        } 
        hx711_data=0;   // reset for next reading
        // first 24 SCK shift data out of hx711, pulses 25, 26, 27 set gain
        for(i = 0; i < 23; i++)
        { 
            LL_TIM_EnableCounter(TIM2);  // re-enable to get another pulse
        // need at least 0.1 us 
        //    k_msleep(1); // let other threads execute while polling data in
            while(LL_TIM_IsEnabledCounter(TIM2)); // wait for 40 us pulse to finish
            if (LL_GPIO_IsInputPinSet(GPIOB, HX711_DOUT)) 
            {  hx711_data = (hx711_data << 1) | 0x00000001; }
            else 
            { hx711_data = (hx711_data << 1) & 0xfffffffe; }
        } 
        for(i = 24; i < PULSE_COUNT; i++)
        { 
            LL_TIM_EnableCounter(TIM2);  // re-enable to get another pulse
        // need at least 0.1 us 
        //    k_msleep(1); // let other threads execute while polling data in
            while(LL_TIM_IsEnabledCounter(TIM2)); // wait for 40 us pulse to finish
        } 
        hx711_buf = hx711_data; // copy value to memory for atomic read  
        // printk("# hx711_data = %x \n", hx711_data);     
        k_msleep(1); // let other threads execute while polling data in
    }
}

// return most recent hx711 value. Will send oldest value repeatedly with no 
// warning. time stamp can be added later if needed
int32_t read_hx711(void)
{ return(hx711_buf);
}


void start_hx711(void)
{  
    #ifdef DEBUG_PRINT
    printk("# Starting hx711 thread\n");
    #endif
    /* spawn hx711 thread */
	k_tid_t tid = k_thread_create(&hx711_thread_data, hx711_thread_stack_area,
			STACKSIZE, hx711_thread, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "hx711_thread");

	k_thread_start(&hx711_thread_data);
}
