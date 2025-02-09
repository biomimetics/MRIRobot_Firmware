/* interface for quadrature decoder in STM32F4 timer peripheral*/
/* PC6		TIM3_CH1		CN10-4	quadrature INA
*  PC7		TIM3_CH2		CN10-19	quadrature INB
*  PA0		TIM5_CH1		 works for quadrature?
*  PA1		TIM5_CH2		 works for quadrature?
*
*/
#include <sys/printk.h>
#include <soc.h> 
#include <stm32f4xx_ll_tim.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_rcc.h>
#include <stm32f4xx_ll_bus.h> // for enabling clocks
#include "stm32_std.h"

// TIM3/4/5 use alt func AF2 in pin multiplexer in port A
#define TIM3_CH1 LL_GPIO_AF_2 
#define TIM3_CH2 LL_GPIO_AF_2 

// index bits for encoders, GPIOB
#define QUAD5_INDEX LL_GPIO_PIN_10
#define QUAD3_INDEX LL_GPIO_PIN_12

/****************************
 * Prototypes
******************************/


// clock must be enabled or timer peripheral is dead 
/* (can't even set registers) */


void quad_inputs_init(void)
{   ErrorStatus err;
	int size = sizeof(LL_GPIO_InitTypeDef);
	char gpio_struct_buf[size]; // allocate safe space for structure
	LL_GPIO_InitTypeDef *gpio_init_struct; // initialization structure for GPIO pins
	gpio_init_struct = (LL_GPIO_InitTypeDef *)(gpio_struct_buf); // set pointer to scratch memory area
	
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	
   

//	printk("# size of GPIO Initialization struct is %d bytes\n", size);
	LL_GPIO_StructInit (gpio_init_struct);
	/* set appropriate registers in structure */
	// initialize PC6 and PC7 for TIM3 encoder input pins
	gpio_init_struct -> Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
	gpio_init_struct -> Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_init_struct -> Pull = LL_GPIO_PULL_UP;
	gpio_init_struct -> Alternate = LL_GPIO_AF_2;
	
	err = LL_GPIO_Init(GPIOC, gpio_init_struct);
    #ifdef DEBUG_PRINT1
	    printk("# initialize GPIOC using PC6 and PC7. err=%d ",err);
    #endif

/* set appropriate registers in structure */
	// initialize PA0 and PA1 for TIM5 encoder input pins
	gpio_init_struct -> Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
	gpio_init_struct -> Mode = LL_GPIO_MODE_ALTERNATE;
	gpio_init_struct -> Pull = LL_GPIO_PULL_UP;
	gpio_init_struct -> Alternate = LL_GPIO_AF_2;
	
	err = LL_GPIO_Init(GPIOA, gpio_init_struct);
	#ifdef DEBUG_PRINT1
		printk("# initialize GPIOA using PA0 and PA1. err=%d ",err);
	#endif

// set up index bits for input
	LL_GPIO_SetPinMode(GPIOB, QUAD3_INDEX, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull (GPIOB, QUAD3_INDEX, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(GPIOB, QUAD5_INDEX, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull (GPIOB, QUAD5_INDEX, LL_GPIO_PULL_UP);

	#ifdef DEBUG_PRINT
		printk("# End qdec inputs init. TIM3 PC6 PC7, TIM5 PA0 PA1 (32 bit)\n");
	#endif
}


void tim3_regs_init(void)
{   ErrorStatus err;
    int sizeTIM = sizeof(LL_TIM_ENCODER_InitTypeDef);
	char tim_struct_buf[sizeTIM]; // allocate safe space for structure
//	printk("# size of TIM Initialization struct is %d bytes\n", sizeTIM);

// initialization structure for encoder
	LL_TIM_ENCODER_InitTypeDef *encoder_init_struct; // structure to hold initialization
	encoder_init_struct = (LL_TIM_ENCODER_InitTypeDef *)(tim_struct_buf); // set pointer to scratch memory area

	LL_TIM_ENCODER_StructInit(encoder_init_struct); // initialize values in structure
	encoder_init_struct-> EncoderMode =0x03; /* count on TI1 and TI2 edges*/
	err = LL_TIM_ENCODER_Init(TIM3, encoder_init_struct);
	LL_TIM_EnableCounter (TIM3);
	#ifdef DEBUG_PRINT1
    	printk("# initialized encoder qdec3. err=%d",err);
	#endif
}

void tim5_regs_init(void)
{   ErrorStatus err;
    int sizeTIM = sizeof(LL_TIM_ENCODER_InitTypeDef);
	char tim_struct_buf[sizeTIM]; // allocate safe space for structure
//	printk("# size of TIM Initialization struct is %d bytes\n", sizeTIM);

// initialization structure for encoder
	LL_TIM_ENCODER_InitTypeDef *encoder_init_struct; // structure to hold initialization
	encoder_init_struct = (LL_TIM_ENCODER_InitTypeDef *)(tim_struct_buf); // set pointer to scratch memory area

	LL_TIM_ENCODER_StructInit(encoder_init_struct); // initialize values in structure
	encoder_init_struct-> EncoderMode =0x03; /* count on TI1 and TI2 edges*/
	err = LL_TIM_ENCODER_Init(TIM5, encoder_init_struct);
	LL_TIM_EnableCounter(TIM5);
	#ifdef DEBUG_PRINT1
   		printk("# initialized encoder qdec5. err=%d\n",err);
	#endif
}


void qdec_init(void)
{
    #ifdef DEBUG_PRINT1
		printk("# TIM3 encoder initialization.");
	#endif
	quad_inputs_init();
    tim3_regs_init();
    tim5_regs_init();
   // printk("# TIM3 encoder initialized\n");
}

int32_t read_qdec3(void)
{   int val;
    uint32_t data_in;
    // uint32_t q3_index;
    data_in = LL_GPIO_ReadInputPort(GPIOC);
    data_in = data_in & (LL_GPIO_PIN_6 | LL_GPIO_PIN_7);
    val = LL_TIM_ReadReg(TIM3, CNT);
	// q3_index = LL_GPIO_IsInputPinSet(GPIOB, QUAD3_INDEX);	
    // printk("# GPIOC= %x . Encoder TIM3_CNT Register =0x%x . Index = %x\n",
            // data_in, val, q3_index);
    return(val);
}

int32_t read_qdec5(void)
{   int val;
    uint32_t data_in;
    // uint32_t q5_index;
    data_in = LL_GPIO_ReadInputPort(GPIOA);
    data_in = data_in & (LL_GPIO_PIN_0 | LL_GPIO_PIN_1);
    val = LL_TIM_ReadReg(TIM5, CNT);
	// q5_index = LL_GPIO_IsInputPinSet(GPIOB, QUAD5_INDEX);
    // printk("# GPIOA= %x . Encoder TIM5_CNT Register =0x%x . Index = %x\n",
           // data_in, val, q5_index);
    return(val);
}

int32_t read_qdec3_index(void)
{   uint32_t q3_index;
	q3_index = LL_GPIO_IsInputPinSet(GPIOB, QUAD3_INDEX);
    return(q3_index);
}

int32_t read_qdec5_index(void)
{   uint32_t q5_index;
	q5_index = LL_GPIO_IsInputPinSet(GPIOB, QUAD5_INDEX);
    return(q5_index);
}

void reset_qdec3(void){
	LL_TIM_WriteReg(TIM3, CNT, 0);
}

void reset_qdec5(void) {
	LL_TIM_WriteReg(TIM5, CNT, 0);
}