/* low level reading serial input using interrupts. 
* use \n to end each line
* Need interrupts to continuously read serial, as thread can be interrupted in middle of reading a line
* using polling
* line Buffer uses hand shaking with uart_input_thread(void) to 
* prevent reading partial data
*/

#include "stm32f4xx.h"
#include "stm32f4xx_ll_usart.h"
#include <sys/printk.h>
#include <zephyr.h>

#define UART_BUFFER_SIZE 128
// USART needs to be high priority to avoid missing input at 150 char/sec = every 6.6 us
// lowest priority is 15
//#define USART2_Priority (IRQ_PRIO_LOWEST - 13)
#define USART2_Priority (2)
#define USART2_IRQ_FLAGS 0
#define USART2_ISR_ARG 0

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void led_toggle();
void uart_interrupt_init(void);
char console_getchar(void);

// indexes for character input ring buffer
char line1_avail, line2_avail; // line 1 buffer is ready to be copied
char line1_read, line2_read; // line1 buffer has been read, and is ready to be over written

// Buffer to store received data
volatile uint8_t rxBuffer[UART_BUFFER_SIZE];
volatile uint8_t rxIndex = 0;


/* polling based console_getchar()*/
/* 
char console_getchar()
{ char c;
  while(!LL_USART_IsActiveFlag_RXNE(USART2))
  { k_msleep(1); }
  c = LL_USART_ReceiveData8(USART2);
  return(c);
}
 */

/* interrupt based console_getchar()using ring buffer*/
char console_getchar()
{ char c;
  static uint8_t getcharIndex = 0;
  while(rxIndex == getcharIndex)
  { k_msleep(1); } // let other threads run, getchar has caught up with rxIndex
  c = rxBuffer[getcharIndex++];
  if (getcharIndex >= UART_BUFFER_SIZE) 
  {  getcharIndex = 0; // Buffer overflow handling, reset index
  }
  return(c);
}


//void USART2_IRQHandler(void *arg) 
ISR_DIRECT_DECLARE(USART2_IRQHandler)
{ char rxData; 
    irq_disable(USART2_IRQn);  // don't allow another usart interrupt until this one is handled/cleared
// check for over run, interrupt not responding in time
    if (LL_USART_IsActiveFlag_ORE(USART2)) 
    { printk("# serial Rx over run\n");
      LL_USART_ClearFlag_ORE(USART2);
    }
    if (LL_USART_IsActiveFlag_RXNE(USART2))  // can be non-empty, and over run at same time
    {   rxData =  LL_USART_ReceiveData8(USART2);
        rxBuffer[rxIndex++] = rxData;
        if (rxIndex >=  UART_BUFFER_SIZE) 
        {  rxIndex = 0; // ring Buffer wrap around handling
      //    printk("# ring buffer wrap\n");
        } 
         //       printk("USART2_IRQHandler: char=0x%x rxIndex=%d \t", rxData, rxIndex);
    }
    
    irq_enable(USART2_IRQn);
    return(0); // The function should return nonzero status if a scheduling decision should potentially be made.  
}



void uart_interrupt_init()
{   // uint32_t int_vector;
    LL_USART_InitTypeDef USART_InitStruct; 
    USART_InitStruct.BaudRate = 230400;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl =  LL_USART_HWCONTROL_NONE; 
   
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_EnableIT_RXNE(USART2); // enable receiver not empty interrupt
    LL_USART_EnableIT_ERROR(USART2); // enable interrupt on errors, such as overRun error
    LL_USART_Enable(USART2);

    /* Zephyr interrupt installer for vector table */
    IRQ_DIRECT_CONNECT(USART2_IRQn, USART2_Priority, USART2_IRQHandler, USART2_IRQ_FLAGS);
 //   IRQ_CONNECT(USART2_IRQn, USART2_Priority, USART2_IRQHandler, USART2_ISR_ARG, USART2_IRQ_FLAGS);
    irq_enable(USART2_IRQn);

/* low level CMSIS interrupts NOT USED*/
   /*  NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn,24); // make sure priority is lower than FreeRTOS queue
    // NVIC_SetVector(USART2_IRQn, USART2_IRQHandler);
    int_vector = NVIC_GetVector(USART2_IRQn);
    printk("Interrupt vector address %x\n", int_vector);
    USART2_IRQHandlerA(); // test run once
    NVIC_EnableIRQ(USART2_IRQn);  // Enable USART2 interrupt in NVIC
     */
}







