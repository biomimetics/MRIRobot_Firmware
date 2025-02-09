// Includes
#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <sys/printk.h>
#include "stm32_std.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

// allow 128 characters per message
K_MSGQ_DEFINE(printq, TEXT_LINE_LENGTH, 8, 4); // 8 items max 

// use semaphor to only give 1 thread access to printq at a time
K_SEM_DEFINE(printq_sem, 0,1); // in effect binary semaphore, max count of 1


/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
/* CAUTION: if printq reading is higher priority than writing can block with empty printq
* and lockout lower priority threads 
*/ 
#define PRINT_PRIORITY 100  //higher number = lower priority, 
#define THREAD_DELAY 1000 // milliseconds

typedef unsigned long int   u32_t;
extern void led_toggle();
extern u32_t get_time_diff(void);
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void printq_add(char *msg);
void printq_init(uint32_t queue_length, uint32_t max_msg_length);
void uart_print_thread();
void printString(char *);  // quick replacement for printf to save stack space
void start_print_thread(void);

/*******************************************************************************
 * Functions
 ******************************************************************************/

// Initialize logging queue (used for both UART and WiFi) and recv queue for commands from WiFi
void printq_init(uint32_t queue_length, uint32_t max_msg_length)
{   // already set up by K_MSGQ_DEFINE(print

}

// add an item to the print queue
void printq_add(char *msg)
{   if(k_sem_take(&printq_sem, K_MSEC(50)))
    { printk("# printq access blocked by other thread. msg = %s", msg); } //assume msg has \n
    else
    {
        if(k_msgq_put(&printq, msg, K_NO_WAIT) != 0) // send data to back of queue,
        {    printk("# printq_add: printq put failed \n"); }
    }
    // non-blocking, wait=0 ==> return immediately if the queue is already full.
    #ifdef DEBUG_PRINT1
    printk("# printq_add %d %s", count, msg);
    #endif
    k_sem_give(&printq_sem);
}

K_THREAD_STACK_DEFINE(print_thread_stack_area, STACKSIZE);
static struct k_thread print_thread_data;

void start_print_thread(void)
{
    printk("# Starting print thread\n");
    /* spawn print thread */
	k_tid_t tid = k_thread_create(&print_thread_data, print_thread_stack_area,
			STACKSIZE, uart_print_thread, NULL, NULL, NULL,
			PRINT_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "print_thread");

	k_thread_start(&print_thread_data);
}



// Task for printing log
void uart_print_thread()
{   int no_msg;
    uint32_t time_diff;
    uint32_t counter = 0;
    char log[TEXT_LINE_LENGTH + 1];

    while (1)
    {   no_msg = k_msgq_get(&printq, &log, K_NO_WAIT); // wait for new item on queue
        if (no_msg == 0)
        {
            time_diff = get_time_diff();
            #ifdef DEBUG_PRINT1
                printk("msg %d %s ", counter, log);  // \n and \r already in log
            #else
                printk("%s ", log);  // \n and \r already in log
                // printString(log); // already formatted, so quicker than printk??
            #endif

            // 
            time_diff = get_time_diff();
     //   printk("# time for printk: %u\n", time_diff);
            counter++;
            // led_toggle();
        }
        
        // k_msleep(10); // let other threads run
    }
}

// add end of string protection
void printString(char *string)
{   
    int i=0;
    while ((string[i] != '\0') && (i < TEXT_LINE_LENGTH)) 
    {
            fputc(string[i],stdout); // print single character, avoid printf to save on stack space and speed up
            i++;
    }   
    if (i>= TEXT_LINE_LENGTH) 
        printk("# printString buffer overrun %d chars", i);
}

