/* Raed all sensors that are part of state
* to avoid slowing down control thread, these should all be fast operations,
* with limited polling time. A/D converter is order 1 us
* sensor reading centralized here, to simplify control code

*/

#include "stm32_std.h"
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

extern float get_time_float(void);
extern uint32_t get_time(void);
extern void printq_add(char *);
extern uint16_t read_adc(int);
extern int32_t read_hx711(void);
extern int32_t read_qdec3();
extern int32_t read_qdec5();
int32_t print_wait = PRINTWAIT;  // wait in time in ms
// #define PRINT_INTERVAL 3000 // interval for printing state

/* size of stack area used by each thread */
#define STACKSIZE 4096  // needs to be large because of printf
/* scheduling priority - larger = lower priority */
#define PRIORITY 120  // low for print_state task

// control thread define
K_THREAD_STACK_DEFINE(print_state_thread_stack_area, STACKSIZE);
static struct k_thread print_state_thread_data;   // structure to hold kernel data about thread

// structure for state data
struct state_data_t state_data;
struct state_data_t state_data1;

// mutex for state structure
K_MUTEX_DEFINE(state_mutex);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void read_state(void);
void print_state(void);
void start_print_state(void);
void print_state_thread(void);

void read_state()
{   float time_start, time_diff;
    int channel;
// not sure mutex is needed, read_state is only used by cotrol_thread
    //if (k_mutex_lock(&state_mutex, K_MSEC(100)) == 0) 
    if(1)
    {
    /* mutex successfully locked */
 
        time_start = get_time_float();  // in sec
        state_data.time_stamp = (float) time_start;

        // 8 analog to digital converter channels
        for (channel = 8; channel <= 15; channel++)
        {   state_data.adc[channel-8] = read_adc(channel); }
        
        // load cell
        state_data.hx711 = read_hx711();
        
        // quadrature decoders
        state_data.qdec3 = read_qdec3();
        state_data.qdec5 = read_qdec5();     
     //   k_mutex_unlock(&state_mutex); 
    }
    else 
    {  printk("# error: read_state Cannot state_data.lock \n");
    }
#ifdef DEBUG_PRINT1
    time_diff= get_time_float()- time_start;  
	printk("# read state elapsed time:%6.3f us\n", 1e6*time_diff);
#endif

}

// CAUTION: printf for floats takes lots of stack space
// so calling thread needs to have enough stack space.
void print_state()
{   char log[TEXT_LINE_LENGTH];

// need to use lock so do not get partial (mixed) ints/floats

    snprintf(log, sizeof(log),
            "t=%8.3f hx711=%d  qdec3,5 %6d%6d  adc8-15 %5d%5d%5d%5d%5d%5d%5d%5d\n", 
             state_data.time_stamp,  state_data.hx711, state_data.qdec3, state_data.qdec5,
             state_data.adc[0], state_data.adc[1],state_data.adc[2],state_data.adc[3],
             state_data.adc[4], state_data.adc[5], state_data.adc[6], state_data.adc[6]);
    printq_add(log);
}


void print_state_thread()
{   char log[TEXT_LINE_LENGTH];
     uint32_t time_stamp; // should be 64 bits 

    time_stamp = get_time();  // only 32 bits need to fix to 64
        snprintf(log, sizeof(log), "# Starting print_state thread: at tick %d (%d ms PRNWAIT)\n",
           (int) time_stamp, print_wait);
    printq_add(log);
    snprintf(log, sizeof(log),
            "# %7s,%9s,%6s,%6s,%5s, %5s, %5s, %5s, %5s, %5s, %5s, %5s  \n",
              "t=","hx711","qdec3","qdec5", "adc8","adc9","adc10", "adc11","adc12","adc13","adc14","adc15"); 
    printq_add(log);

    while(1)
    {   
        // need to use lock so do not get partial (mixed) ints/floats
        if (k_mutex_lock(&state_mutex, K_MSEC(100)) == 0) 
        {
            memcpy(&state_data1, &state_data, sizeof(state_data)); // make quick copy for printing 
            k_mutex_unlock(&state_mutex); 
        }
        else 
        {  printk("# error: print_state Cannot state_data.lock \n");
        }
        
        snprintf(log, sizeof(log),
            "%9.4f,%9d,%6d,%6d,%5d, %5d, %5d, %5d, %5d, %5d, %5d, %5d\n", 
             state_data1.time_stamp,  state_data1.hx711, state_data1.qdec3, state_data1.qdec5,
             state_data1.adc[0], state_data1.adc[1],state_data1.adc[2],state_data1.adc[3],
             state_data1.adc[4], state_data1.adc[5], state_data1.adc[6], state_data1.adc[6]);
        printq_add(log);
        k_msleep(print_wait);
    }
}


void start_print_state()
{
    printk("# Starting print_state thread\n");
/* spawn thread */
	k_tid_t tid = k_thread_create(&print_state_thread_data, print_state_thread_stack_area,
			K_THREAD_STACK_SIZEOF(print_state_thread_stack_area), 
            (k_thread_entry_t) print_state_thread, 
            NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "print_state");
	k_thread_start(&print_state_thread_data);
    // printk("# Starting command queue\n");
    
}
