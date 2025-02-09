/* main control thread 
*   1. read state data from A/D, quadrature, serial, etc (maybe from a queue)
*   2. calculate control
*   3. output control to D/A, PWM, etc
*   4. log state and control data
*   5. sleep briefly to allow other lower priority threads to run 
*   modelled after EE192 Spring 2021 FreeRTOS script, converted to Zephyr RTOS
*   May 2023 R. Fearing
*/
// Includes
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <math.h>
#include "stm32_std.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/* size of stack area used by each thread */
#define STACKSIZE 4096
/* scheduling priority - larger = lower priority */
#define CONTROL_PRIORITY 10  // high for control task, but USART2 input should be higher
#define PI 3.14159265359
#define FREQ 50  // frequency in Hz


extern void printq_add(char *);
extern float get_time_float(void);
extern uint32_t get_time(void);
extern void print_time(void);
extern void set_pwm(int, uint32_t);
extern void generatePulses(int, int);
extern void stepOff(void);
extern void set_dac(int);
extern uint16_t read_adc(int);
// extern int32_t read_hx711(void);
// extern int32_t read_qdec3();
// extern int32_t read_qdec5();
extern void read_state(void);
extern void print_state(void);
extern int32_t print_wait; // delay time

// control thread define
K_THREAD_STACK_DEFINE(control_stack_area, STACKSIZE);
static struct k_thread control_thread_data;   // structure to hold kernel data about thread

// command queue define
K_MSGQ_DEFINE(cmdq, sizeof(struct cmd_struct_def), 4, 16); // 4 items max, align on 16 bytes 

// structure for state data
extern struct state_data_t state_data;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void control_thread(void);
void start_control(void);
void parse_cmd(char *, int);

void estop(void);

// turn all PWM and motors off
void estop()
{ 
    generatePulses(1, (int) 0); 
    generatePulses(2, (int) 0); 
    set_pwm(1, (uint32_t) 0);
    set_pwm(2, (uint32_t) 0);
    set_pwm(3, (uint32_t) 0);
    set_pwm(4, (uint32_t) 0);
    set_pwm(5, (uint32_t) 0);
    set_pwm(6, (uint32_t) 0);
    set_pwm(7, (uint32_t) 0);
    set_pwm(8, (uint32_t) 0);
}



void start_control()
{
    printk("# Starting control thread\n");
/* spawn control thread */
	k_tid_t tid = k_thread_create(&control_thread_data, control_stack_area,
			K_THREAD_STACK_SIZEOF(control_stack_area), 
            (k_thread_entry_t) control_thread, 
            NULL, NULL, NULL, CONTROL_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "control_thread");
	k_thread_start(&control_thread_data);
    // printk("# Starting command queue\n");
    
}


void control_thread()
{   uint32_t time_stamp; // should be 64 bits
    char log[TEXT_LINE_LENGTH];
    int control; // control output
    struct cmd_struct_def cmd_struct; // local copy

    k_msleep(100); // some delay before starting control thread
    // give lower priority threads chance to run?

    time_stamp = get_time();  // only 32 bits need to fix to 64
    snprintf(log, sizeof(log), "# Starting control thread: at tick %d \n",
           (int) time_stamp);
    // printq_add(log);
    
    while(1)  // main control loop
    {   // check without blocking for new command on queue
        if(k_msgq_get(&cmdq, &cmd_struct, K_NO_WAIT)==0)
        { /*
              printk("# new cmdq value found. Time %f cmd=%s value=%d\n", 
                    cmd_struct.time_stamp, cmd_struct.cmd, cmd_struct.value);
           */ 
            parse_cmd(cmd_struct.cmd, cmd_struct.value);
        } 
        #ifdef DEBUG_PRINT1
        else
        {   printk("# cmdq empty ");
        }
        #endif
        
        k_msleep(1);
        // time_stamp = get_time();
        read_state();
        control=(int)  1000*(1 + cos(2*PI*FREQ*state_data.time_stamp));
        set_dac(control);

        // avoid any print within main control loop
       
    }

}

// handle commands from UDP, format command (8 characters), value (int)
void parse_cmd(char command[], int value)
{ char ucommand[INPUT_CMD_LENGTH];
    int i=0;
    char log[TEXT_LINE_LENGTH];
// last character is ucommand[INPUT_COMMAND_LENGTH-1] is \0 and should not be over written
    while( (i < INPUT_CMD_LENGTH-1) && (command[i] != '\0') )
    {  ucommand[i] = (char) toupper((int) command[i] ); // convert to upper case
        i++;
    }
    ucommand[i] = '\0';   // force last character to be end of string
#ifdef DEBUG_PRINT1
    snprintf(log, sizeof(log),"# ucommand = 0x[%x %x %x %x %x %x %x %x]\n", 
                ucommand[0],ucommand[1],ucommand[2],ucommand[3],
                ucommand[4],ucommand[5],ucommand[6],ucommand[7]); 
    printq_add(log);  
#endif 
//    printk("\n# In parse_command. command:%s value %d\n", ucommand, value);
    if (strcmp(ucommand,"TIME") == 0) 
    {   print_time(); 
        return;
    }
    if(strcmp(ucommand,"ESTOP") == 0)
    {   estop();
        snprintf(log, sizeof(log),"# Emergency Stop received\n"); 
        printq_add(log);
        return;
    }
    if(strcmp(ucommand,"PFRQ1") == 0)
    {   generatePulses(1, (uint32_t) value); 
        return;
    }
    if(strcmp(ucommand,"PFRQ2") == 0)
    {    generatePulses(2, (uint32_t) value); 
        return;
    }
    if(strcmp(ucommand,"STEPOFF") == 0)
    {   stepOff(); 
        return;
    }
    if(strcmp(ucommand,"PWM1") == 0)
    {    set_pwm(1, (uint32_t) value); return; }
    if(strcmp(ucommand,"PWM2") == 0)
    {    set_pwm(2, (uint32_t) value); return; }
    if(strcmp(ucommand,"PWM3") == 0)
    {    set_pwm(3, (uint32_t) value); return; }
    if(strcmp(ucommand,"PWM4") == 0)
    {    set_pwm(4, (uint32_t) value); return; }
    if(strcmp(ucommand,"PWM5") == 0)
    {    set_pwm(5, (uint32_t) value); return;}
    if(strcmp(ucommand,"PWM6") == 0)
    {    set_pwm(6, (uint32_t) value);return; }
    if(strcmp(ucommand,"PWM7") == 0)
    {    set_pwm(7, (uint32_t) value); return;}
    if(strcmp(ucommand,"PWM8") == 0)
    {    set_pwm(8, (uint32_t) value);return; }
    if (strcmp(ucommand, "DAC") == 0)
    {    set_dac(value); return;}
    if (strcmp(ucommand, "ADC") == 0)
    {    read_adc(8); return;}              // should extend to choose channel
    if (strcmp(ucommand, "PRNWAIT") == 0)
    {    snprintf(log, sizeof(log),"# PRNWAIT changed from %d to %d ms \n",
         print_wait, value);
         printq_add(log);
         print_wait = value;
         return;
    }              // should extend to choose channel

    // if no successful message send error to log
    snprintf(log, sizeof(log),"# parse_cmd %s unrecognized\n", ucommand); 
    // printq_add(log);
    
/*    if (strcmp(ucommand, "KD") == 0)
    {    set_control_gain(1, value); }
    if (strcmp(ucommand, "RUN") == 0)
    {   run_control(); }
    */
}


