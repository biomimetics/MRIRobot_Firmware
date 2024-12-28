/* thread for reading serial input without blocking
* serial input is written to command queue for asynchronous processing 
*/

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <errno.h>
extern int errno;

#include <string.h>

#include <console/console.h>
#include "stm32_std.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

extern struct k_msgq cmdq;  // command queue to hold received USB commands 

/* size of stack area used by each thread */
#define STACKSIZE 2048
/* scheduling priority - larger = lower priority */
#define PRIORITY 64  // high number, low priority for input thread (blocking)

#undef CONFIG_UART_CONSOLE_ECHO
#define CONFIG_UART_CONSOLE_ECHO 0



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void printq_add(char *msg);
extern float get_time_float(void);
void start_uart_input(void);
void uart_input_thread(void);


K_THREAD_STACK_DEFINE(input_stack_area, STACKSIZE);
static struct k_thread input_thread_data;   // structure to hold kernel data about thread


void start_uart_input(void)
{
    printk("# Starting uart input thread\n");
/* spawn control thread */
	k_tid_t tid = k_thread_create(&input_thread_data, input_stack_area,
			K_THREAD_STACK_SIZEOF(input_stack_area), 
            (k_thread_entry_t) uart_input_thread, 
            NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "input_thread");
	k_thread_start(&input_thread_data);
}

/* parse input manually- use basic functions to avoid problems with excess length or improperly terminated strings
* this avoid issues with strlcpy, or missing strtok_r()
*/
void parse_input(char *input, struct cmd_struct_def *cmd_struct)
{ 
  int i = 0; int j =0;
  char value_string[INPUT_CMD_LENGTH] = "0"; // in case no valid input set value to 0
  int value;
  char *end_pointer; // end of scan for string to long conversion
  char c;

  // copy first string to command in structure, up to space delimiter
  while( (i < INPUT_CMD_LENGTH-1) && (input[i] != '\0') && (input[i] != ' '))
  { cmd_struct->cmd[i] = input[i];
    i++;
  }
  cmd_struct->cmd[i] = '\0'; // force string to be properly terminated at last input or truncate
  
  // copy second string after space delimiter to temporary string, terminate if end of string
  // value also limited to INPUT_CMD_LENGTH. Should check if text is space or number
  c = input[i]; // read next character
  while( (j < INPUT_CMD_LENGTH-1) && (c != '\0'))                             
  { if ( ((c >= '0') && (c <= '9')) || (c == '-')   ) 
    { value_string[j] = c;
    i++; j++;
    c = input[i];
    }
    else
    { if (c == ' ')  // skip spaces
      { i++; 
      c = input[i];
      }
      else
      { j++;  // make sure '\0' is written after '0' to get valid number
        break; // exit while loop if not ASCII space or number
      }
    }
  }
  value_string[j] = '\0'; // terminate string 
  // CAUTION strtol does not check for valid digits
  value = (int) strtol(value_string, &end_pointer, 10);  // using base 10
//  printk("# strtol errno=%d\n", errno);
//  printk("# start pointer =0x%lx  end pointer = 0x%lx\n", 
//              (long)value_string, (long)end_pointer);
  cmd_struct -> value = value;
  // printk("# cmd = %s value = %d\n", cmd_struct->cmd, cmd_struct->value);
    
}

void uart_input_thread(void)
{
    float time_stamp; // should be 64 bits
    char log[TEXT_LINE_LENGTH];
    char line[80];
    int i = 0;
    int j = 0;
    char c; 
    struct cmd_struct_def cmd_struct;    // local copy structure char[8] cmd, int value
    
    // console_getline_init();
   // console_init();
    
    time_stamp = get_time_float();  // only 32 bits need to fix to 64
    snprintf(log, sizeof(log), "# Starting uart input thread: at tick %f \n",
           time_stamp);
    printq_add(log);

    while(1)  // main input loop
    {   k_msleep(SLEEP_TIME_MS); // need short delay to not miss input characters
    // or should wait for start of new line, and read at high priority
#ifdef DEBUG_PRINT1
        snprintf(log, sizeof(log),"# uart_input_thread. Enter Command txt+int:\n");
      	printq_add(log);
#endif
        // need getline without echo for interface with python
        c = '\n';
        while( (c == '\n') || (c == '\r'))  
        {  c = (char) console_getchar(); // discard extra line feed characters
        }
        line[0] = c;
        j = 1;
       
        while( (c != '\r') && (c != '\n') && (j < sizeof(line)-1))
        { 
          c = (char) console_getchar();
          line[j] = c;
          j++;
        }
        line[j-1] ='\0';   // omit end of line char. make sure string is terminated, last character is line[79]
        
//        printk("# UART read line %s\n", line);

//        line = console_getline();  // returns pointer to char buffer with line (< 128 length)
        // need to parse input line
        parse_input(line, &cmd_struct);
        
        // printk("# cmd = %s value = %d\n", cmd_struct.cmd, cmd_struct.value);
        // input commmand should be added to command queue
        time_stamp = get_time_float();
        cmd_struct.time_stamp = time_stamp;
      	
        snprintf(log, sizeof(log), "# Input_thread count %d time %f cmd = %s value = %d\n", 
            i, cmd_struct.time_stamp, cmd_struct.cmd, cmd_struct.value);
        printq_add(log);
      	i++;

        if(k_msgq_put(&cmdq, &cmd_struct, K_NO_WAIT) != 0) // send data to back of queue,
        {    printk("# uart_input_thread: cmdq put failed \n"); }
    // non-blocking, wait=0 ==> return immediately if the queue is already full.
    }




}