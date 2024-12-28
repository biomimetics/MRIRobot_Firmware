/* use hardware clock to estimate time of various functions*/
/* stm32 is missing 64 bit clock counter, so need to add wrap around on 32 bit counter */

#include <zephyr.h>
#include <stdint.h>
#include <zephyr/types.h>
#include <kernel.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>

extern void printq_add(char *);

float get_time_float(void)
{   static uint32_t old_time =0; // initialize 
    uint32_t new_time;
    static uint64_t upper_time=0;
    float time_float;
    int lock_num;

// need to make uninterruptible so time is not corrupted during rollover
// routine can be called by multiple threads, and static variable could be updated twice
    lock_num = irq_lock();
    new_time = k_cycle_get_32();
    if(old_time > new_time)
    { upper_time = upper_time + 0x100000000;
    }   // add carry to upper word
    old_time = new_time;
    irq_unlock(lock_num);

    time_float = (float)(upper_time + new_time)/(float) CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
    return(time_float);
}

uint32_t get_time(void)
{  
//    time64 = k_cycle_get_64();

    return(k_cycle_get_32());
}

/* careful for timer overflow - need to change to 64 bit*/
uint32_t get_time_diff(void)
{ static uint32_t old_time = 0;
    uint32_t new_time;
    uint32_t cycles_spent;

    /* capture initial time stamp */
    new_time = k_cycle_get_32();

    /* compute how long the work took (assumes no counter rollover) */
    cycles_spent = new_time - old_time;
    old_time = new_time;
    return(cycles_spent);

}

/* careful for timer overflow - need to change to 64 bit*/
/* add time to print queue (use floating point)*/
void print_time(void)
{   char string[80];

    uint32_t new_time;
    uint32_t time_fix;
//    uint64_t new_time64;
    double time_float;

/* capture initial time stamp */
    new_time = k_cycle_get_32(); //get number of cycles
    time_float = (double) new_time / (double) CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
    time_fix = (long) 1e6*time_float;
    snprintf(string, 80,"# k_cycle_get_32. newtime= %u time_fix = %u, t=%g sec. \n", 
            new_time, time_fix, time_float);
    printk("%s",string);
    // printq_add(string);
    

   /* new_time64 = k_cycle_get_64();
    snprintf(string, 80,"# k_cycle_get_64. newtime64= %llu \n", 
            new_time64);
    printk("%s",string);
    */
    return;

}