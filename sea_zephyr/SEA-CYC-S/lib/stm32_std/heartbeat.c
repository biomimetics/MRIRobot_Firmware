/* heart beat task. Can also use for watchdog timer keep alive*/
#include <zephyr.h>
#include <sys/printk.h>

/* size of stack area used by each thread */
#define STACKSIZE 128
/* scheduling priority - larger = lower priority */
#define PRIORITY 32
#define BEATTIME 500 // in ms

extern void led_toggle();

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void heartbeat_thread(void);
void start_heartbeat(void);

K_THREAD_STACK_DEFINE(heartbeat_stack_area, STACKSIZE);
static struct k_thread heartbeat_thread_data;


void start_heartbeat()
{
    
    printk("# Starting heartbeat thread\n");
/* spawn heartbeat thread */
	k_tid_t tid = k_thread_create(&heartbeat_thread_data, heartbeat_stack_area,
			K_THREAD_STACK_SIZEOF(heartbeat_stack_area), (k_thread_entry_t) heartbeat_thread, 
            NULL, NULL, NULL, PRIORITY, 0, K_FOREVER);

	k_thread_name_set(tid, "heartbeat");

	k_thread_start(&heartbeat_thread_data);

}

void heartbeat_thread(void)
{   // CAUTION: stack is too small to use printk
    while(1)
    {   k_msleep(BEATTIME); // 
        led_toggle();
    }
}