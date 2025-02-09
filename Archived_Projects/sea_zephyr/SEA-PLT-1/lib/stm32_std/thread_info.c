#include <zephyr.h>
#include <sys/printk.h>

// prototypes
void thread_info_print(k_tid_t, void *);
void thread_info(void);

void thread_info_print(k_tid_t thread_id, void *user_data)
{ 
    char thread_name[80];
	int err;
	const char *state_msg; // pointer to a constant string
	int priority;
	int stack_ptr, stack_size;
	long long unsigned int cycles; 
	
   // k_tid_t thread_id;
	// thread_id = k_current_get();
	err = k_thread_name_copy(thread_id, thread_name, sizeof(thread_name));
	if (err)
	{	printk("# thread name copy error %d\n", err);
	}
	state_msg = k_thread_state_str(thread_id); // CAUTION: possible string overflow!
	
	priority = k_thread_priority_get(thread_id);
	stack_ptr = thread_id ->stack_info.start;
	stack_size = thread_id ->stack_info.size;

	k_thread_runtime_stats_t rt_stats_thread;
 	k_thread_runtime_stats_get(thread_id, &rt_stats_thread);
	cycles = rt_stats_thread.execution_cycles;

	printk("# %16s  %10s %8d \t%x \t%d \t %llu\n", 
			thread_name, state_msg, priority, stack_ptr, stack_size, cycles);
}


void thread_info(void)
{	void *cb_data[80]; 

	printk("# Name                State     Priority   Stack         size     cycles\n");
	k_thread_foreach((k_thread_user_cb_t) thread_info_print, cb_data);

}