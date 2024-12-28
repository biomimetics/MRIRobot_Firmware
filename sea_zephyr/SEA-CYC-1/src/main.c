/* main.c - starts up threads in an orderly fashion
 * then basically sleeps .
 * started April 2023 by R. Fearing copying from EE192 2021 skeleton
 * converting FreeRTOS to Zephyr RTOS
 */

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <console/console.h>
#include <drivers/gpio.h>

// #include <device.h>
// #include <devicetree.h>
// #include <drivers/gpio.h>

#include "stm32_std.h"

#include "motor.h"
#include "test.h"
#include "encoder.h"
#include "pid.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void printq_add(char *msg);
extern int led_init();
extern void led_toggle();
extern void dac_init(void);
extern void adc_init(void);
extern void adc_test(void);
extern void pwm_init(void);
extern void pwm_test(void);
extern void hx711_init(void);
extern void qdec_init(void);
extern void hx711_test(void);
extern void start_hx711(void);
extern void start_print_thread(void);
extern void start_heartbeat(void);
extern void thread_info(void);
extern void start_control(void);
extern void uart_interrupt_init();
extern void start_uart_input(void);
extern void start_print_state(void);

void startup();

/*******************************************************************************
 * Define ports to be used
 ******************************************************************************/

// Set Other Ports

/*******************************************************************************
 * Define Testing Frequencies
 ******************************************************************************/
// Frequencies to test
const long num_tests = 11; // manually change this value for testing
// double freqs[15] = {0.5};

//+-----------------------------------+
//|            Main Code              |
//+-----------------------------------+

void main(void)
{
	startup();

	printk("beginning CB_Motor_test\n");
	if (setupMotors())
	{
		printk("\nMotor controller setup failed!\n");
		return;
	}

	int count = 30000;
	setup_pid();
	set_target(0);
	int target = 0;

	k_msleep(5000);

	while (1)
	{
		if (count >= 30000)
		{
			// Impedance Control Test
			// runImpedanceTest(60.0, 120.0, 0.2, 0.01);

			// Force Control Test
			// runImpedanceTest(60.0, 120.0, 0.0,  0.0);
			
			// Speed Control Test
			runSpeedTest(60 * 6);
		}
	}
}

void startup()
{

	long a = 0;
	char string[80];
	// use # as first character for comment to ignore
	// printk("# Hello World 7/11/23 from main()\n");

	if (led_init())
		printk("# led_init: success. Wait for LED blink\n");
	else
		printk("# led_init: failed.\n");

	while (a < 50)
	{
		led_toggle();
		k_msleep(SLEEP_TIME_MS / 10);
		a = a + 1;
	}

	start_print_thread();
	// printq_add("# printq test message- hello from main.c \n");
	// snprintf(string, 80, "# Clock cycles per second %d\n", CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);
	// printq_add(string);
	k_msleep(250); // suspend main() to allow print to run
				   // snprintf(string, 80, "# float number %f\n", 3.14159);
				   // printk("%s", string);
				   /* Peripheral initialization */
	dac_init();
	adc_init();
	//	adc_test();
	pwm_init();
	//	pwm_test();
	hx711_init();
	//	hx711_test();
	qdec_init();

	/* thread starting*/

	// Heartbeat interferes with ADC output
	// start_heartbeat();
	// k_msleep(10);
	// thread_info(); // get thread info for debugging
	uart_interrupt_init();
	start_uart_input();
	// k_msleep(10);
	// thread_info(); // get thread info for debugging
	// printq_add("# main.c after start_uart_input\n");
	start_hx711();
	// k_msleep(10);
	// thread_info(); // get thread info for debugging
	// start_control();

	// start_print_state();
	k_msleep(250); // suspend main() to allow print to run
	thread_info(); // get thread info for debugging
				   // k_msleep(10);
	// thread_info(); // get thread info for debugging
	k_msleep(4 * SLEEP_TIME_MS); // wait to see what time threads have used
								 // thread_info();				 // get thread info for debugging
								 // printq_add("# main.c after thread_info()\n");
								 // printq_add("# STM32READY\n"); // python script should wait for this before starting
}
