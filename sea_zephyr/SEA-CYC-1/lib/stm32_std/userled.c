/* driver interface use user LED
* connected to GPIO PA5 on STM32 F446RE nucleo board
*/

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include <stdlib.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif


const struct device *LEDdev;  // structure for LED device

int led_init()
{ 
	int ret;

	LEDdev = device_get_binding(LED0);
	if (LEDdev == NULL) {
		return(0);
	}

	ret = gpio_pin_configure(LEDdev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return(0);
	}
	printk("# LED0 initialized PA5\n");
	return(1); // success
}
void led_toggle(void)
{ static bool led_is_on; 
	led_is_on = gpio_pin_get(LEDdev, PIN);
	led_is_on = !led_is_on;
	gpio_pin_set(LEDdev, PIN, (int)led_is_on);
}	



