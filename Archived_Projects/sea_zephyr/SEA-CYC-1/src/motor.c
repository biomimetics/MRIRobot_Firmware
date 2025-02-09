#include "motor.h"

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

// ------------ Motor Set Ports ------------
#define SET_DIR DT_ALIAS(mdir0)
#define SET_DIR_LABEL DT_GPIO_LABEL(SET_DIR, gpios)
#define SET_DIR_PIN DT_GPIO_PIN(SET_DIR, gpios)

#define SET_EN DT_ALIAS(d13)
#define SET_EN_LABEL DT_GPIO_LABEL(SET_EN, gpios)
#define SET_EN_PIN DT_GPIO_PIN(SET_EN, gpios)

// ------------ Device Variables ------------
const struct device *set_dir_dev;
const struct device *set_en_dev;

int setupMotors() {

    // +-----------------------------------+
    // | Setup GPIO devices                |
    // +-----------------------------------+

    // Setup Control Devices
    set_dir_dev = device_get_binding(SET_DIR_LABEL);
    gpio_pin_configure(set_dir_dev, SET_DIR_PIN, GPIO_OUTPUT);
    if (!set_dir_dev) {
        printk("\nCannot find SET_DIR device!\n");
        while(1);
    }

    set_en_dev = device_get_binding(SET_EN_LABEL);
    gpio_pin_configure(set_en_dev, SET_EN_PIN, GPIO_OUTPUT);
    if (!set_en_dev)
    {
        printk("\nCannot find EN_SWITCH device!\n");
        return;
    }

    dac_init();

    return 0;
}

void setMotorSpeed(double speed) {
    // printk("%d\n", (int) speed);

    if (speed > 100) {
        speed = 100;
    } else if(speed < -100) {
        speed = -100;
    }

    int converted_speed = ((speed > 0 ? speed : -speed) * 4095) / 100;

    gpio_pin_set(set_dir_dev, SET_DIR_PIN, (speed < 0));
    gpio_pin_set(set_en_dev, SET_EN_PIN, (speed > 0 & speed < -0));
    set_dac(converted_speed);
}

void testMotor() {

    int i = 0;
    for (i = 0; i < 4096; i++)
    {
        setMotorSpeed(i);
        getCount();
        printk("%d\n", i);
        k_sleep(K_SECONDS(0.005));
    }

    for (i = 4096; i > 0; i--)
    {
        setMotorSpeed(i);
        getCount();
        printk("%d\n", i);
        k_sleep(K_SECONDS(0.005));
    }
}


