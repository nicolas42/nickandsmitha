/**
**************************************************************************************
* @file      apps/s/src/scu_lsm6dsl.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Initialises the Accelerometer/Gyrometer device.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* initialise_lsm6dsl() - initialises the lsm6dsl sensor device.
***************************************************************************************
*/

#include "scu_common.h"
#include <kernel.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#define GPIO_NODE DT_NODELABEL(gpioc)
#define RANGER_TRIG_PIN 5
#define RANGER_ECHO_PIN 4

#define TIMEOUT 11600

enum pin_state {
    IDLE,
    FALLING_EDGE, 
    RISING_EDGE
};

static const struct device *dev_hcsr04;
static struct gpio_callback echo_data;
static uint32_t echo_start_time, echo_end_time;

static uint32_t us_dist;
static enum pin_state echo_pin_state = IDLE;

void calculate_dist() {
    uint32_t count  = 0;
    uint32_t count_in_us = 0;
    us_dist = 0;

    count = echo_end_time - echo_start_time;
    count_in_us = k_cyc_to_us_floor64(count);

    if(count_in_us < TIMEOUT) {
        //us_dist = (k_cyc_to_us_floor64(count))/58;
        us_dist = count_in_us/58;
    }
    
 //   printk("Distance measured = %d\n", us_dist);
}

uint32_t get_distance() {
    return us_dist;
}

void input_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    
    if(gpio_pin_get(dev_hcsr04, RANGER_ECHO_PIN) == 1) {
        echo_start_time = k_cycle_get_32();  
        echo_pin_state = RISING_EDGE;
     
    } 
    else if ((gpio_pin_get(dev_hcsr04, RANGER_ECHO_PIN) == 0) && (echo_pin_state == RISING_EDGE)) {
        echo_end_time = k_cycle_get_32();
        echo_pin_state = FALLING_EDGE;

        calculate_dist();
    }

}

void set_trigger() {
    gpio_pin_set(dev_hcsr04, RANGER_TRIG_PIN, 1 );
    k_busy_wait(11);
    gpio_pin_set(dev_hcsr04, RANGER_TRIG_PIN, 0);
 //   k_msleep(1);
}

void initialise_hcsr04() {
  
    dev_hcsr04 = device_get_binding(DT_LABEL(GPIO_NODE));
    
    gpio_pin_configure(dev_hcsr04, RANGER_TRIG_PIN, GPIO_OUTPUT);
    gpio_pin_configure(dev_hcsr04, RANGER_ECHO_PIN, GPIO_INPUT);
    gpio_pin_set(dev_hcsr04, RANGER_TRIG_PIN, 0);
    gpio_pin_set(dev_hcsr04, RANGER_ECHO_PIN, 0);
    gpio_pin_interrupt_configure(dev_hcsr04, RANGER_ECHO_PIN, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&echo_data, input_changed, BIT(RANGER_ECHO_PIN));
    gpio_add_callback(dev_hcsr04, &echo_data);    
}

