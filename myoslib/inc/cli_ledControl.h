/**
******************************************************************************
* @file      myoslib/inc/cli_ledControl.h
* @author    Smitha Ratnam - 44425782
* @date      15032021
* @brief     Controls the status RGB leds based on the command from the user. 
*         REFERENCE: zephyr shell library api and documentation.
*****************************************************************************
*                 EXTERNAL FUNCTIONS
*****************************************************************************
* cli_led_init() - initialise the RGB leds 
*****************************************************************************
*/

#ifndef CLI_LED_CONTROL_H
#define CLI_LED_CONTROL_H

#include <zephyr.h>
#include <shell/shell.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/* The devicetree node identifiers for the "led1", "led2", "led3" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)


#if DT_NODE_HAS_STATUS(LED0_NODE, okay) & DT_NODE_HAS_STATUS(LED1_NODE, okay) & DT_NODE_HAS_STATUS(LED2_NODE, okay) & DT_NODE_HAS_STATUS(LED3_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN_LED0	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS_LED0	DT_GPIO_FLAGS(LED0_NODE, gpios)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN_LED1	DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS_LED1	DT_GPIO_FLAGS(LED1_NODE, gpios)
#define LED2	DT_GPIO_LABEL(LED2_NODE, gpios)
#define PIN_LED2	DT_GPIO_PIN(LED2_NODE, gpios)
#define FLAGS_LED2	DT_GPIO_FLAGS(LED2_NODE, gpios)
#define LED3	DT_GPIO_LABEL(LED3_NODE, gpios)
#define PIN_LED3	DT_GPIO_PIN(LED3_NODE, gpios)
#define FLAGS_LED3	DT_GPIO_FLAGS(LED3_NODE, gpios)
#endif 

/* @brief   Function to initialise the leds so that they respond properly to the command line arguments.
*  return - returns if the device is not configured or if the binding fails. 
*/
extern void cli_led_init(void);

#endif