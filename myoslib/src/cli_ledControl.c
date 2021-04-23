/**
******************************************************************************
* @file      myoslib/src/cli_ledControl.h
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

#include "cli_ledControl.h"
#include "os_logMessageDriver.h"

/* Global variables */
/* The device ids or handles for the red, green and blue leds. */
const struct device *devr, *devb, *devg, *dev;

/* External function to control the leds based on user input.*/
void cli_led_init()
{

    int ret1, ret2, ret3;

    /* Get led device binding handles */
    devr = device_get_binding(LED1);
    devb = device_get_binding(LED3);
    devg = device_get_binding(LED2);

    /* Check if the device exists */
    if ((devr == NULL) || (devb == NULL) || (devg == NULL)) {

        printk("Device binding for leds failed");    
        return;
    }

    /* Configure the gpio pins for the leds. */
    ret1 = gpio_pin_configure(devr, PIN_LED1, GPIO_OUTPUT_ACTIVE | FLAGS_LED1);
    ret2 = gpio_pin_configure(devb, PIN_LED3, GPIO_OUTPUT_ACTIVE | FLAGS_LED3);
    ret3 = gpio_pin_configure(devg, PIN_LED2, GPIO_OUTPUT_ACTIVE | FLAGS_LED2);

    /* Check if the configuration has succeeded. If not return to main program*/
    if ((ret1 < 0) || (ret2 < 0) || (ret3 < 0)) {

        printk("Configuration of GPIOs for LEDs failed");
        return;

    }

    /* GPIO is ready to communicate with LEDs. Initialise the leds to off state. */
    gpio_pin_set(devr, PIN_LED1, (int)false);    
    gpio_pin_set(devb, PIN_LED3, (int)false);
    gpio_pin_set(devg, PIN_LED2, (int)false);

}

/* @brief : Shell command handler to process user input and turn on, turn off or toggle the required led.
*  Usage  : led o/f/t r/g/b - Turn on/off or toggle the red/gree/blue leds
*
* External functions used :  Uses the log_app_message() function to log an error message with the log message driver.
*/
static int cmd_led_control(const struct shell *shell, size_t argc, char **argv)
{
    /* Structure used by the log message driver to log messages. It is used here to log error messages when wrong options are presented by the user.*/
    struct message_data error_msg;

    /* Initlialise the message structure to be sent to the log message driver. Only the error message is going to be used for wrong options.*/
    error_msg.log_mesg = ERROR;

    /* If options are wrong, then print the correct usage.*/
    error_msg.desc = "The options are wrong.\nUsage: led o/f/t r/g/b\n For turning on or off, red/green/blue leds\n";

	if (argc == 3) {
        if (strcmp(argv[1],"o") == 0) {

            if (strcmp(argv[2],"r") == 0) {

                gpio_pin_set(devr, PIN_LED1, true);
            }
            else if (strcmp(argv[2],"g") == 0) {

                gpio_pin_set(devg, PIN_LED2, true);
            }
            else if (strcmp(argv[2],"b") == 0) {

                gpio_pin_set(devb, PIN_LED3, true);
            }
            else {

                /* Call the log message driver function to log the error message to the shell */
                os_log_app_message(&error_msg);
            }
        }
        else {
            if (strcmp(argv[1],"f") == 0) {

                if (strcmp(argv[2],"r") == 0) {

                    gpio_pin_set(devr, PIN_LED1, false);

                }
                else if (strcmp(argv[2],"g") == 0) {
                  
                    gpio_pin_set(devg, PIN_LED2, false);

                }
                else if (strcmp(argv[2],"b") == 0) {
                  
                    gpio_pin_set(devb, PIN_LED3, false);
                }
                else {

                    /* Call the log message driver function to log the error message to the shell */
                    os_log_app_message(&error_msg);

                }
            }
            else {

                if (strcmp(argv[1],"t") == 0) {

                    if (strcmp(argv[2],"r") == 0) {                        

                        gpio_pin_toggle(devr, PIN_LED1);

                    }
                    else if (strcmp(argv[2],"g") == 0) {
                     
                        gpio_pin_toggle(devg, PIN_LED2);

                    }
                    else if (strcmp(argv[2],"b") == 0) {
       
                        gpio_pin_toggle(devb, PIN_LED3);

                    }
                    else {

                       /* Call the log message driver function to log the error message to the shell */
                        os_log_app_message(&error_msg);

                    }
                }
                else {

                    /* Call the log message driver function to log the error message to the shell */
                    os_log_app_message(&error_msg); 

                }

            } 
                   
        }  
    }     
    return 0;
}

/* Register shell command to control the leds */
SHELL_CMD_REGISTER(led, NULL, "Control RGB LED", cmd_led_control);
