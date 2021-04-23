/**
**************************************************************************************
* @file      myoslib/inc/os_logMessageDriver.c
* @author    Smitha Ratnam - 44425782
* @date      15032021
* @brief     Provides an interface for CLI functions to log messages.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* os_log_message_driver_init() - Initialises the queues, semaphores and thread stacks.
*                                Creates the threads.
* os_log_message_driver_deinit() - Stops the threads, cleans the message queues.
* os_log_app_message() - Provides the interface to the CLI functions to log messages.
***************************************************************************************
*/

#include <shell/shell_uart.h>
#include "os_logMessageDriver.h"

/* scheduling priority used by each thread */
#define PRIORITY 7

#define RX_THREAD_STACK_SIZE 512
#define TX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define TX_THREAD_PRIORITY 2

/* Global Variables */

/* Handles for the threads */
k_tid_t consumer_id;
k_tid_t producer_id;

/* Thread data structures to be used by the macros for defining threads */
struct k_thread rx_thread_data;
struct k_thread tx_thread_data;


/* Message structure definition for the log message. */
/* @brief - received_msg is populated by the consumer thread while tx_data is populated by the external function called by the cli peripheral */
struct message_data received_msg, tx_data;

/* Define the semaphores, queues, thread stacks. Being atomic, these need to be called outside the function */

/* Semaphore for accessing the queue between threads */
K_SEM_DEFINE(log_queue_sem, 0, 1);

/* Semaphore for access the data between the external function and the queue to transmit */
K_SEM_DEFINE(log_data_sem, 0, 1);

/* Define the common queue to send and receive log messages */
K_MSGQ_DEFINE(log_msg_queue, sizeof(struct message_data), 10, 4);

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);

/* Function definitions */

void os_log_message_driver_deinit(void) {

    k_msgq_cleanup(&log_msg_queue);

}    


/* internal function to initialise the fields of the message structure used for sending data to the queue */
void initialise_tx(void) {

    tx_data.desc = "";
    tx_data.log_mesg = NONE;

}

/* @brief - consumer thread to consume the messages from the queue. 
* Uses the semaphore log_queue_sem to access the queue which is a shared resource.
*/
void consumer(void* arg1, void* arg2, void* arg3) {

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct message_data rx_data;
    
    while(1) {

        rx_data.log_mesg = NONE;
        rx_data.desc = "";

        if (k_sem_take(&log_queue_sem, K_MSEC(50)) == 0) {
        
            k_msgq_get(&log_msg_queue, &rx_data, K_FOREVER);
               
            
            received_msg.log_mesg = rx_data.log_mesg;
            received_msg.desc = rx_data.desc;

            /* get shell pointer to access the shell and print the messages.*/
            /* Pointer to the shell */
            const struct shell *log_shell = shell_backend_uart_get_ptr();

            switch(rx_data.log_mesg) {

                case ERROR:
                    shell_error(log_shell, "%s\n", rx_data.desc);
                    break;

                case DEBUG:
                    shell_print(log_shell, "%s\n", rx_data.desc);
                    break;

                case LOG:
                    shell_info(log_shell, "%s\n", rx_data.desc);
                    break;

                case NONE:
                    break;

            }
            
            k_msleep(100);
        }  
    }
}

/* External function used by cli functions to log the messages to the queue */
void os_log_app_message(struct message_data* msg) {

    tx_data.log_mesg = msg->log_mesg;

    tx_data.desc = msg->desc;   

    /* Release semaphore for data access */
    k_sem_give(&log_data_sem);

}

/* Producer thread to post messages into the common queue */
void producer(void* arg1, void* arg2, void* arg3) {

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    initialise_tx();
    while(1) {
      
        /* wait for semaphore to access the data */
        if (k_sem_take(&log_data_sem, K_MSEC(50)) == 0) {
          
            /* put message into queue */
            while(k_msgq_put(&log_msg_queue, &tx_data, K_NO_WAIT) != 0) {

                k_msgq_purge(&log_msg_queue);
            }

            /* initialise the structure to avoid contiuous printing. */
            initialise_tx();

            /* Release semaphore for queue access */
            k_sem_give(&log_queue_sem);

        }
    
        k_msleep(100);

    }
}

/* Initilise the threads and start them */
void os_log_message_driver_init(void) {

    /* create and start consumer thread and get its handle */
    consumer_id = k_thread_create(&rx_thread_data, rx_thread_stack,
				 K_THREAD_STACK_SIZEOF(rx_thread_stack),
				 consumer, NULL, NULL, NULL,
				 RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (!consumer_id) {

		printk("ERROR spawning RX thread\n");
        return;
	}

    /* create and start the producer thread and get its handle */
    producer_id = k_thread_create(&tx_thread_data, tx_thread_stack,
				 K_THREAD_STACK_SIZEOF(tx_thread_stack),
				 producer, NULL, NULL, NULL,
				 TX_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (!producer_id) {

		printk("ERROR spawning TX thread\n");
        return;

	}
    
}

/* Shell function to filter out the messages as error, debug or log */
/* Usage : mfilter e/d/l - filters the error or debug or log messages respectively.*/
static int  cmd_log_message_handler(const struct shell* shell, size_t argc, char** argv) {

    ARG_UNUSED(argc);
     
    if (strcmp(argv[1], "e") == 0) {

        if(received_msg.log_mesg == ERROR) {

                shell_error(shell, "%s", received_msg.desc);
        }

    }
    else {

        if(strcmp(argv[1], "l") == 0) {

            if(received_msg.log_mesg == LOG) {

                shell_info(shell, "%s",received_msg.desc);

            }

        }    
        else {

            if(strcmp(argv[1],"d") == 0) {

                if(received_msg.log_mesg == DEBUG) {

                    shell_print(shell,"%s", received_msg.desc);
                }

            }
            else {

                shell_print(shell, "%s", received_msg.desc);      
            }
        }
    }

    return 0;

}

/* Register the command with the shell */
SHELL_CMD_REGISTER(mfilter, NULL, "Filter log message by type", cmd_log_message_handler);