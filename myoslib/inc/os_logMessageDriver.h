/**
**************************************************************************************
* @file      myoslib/inc/os_logMessageDriver.h
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

#ifndef OS_LOG_MESSAGE_DRIVER_H
#define OS_LOG_MESSAGE_DRIVER_H

#include <kernel.h>
#include <device.h>
#include <sys/printk.h>
#include <string.h>
#include <sys/mutex.h>
#include <shell/shell.h>
#include <sys/libc-hooks.h>


/* Enumeration of error types */
enum log_type {
    NONE,
    ERROR,
    LOG,
    DEBUG
};

/* Message structure for the log data */
struct message_data {
    enum log_type log_mesg;

    /* String to describe the message */
    const char* desc;
};

extern void os_log_message_driver_init();
extern void os_log_message_driver_deinit();
extern void os_log_app_message(struct message_data* msg);

#endif