/**
**************************************************************************************
* @file      myoslib/src/os_hci.h
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Provides an interface for HAL functions to queue spi slave messages.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* os_hci_init() - Initialises the queues, semaphores and thread stacks.
*                                Creates the threads.
* os_hci_deinit() - Stops the threads, cleans the message queues.
* os_process_hci_packet - Provides the interface to the HAL functions to queue the slave 
*                         responses for further processing messages.
***************************************************************************************
*/
#ifndef OS_HCI_H
#define OS_HCI_H

#include <kernel.h>
#include <device.h>
#include <sys/printk.h>
#include <string.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <shell/shell.h>
#include <sys/libc-hooks.h>
#include "hci_packet.h"

extern void os_hci_init();
extern void os_hci_deinit();
extern void os_process_hci_packet(struct hci_packet msg);
extern void os_get_accel_values(struct accel* a);

#define ADV_LEN 12
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// /* Advertising data */
// uint8_t manuf_data[ADV_LEN] = {
// 	0x01 /*SKD version */,
// 	0x83 /* STM32WB - P2P Server 1 */,
// 	0x20 /* GROUP A Feature  */,
// 	0x30 /* GROUP A Feature */,
// 	0x40 /* GROUP B Feature */,
// 	0x50 /* GROUP B Feature */,
// 	0x60, /* BLE MAC start -MSB */
// 	0x70,
// 	0x00,
// 	0x00,
// 	0x00,
// 	0x00 /* BLE MAC stop */
// };

// /* Set Scan Response data */
// const struct bt_data sd[] = {
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// };

// const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
// };

#endif