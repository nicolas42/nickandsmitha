/**
******************************************************************************
* @file      apps/p2/src/main.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     The main thread. 
*            Initialises the USB device and the os_hci. 
*         REFERENCE: zephyr shell library api and documentation.
*****************************************************************************
*/

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <stdlib.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>
//#include "os_hci.h"
#include <ctype.h>
#include <logging/log.h>
#include <bluetooth/bluetooth.h>
// #include <bluetooth/hci.h>
// #include <bluetooth/conn.h>
// #include <bluetooth/uuid.h>
// #include <bluetooth/gatt.h>

LOG_MODULE_REGISTER(main);

 #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
 #define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define ADV_LEN 12
//#define IBEACON_RSSI 0xb4

/* Advertising data */

static uint8_t manuf_data[ADV_LEN] = {
	0x01 /*SKD version */,
	0x83 /* STM32WB - P2P Server 1 */,
	0x20 /* GROUP A Feature  */,
	0x30 /* GROUP A Feature */,
	0x63 /* GROUP B Feature */,
	0x73 /* GROUP B Feature */,
	0x00, /* BLE MAC start -MSB */
	0x07,
	0x00,
	0x00,
	0x00,
	0x00 /* BLE MAC stop */
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data ad[] = {
 	//BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
 	//BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
 	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
};

void main(void)
{
	const struct device *dev = NULL;
	int ret;
   	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;


    /* Initialise the USB driver */
	/* Check if device is available and accessible */
	dev = device_get_binding("CDC_ACM_0");

	if (!dev) {

		printk("USB device not found");
		return;
	}

    /* Enable the usb device */
	ret = usb_enable(NULL);

	if (ret != 0) {
		printk("Failed to enable USB");
		return;
	}

       // os_hci_init();
    
        int err = bt_enable(NULL);
        	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	printk("%d",BT_LE_ADV_NCONN_IDENTITY);
	if (err) {
		printk("Advertising failed to start (err %d)", err);
		return;
	}

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));
    while(1) {
	printk("StaticP3 started, advertising as %s\n", addr_s);
	k_msleep(100);
	}

}
