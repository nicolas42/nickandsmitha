/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>


#include <zephyr.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <string.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7



static void start_scan(void);

static struct bt_conn *default_conn;

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	// if (default_conn) {
	// 	return;
	// }

	// /* We're only interested in connectable events */
	// if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	//     type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
	// 	return;
	// }

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	// printk("Device found: %s (RSSI %d)\n", addr_str, rssi);


	// if ((strncmp(addr_str, "C3:DC:25:A7:9A:AB", strlen("C3:DC:25:A7:9A:AB"))==0) || 
	// (strncmp(addr_str, "EE:CD:D5:D4:09:A7", strlen("EE:CD:D5:D4:09:A7"))==0)){

	// two byte preamble
	if (   (ad->data[2] == 0x63) 
		&& (ad->data[3] == 0x73) 
		&& (ad->data[4] == 0x00) 
		&& (ad->data[5] == 0x00)){
	
		// for (int i=0; i<ad->len; i+=1){
		// 	printk("%02X ", ad->data[i]);
		// }

		// output json to serial
		printk("[[%d,%u],[%d,%u],[%d,%u],[%d,%u]]\n", 
		(int8_t)ad->data[6], (uint8_t)ad->data[7], 
		(int8_t)ad->data[8], (uint8_t)ad->data[9], 
		(int8_t)ad->data[10], (uint8_t)ad->data[11],
		(int8_t)ad->data[12], (uint8_t)ad->data[13]);

	}


	// for (int i=0; i<ad->len; i+=1){
	// 	printk("%02X ", ad->data[i]);
	// }
	// printk("\n");


	// printk("device %s (RSSI %d)\n", addr_str, rssi);

	// }

	// k_sleep(K_MSEC(100));

	// for (int i=0; i<ad->len; i+=1){
	// 	printk("%02X ", ad->data[i]);
	// }

// 02 01 06 09 09 53 54 41 54 49 43 50 33 0D FF 01 83 20 30 40 50 60 70 00 00 00 00 rssi < -70
// 02 01 06 09 09 53 54 41 54 49 43 50 33 0D FF 01 83 20 30 40 50 60 70 00 00 00 00 rssi < -70

// Device found: 7D:97:17:C3:B5:48 (random) (RSSI -87)
// 1A FF 4C 00 02 15 D4 84 93 2C 33 A3 49 59 A5 65 F8 13 31 AA 26 FB 00 03 00 01 BF rssi < -70

// Device found: 50:95:C5:DA:A9:AD (random) (RSSI -83)
// 1A FF 4C 00 02 15 D4 84 93 2C 33 A3 49 59 A5 65 F8 13 31 AA 26 FB 00 02 00 01 BF rssi < -70


	/* connect only to devices in close proximity */
	if (rssi < -70) {
		// printk("rssi < -70\r\n");
		return;
	}

	if (bt_le_scan_stop()) {
		printk("bt_le_scan_stop\r\n");
		return;
	}

	start_scan();

	// err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
	// 			BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	// if (err) {
	// 	printk("Create conn to %s failed (%u)\n", addr_str, err);
	// 	start_scan();
	// }
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		// printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	// printk("Scanning successfully started\n");
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);

		bt_conn_unref(default_conn);
		default_conn = NULL;

		start_scan();
		return;
	}

	if (conn != default_conn) {
		return;
	}

	printk("Connected: %s\n", addr);

	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != default_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_scan();
}

static struct bt_conn_cb conn_callbacks = {
		.connected = connected,
		.disconnected = disconnected,
};

void main(void)
{

	// Initialise usb console
	{
		const struct device *dev = device_get_binding(
				CONFIG_UART_CONSOLE_ON_DEV_NAME);
		uint32_t dtr = 0;

		if (usb_enable(NULL)) {
			return;
		}

		/* Poll if the DTR flag was set, optional */
		while (!dtr) {
			uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		}

		if (strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME) !=
			strlen("CDC_ACM_0") ||
			strncmp(CONFIG_UART_CONSOLE_ON_DEV_NAME, "CDC_ACM_0",
				strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME))) {
			printk("Error: Console device name is not USB ACM\n");

			return;
		}

	}


	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);

	start_scan();
}
