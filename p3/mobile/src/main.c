/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>



// static uint8_t mfg_data[] = { 0x25, 0x92, 0xEE, 0xD6, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };

// static const struct bt_data ad[] = {
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 10),
// };

// static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
// 		    struct net_buf_simple *buf)
// {
// 	mfg_data[4]++;
// }





// static uint8_t mfg_data[20] = { 0x00 };

// static const struct bt_data ad[] = {
// 	{
// 		.type = 0xFF,
// 		.data_len = 20,
// 		.data = (const uint8_t *) mfg_data
// 	}
// };




// static uint8_t mfg_data[] = { 0xff, 0xff, 0x00 };

// static const struct bt_data ad[] = {
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 3),
// };



// #ifndef IBEACON_RSSI
// #define IBEACON_RSSI 0xc8
// #endif

/*
 * Set iBeacon demo advertisement data. These values are for
 * demonstration only and must be changed for production environments!
 *
 * UUID:  18ee1516-016b-4bec-ad96-bcb96d166e97
 * Major: 0
 * Minor: 0
 * RSSI:  -56 dBm
 */
// 1A FF 4C 00 02 15 D4 84 93 2C 33 A3 49 59 A5 65 F8 13 31 AA 26 FB FF FF 00 01 BF 


// static uint8_t mfg_data[25] = { 0x00 };
// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 25),
// };












static uint8_t mfg_data[29] = { 0x00 };

// 31 bytes max including 2 byte preamble
// 0xFF means it's a manufacturer's message why not
static const struct bt_data ad[] = {
	{
		.type = 0xFF,
		.data_len = 29,
		.data = (const uint8_t *) mfg_data
	}
};

int id_is_equal(uint8_t *data, uint8_t d6, uint8_t d7, uint8_t d8, uint8_t d9)
{
	return (data[6] == d6) && (data[7] == d7) && (data[8] == d8) && (data[9] == d9);
}


static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{

	// marshall data 

	mfg_data[0] = 0x63; // N 
	mfg_data[1] = 0x73; // S
	mfg_data[2] = 0x00; // 0
	mfg_data[3] = 0x00; // 0

	// NS1
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x01)){
		mfg_data[4] = rssi;
		mfg_data[5] = buf->data[10];
	}
	// NS2
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x02)){
		mfg_data[6] = rssi;
		mfg_data[7] = buf->data[10];
	}
	// NS3
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x03)){
		mfg_data[8] = rssi;
		mfg_data[9] = 0x00;
	}
	// NS4
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x04)){
		mfg_data[10] = rssi;
		mfg_data[11] = 0x00;
	}
	// NS5
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x05)){
		mfg_data[12] = rssi;
		mfg_data[13] = 0x00;
	}
	// NS6
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x06)){
		mfg_data[14] = rssi;
		mfg_data[15] = 0x00;
	}
	// NS7
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x07)){
		mfg_data[16] = rssi;
		mfg_data[17] = 0x00;
	}
	// NS8
	if (id_is_equal(buf->data, 0x63,0x73,0x00, 0x08)){
		mfg_data[18] = rssi;
		mfg_data[19] = 0x00;
	}


}

void main(void)
{
	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x0010,
		.window     = 0x0010,
	};
	int err;

	printk("Starting Scanner/Advertiser Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");



	// scan for ads
	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}


	// advertise
	do {
		k_sleep(K_MSEC(10));

		/* Start advertising */
		err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
				      NULL, 0);
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}

		k_sleep(K_MSEC(10));

		err = bt_le_adv_stop();
		if (err) {
			printk("Advertising failed to stop (err %d)\n", err);
			return;
		}
	} while (1);

}
