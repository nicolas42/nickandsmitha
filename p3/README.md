# devices

thingy52_nrf52832 (mobile)
nrf52840dongle_nrf52840 (base)
disco_l475_iot1


# flash base (dongle) 
nrf52840dongle_nrf52840
from https://docs.zephyrproject.org/latest/boards/arm/nrf52840dongle_nrf52840/doc/index.html



rm -rf build; west build -b nrf52840dongle_nrf52840 && \
\
nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
    --application build/zephyr/zephyr.hex \
    --application-version 1 pkg.zip; \
nrfutil dfu usb-serial -pkg pkg.zip -p /dev/ttyACM0




https://github.com/csiro-wsn/contiki-examples


# flash mobile (thingy)

rm -rf build; west build -b thingy52_nrf52832; west flash



# blue st protocol
https://www.st.com/resource/en/user_manual/dm00550659-getting-started-with-the-bluest-protocol-and-sdk-stmicroelectronics.pdf



# errors

https://devzone.nordicsemi.com/f/nordic-q-a/42614/nordic-thingy-52-usb-port---can-that-be-used-as-a-serial-port-to-connect-to-a-pc

The USB is only for charging. The thingy:52 is based on the nRF52832 which does not support USB. 


The nRF52840 does, and Nordic showed off a new Thingy at CES with their cell module nRF9160, alongside a nRF52840. Hopefully that will be able to use the USB port for CDC as per your application. 



# bluetooth architecture

https://docs.zephyrproject.org/latest/guides/bluetooth/bluetooth-arch.html




The st_ble example is good base code but I'm pretty sure it requires a connection to be made and I don't know how to connect to a base.

Right now I have misc/{beacon, central} where the beacon is advertising some data and the central, thing, is printing it out.  Will that be enough for our purposes?  Probably if I can change hte information that is being advertised.



# Notes April 21

#define BT_UUID_HTS_VAL 0x1809
/** @def BT_UUID_HTS
 *  @brief Health Thermometer Service
 */
#define BT_UUID_HTS \
	BT_UUID_DECLARE_16(BT_UUID_HTS_VAL)
/** @def BT_UUID_DIS_VAL
 *  @brief Device Information Service UUID value


base main enables usb and bluetooth, sets up callbacks that fire on connection and disconnection and then starts scanning for bluetooth devices (scan_start).

if a device is found then the aptly named device_found function is fired.

I prints something like this 

[DEVICE]: 5C:DD:CC:88:C2:52 (random), AD evt type 4, AD data len 0, RSSI -82


Incidentally the thingy has this mac? address.
Connected: EE:CD:D5:D4:09:A7 (random)


when a device is found it prints out the device number, 
[DEVICE]: 5C:DD:CC:88:C2:52 (random), AD evt type 4, AD data len 0, RSSI -82



# Advertising using misc/{ibeacon, central}

Device found: EE:CD:D5:D4:09:A7 (random) (RSSI -72)
����WOOhyrprojecrssi < -70





if ((strcmp(addr_str, "C3:DC:25:A7:9A:AB")==0) | (strcmp(addr_str, "EE:CD:D5:D4:09:A7")==0)){
    // for (int i=0; i<ad->len; i+=1) printk("%c", ad->data[i]);
}


The current demo uses
phone ibeacon -> thingy misc/scan_adv -> dongle misc/central

The next thing to test is receiving from multiple beacons and possibly packaging their data.



# April 23

my phone

1A FF 4C 00 02 15 D4 84 93 2C 33 A3 49 59 A5 65 F8 13 31 AA 26 FB 00 01 00 02 BF rssi < -70


1A FF 4C 00 02 15 4C 19 CE E5 29 46 44 1E A4 4D 91 4E 4C 59 37 34 00 05 00 06 BF rssi < -70

02 01 06 09 09 53 54 41 54 49 43 50 33 0D FF 01 83 20 30 40 50 60 70 00 00 00 00 device C3:DC:25:A7:9A:AB (random) (RSSI -76)

