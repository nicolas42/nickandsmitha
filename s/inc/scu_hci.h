/**
******************************************************************************
* @file      apps/s/inc/scu_hci.h
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     HCI packet structure and information used by the application.
*         REFERENCE: B-L475e-IOT01A Platform datasheet.
*****************************************************************************
*                 EXTERNAL FUNCTIONS
*****************************************************************************
* void scu_init() - Initialises the scu thread and the spi device in the slave 
*                   mode. 
*                   Initialises the I2C device and the sensors. 
*****************************************************************************
*/
#ifndef SCU_HCI_H
#define SCU_HCI_H
#include <stdlib.h>
#include <stdint.h>


enum sid {
    NOREG,
    LSM6DSL,
    LIS3MDL,
    LPS22HB,
    VL53L0X,
    HTS221, 
    HCSR04
};

enum transfer_type {
    NONE,
    REQUEST = 0x01,
    RESPONSE = 0x02
};

struct packet_st {
    uint8_t sid;
    uint8_t i2c_addr_rw_bit;
    uint8_t reg_addr;
    uint8_t reg_data[16];
};

struct hci_packet {
    uint8_t preamble;
    enum transfer_type tt : 4;
    uint8_t payload_length : 4;
    struct packet_st payload;
};

extern void scu_init();
#endif