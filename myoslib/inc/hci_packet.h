/**
******************************************************************************
* @file      myoslib/inc/hal_packet.h
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Provides the hci spi transport layer.
*         REFERENCE: zephyr shell library api and documentation.
******************************************************************************
*/
#ifndef HCI_PACKET_H
#define HCI_PACKET_H
#include <stdint.h>

enum sid {

    NOREG,
    LSM6DSL = 1,
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

/* Function for easy printing */
static inline const char* getstring(uint8_t sidval) {
    switch(sidval) {
        case LSM6DSL:
            return "LSM6DSL";
        default:
            return "";
    }
}

struct accel {
  
    double x;
    double y;
    double z;

};

#endif