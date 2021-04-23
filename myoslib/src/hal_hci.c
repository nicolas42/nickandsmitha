/**
******************************************************************************
* @file      myoslib/inc/hal_hci.h
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Provides the hci spi transport layer.
*         REFERENCE: zephyr shell library api and documentation.
*****************************************************************************
*                 EXTERNAL FUNCTIONS
*****************************************************************************
* hal_hci_init() - initialise  spi structures and device as master.
*****************************************************************************
*/
#include <device.h>
#include <kernel.h>
#include <string.h>
#include <sys/printk.h>
#include <shell/shell.h>
#include <shell/shell_uart.h>
#include <drivers/spi.h>

#include "hci_packet.h"
#include "os_hci.h"

struct hci_packet hci_req_pkt, hci_rsp_pkt;
static struct spi_config spi_cfg;

#define SLAVE_NODE DT_NODELABEL(spi_dev)


const uint8_t HTS221_ADDR = 0x5E;
const uint8_t LIS3MDL_ADDR = 0x1D;
const uint8_t LPS22HB_ADDR = 0x5D;
const uint8_t LSM6DSL_ADDR = 0x6A;
const uint8_t VL53L0X_ADDR = 0x29;
const uint8_t OUTX_L_XL = 0x28;
const uint8_t OUTX_H_XL = 0x29;
const uint8_t OUTY_L_XL = 0x2A;
const uint8_t OUTY_H_XL = 0x2B;
const uint8_t OUTZ_L_XL = 0x2C;
const uint8_t OUTZ_H_XL = 0x2D;



#define HCI_RX_THREAD_PRIORITY 3
#define HCI_RX_THREAD_STACK_SIZE 512

K_SEM_DEFINE(hal_spi_sem, 0, 1);

K_THREAD_STACK_DEFINE(hci_rx_thread_stack, HCI_RX_THREAD_STACK_SIZE);


struct k_thread hci_rx_thread_data;
static bool tx_done = 0;
k_tid_t hal_hci_id;

/* Map i2c address to sid */
uint8_t get_i2c_addr(uint8_t sid_val) {

    switch(sid_val) {

        case LSM6DSL:
            return LSM6DSL_ADDR;
            break;
        
        case LPS22HB:
            return LPS22HB_ADDR;
            break;

        case HTS221:
            return HTS221_ADDR;
            break;

        case LIS3MDL:
            return LIS3MDL_ADDR;
            break;

        case VL53L0X:
            return VL53L0X_ADDR;
            break;

        default:
            break;

    }
    return LSM6DSL_ADDR;

}

/* shell command handler to read and write to i2c registers */
static int cmd_hci_handler(const struct shell* shell, size_t argc, char** argv) {

    ARG_UNUSED(argc);

    struct hci_packet hpkt_cli, hpkt;

    hpkt_cli.preamble = 0xAA;
    hpkt_cli.tt = REQUEST;
    hpkt_cli.payload.sid = atoi(argv[2]);
    hpkt_cli.payload.reg_addr = atoi(argv[3]);
    tx_done = 0;

    if (strcmp(argv[1], "r") == 0) {
        
        hpkt_cli.payload_length = 3;
        hpkt_cli.payload.sid = atoi(argv[2]);
        hpkt_cli.payload.i2c_addr_rw_bit = get_i2c_addr(hpkt_cli.payload.sid) | 0x01;
        printk("Sending i2c read to spi address : %d\n", get_i2c_addr(hpkt_cli.payload.sid));
        hpkt_cli.payload.reg_addr = atoi(argv[3]);
       
        os_process_hci_packet(hpkt_cli);

    }
    else {

        if(strcmp(argv[1], "w") == 0) {

            hpkt_cli.payload_length = 4;
            hpkt_cli.payload.i2c_addr_rw_bit = get_i2c_addr(hpkt_cli.payload.sid);
            hpkt_cli.payload.reg_data[0] = atoi(argv[4]);
            printk("Sending i2c write to spi address : %d\n", get_i2c_addr(hpkt_cli.payload.sid));
          
            os_process_hci_packet(hpkt_cli);

        }    
    }
   
    return 0;

}

/* Shell command handler to read acceleration values for the 3 axes from the sensor */
static int cmd_lsm6dsl_handler(const struct shell *shell, size_t argc, char** argv) {

    struct hci_packet cmd_pkt;
    ARG_UNUSED(argc);

    cmd_pkt.preamble = 0xAA;
    cmd_pkt.tt = REQUEST;
    cmd_pkt.payload_length = 4;
    cmd_pkt.payload.sid = LSM6DSL;
    cmd_pkt.payload.i2c_addr_rw_bit = get_i2c_addr(LSM6DSL) | 0x01;

    tx_done = 0;
    if(strcmp(argv[1],"r") == 0) {
       shell_print(shell, "read lsm6dsl");
        if(strcmp(argv[2], "x") == 0) {        
            cmd_pkt.payload.reg_addr = 0x28;
            shell_print(shell, "Sending spi packet");   
            
            os_process_hci_packet(cmd_pkt);
        }
        else if(strcmp(argv[2], "y") == 0) {
            cmd_pkt.payload.reg_addr = 0x2A;
            shell_print(shell, "Sending spi packet");
           
            os_process_hci_packet(cmd_pkt);
         
        }
        else if(strcmp(argv[2],"z") == 0) {
            cmd_pkt.payload.reg_addr = 0x2C;
            shell_print(shell, "Sending spi packet");
            
            os_process_hci_packet(cmd_pkt);
        
        }
        else if(strcmp(argv[2], "a") == 0) {
            struct accel a;
            
            os_get_accel_values(&a);
            shell_print(shell, "lsm6dsl accel values are : %f, %f, %f", a.x, a.y, a.z);
           
        }
    }
  
    return 0;
}

static int cmd_uranger_handler(const struct shell *shell, size_t argc, char** argv) {
    struct hci_packet cmd_pkt;
    ARG_UNUSED(argc);

    cmd_pkt.preamble = 0xAA;
    cmd_pkt.tt = REQUEST;
    cmd_pkt.payload_length = 4;
    cmd_pkt.payload.sid = HCSR04;
    cmd_pkt.payload.i2c_addr_rw_bit = 0xff;

    tx_done = 0;
    if(strcmp(argv[1],"r") == 0) {
        shell_print(shell, "read ultrasonic ranger distance value");

        cmd_pkt.payload.reg_addr = 0x28;
        shell_print(shell, "Sending spi packet");   
        
        while(1) {
            os_process_hci_packet(cmd_pkt);
            k_msleep(500);
        }
        
    }

}

SHELL_CMD_REGISTER(i2creg, NULL, "Get i2creg", cmd_hci_handler);
SHELL_CMD_ARG_REGISTER(lsm6dsl, NULL, "Accel sensor values", cmd_lsm6dsl_handler,3,0);
SHELL_CMD_REGISTER(uranger, NULL, "Ultrasonic ranger", cmd_uranger_handler);