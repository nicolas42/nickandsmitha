/**
******************************************************************************
* @file      apps/s/src/scu_hci.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Provides the spi transport for the slave i2c read/write via 
*            the shell. 
*         REFERENCE: zephyr shell library api and documentation.
*                    B-L475e-IOT01A Platform datasheet
*****************************************************************************
*                 EXTERNAL FUNCTIONS
*****************************************************************************
* scu_init() - initialise  spi structures and device as master.
*****************************************************************************
*/

#include "scu_hci.h"
#include "scu_common.h"
#include <device.h>
#include <init.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <shell/shell_uart.h>

/* scheduling priority used by each thread */
#define PRIORITY 7

#define RX_THREAD_STACK_SIZE 1024
#define TX_THREAD_STACK_SIZE 1024
#define RX_THREAD_PRIORITY 6
#define TX_THREAD_PRIORITY 6
const uint8_t OUTX_L_XL = 0x28;
const uint8_t OUTX_H_XL = 0x29;
const uint8_t OUTY_L_XL = 0x2A;
const uint8_t OUTY_H_XL = 0x2B;
const uint8_t OUTZ_L_XL = 0x2C;
const uint8_t OUTZ_H_XL = 0x2D;

/* Global Variables */
struct spi_config spi_cfg;

static struct spi_buf rx;

static struct spi_buf tx;

const struct device *spi_slave;

/* Handles for the threads */
k_tid_t hci_id;

/* Thread data structures to be used by the macros for defining threads */
struct k_thread rx_thread_data;
struct k_thread tx_thread_data;

/* Message structure definition for the log message. */
/* @brief - received_msg is populated by the consumer thread while tx_data is populated by the external function called by the cli peripheral */
struct hci_packet hci_data;

/* Define the semaphores, queues, thread stacks. Being atomic, these need to be called outside the function */

/* Semaphore for accessing the queue between threads */
K_SEM_DEFINE(hci_spi_sem, 0, 1);

/* Define the common queue to send and receive log messages */
K_MSGQ_DEFINE(hci_msgq, sizeof(struct hci_packet), 10, 4);

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);

K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);

/* Define hci packets for request and response data */
struct hci_packet hci_req_pkt, hci_resp_pkt;

/* I2C Slave addresses */
const uint8_t HTS221_ADDR = 0x5E;
const uint8_t LIS3MDL_ADDR = 0x1D;
const uint8_t LPS22HB_ADDR = 0x5D;
const uint8_t LSM6DSL_ADDR = 0x6A;
const uint8_t VL53L0X_ADDR = 0x29;


const struct device* dev_i2c;
struct spi_cs_control *spi_ctrl;
static bool rx_done = 0;


/* Device sensor register addresses as per sid */
uint16_t get_dev_addr(uint8_t sid_val) {

    switch(sid_val) {

        case  LSM6DSL:
            return (uint16_t)LSM6DSL_ADDR;     

        case LIS3MDL:
            return (uint16_t)LIS3MDL_ADDR;
            
        case LPS22HB:
            return (uint16_t)LPS22HB_ADDR;
            
            
        case VL53L0X:
            return (uint16_t)VL53L0X_ADDR;
            
        case HTS221:
            return (uint16_t)HTS221_ADDR; 
    
    }
}

/* String function for easy printing */
const char* getstring(uint8_t sidval) {
    switch(sidval) {
        case LSM6DSL:
            return "LSM6DSL";
        default:
            return "";
    }
}


void spi_send() {
    uint8_t rxbuf[1] = {0};
    uint8_t buff[16];

    struct spi_buf_set rx_send = {
        
        .buffers = &(const struct spi_buf){

            .buf = rxbuf,
            .len = sizeof(rxbuf),

        },
        .count = 1,
    };

    /* fill up the response packet to be sent to master */
    buff[0] = hci_resp_pkt.preamble;
    buff[1] = hci_resp_pkt.payload.reg_data[0];
    buff[2] = hci_resp_pkt.payload.reg_data[1];
    buff[3] = hci_resp_pkt.payload.sid;
    buff[4] = hci_resp_pkt.payload.i2c_addr_rw_bit;
    buff[5] = hci_resp_pkt.payload.reg_addr;
    buff[6] = hci_resp_pkt.payload.reg_data[0];
    buff[7] = hci_resp_pkt.payload.reg_data[1];

    struct spi_buf_set tx_send = {

        .buffers = &(const struct spi_buf){

            .buf = buff,
            .len = sizeof(buff),

        },
        .count = 1,

    };

    /* send response to master */
    spi_transceive(spi_slave, &spi_cfg, &tx_send, NULL);

    k_msleep(100);

    printk("Slave sending: %d,%d,%d,%d,%d,%d,%d,%d\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);
}


/* Function to read i2c registers and send response data over spi as per hci protocol */
void read_and_send_regs() {
       
    uint8_t buf[10];
    uint16_t addr = 0;
    int16_t full_reg_val = 0;
    uint16_t higher_data_byte = 0;
    
    buf[0] = 0xAA;
    buf[1] = RESPONSE; 
    buf[2] = 4; 
    addr = get_dev_addr(hci_req_pkt.payload.sid);
   
    buf[3] = hci_req_pkt.payload.sid;
    buf[4] = addr;
    buf[5] = hci_req_pkt.payload.reg_addr;

      
    if( i2c_reg_read_byte(dev_i2c, addr, hci_req_pkt.payload.reg_addr, &buf[6]) != 0) {

        printk("i2c read error");
        return;

    }
    
    /* If the addresses are those of the Acceleration registers, then get the higher byte as well. This is for the "lsm6dsl" command and to avoid the master having to do a second read command. */
    if((hci_req_pkt.payload.reg_addr == OUTX_L_XL) || (hci_req_pkt.payload.reg_addr == OUTY_L_XL) || (hci_req_pkt.payload.reg_addr == OUTZ_L_XL)) {

        uint8_t next_addr = hci_req_pkt.payload.reg_addr  + 1;

        k_msleep(5);

        if( i2c_reg_read_byte(dev_i2c, addr, next_addr, &higher_data_byte) != 0) {

            printk("i2c read error");
            return;

        }

        buf[7] = higher_data_byte;
        higher_data_byte = higher_data_byte << 8;
        
        full_reg_val = (int16_t)(higher_data_byte |  buf[6]); 
 
    }
    
    printk("%s: reg_addr = %x, reg_data = %d, full reg value = %d, higher byte = %d\n", 
        getstring(hci_req_pkt.payload.sid), buf[5], buf[6], full_reg_val, higher_data_byte);

    hci_resp_pkt.preamble = buf[0];
    hci_resp_pkt.tt = RESPONSE;
    hci_resp_pkt.payload_length = buf[2];
    hci_resp_pkt.payload.sid = buf[3];
    hci_resp_pkt.payload.i2c_addr_rw_bit = addr;
    hci_resp_pkt.payload.reg_addr = buf[5];
    hci_resp_pkt.payload.reg_data[0] = buf[6];
    hci_resp_pkt.payload.reg_data[1] = buf[7];

    rx_done = 1;
    
}

/* Write to I2C register after receiving hci packet over spi */
void write_to_regs() {

    uint16_t addr = 0;
    addr = get_dev_addr(hci_req_pkt.payload.sid);

    if( i2c_reg_write_byte(dev_i2c, addr, hci_req_pkt.payload.reg_addr, hci_req_pkt.payload.reg_data[0]) != 0) {

        printk("i2c write error\n");
        return;

    } 

    hci_resp_pkt.preamble = 0xaa;
    hci_resp_pkt.tt = RESPONSE;
    hci_resp_pkt.payload_length = 4;
    hci_resp_pkt.payload.sid = hci_req_pkt.payload.sid;
    hci_resp_pkt.payload.i2c_addr_rw_bit = addr;
    hci_resp_pkt.payload.reg_addr = hci_req_pkt.payload.reg_addr;
    hci_resp_pkt.payload.reg_data[0] = hci_req_pkt.payload.reg_data[0];
   
    printk("write reg called to addr : %d\n", hci_req_pkt.payload.reg_addr);
    printk("written reg val = %d\n", hci_req_pkt.payload.reg_data[0]);

}
  
/* Process hci request */
void process_req() {
    
    if (hci_req_pkt.payload.sid == HCSR04) {
        uint32_t distance = 0;
   
        set_trigger();
        
        k_msleep(0.5);
       
        distance = get_distance();
             
        printk("Distance from sensor = %d\n", distance);
        hci_resp_pkt.preamble = 0xaa;
        hci_resp_pkt.tt = RESPONSE;
        hci_resp_pkt.payload_length = 4;
        hci_resp_pkt.payload.sid = 6;
        hci_resp_pkt.payload.i2c_addr_rw_bit = 0xff;
        hci_resp_pkt.payload.reg_addr = 0xFF;
        hci_resp_pkt.payload.reg_data[0] = distance & 0xFF;
        hci_resp_pkt.payload.reg_data[1] = (distance >> 8) & 0xFF;      

    } 
    else if ((hci_req_pkt.payload.i2c_addr_rw_bit & 0x01) == 0x01) {

        read_and_send_regs();

    }
    else {

        write_to_regs();
    }
}

/* process hci packet */
void process_pkt() {

    switch (hci_req_pkt.tt)
    {

    case REQUEST:
        process_req();
        break;

    /* To cover all the enums */
    case RESPONSE:
        break;

    default:
        break;
    }
}



void spi_receive(struct hci_packet* hpkt, uint8_t* data) {
        
    /* Fill up request structure from data available from master for processing */
    hci_req_pkt.preamble = data[0];
    hci_req_pkt.tt = data[1];
    hci_req_pkt.payload_length = data[2];
    hci_req_pkt.payload.sid = data[3];
    hci_req_pkt.payload.i2c_addr_rw_bit = data[4];
    hci_req_pkt.payload.reg_addr = data[5];
    hci_req_pkt.payload.reg_data[0] = data[6];

    process_pkt();

}


void hci_thd() {
    
    while(1) {

/* Initialise receive and transmit buffers for spi calls. */
        struct hci_packet hpkt;
        uint8_t txbuf[1] = {0};
        uint8_t rxbuf[1] = {0};
        uint8_t data[7];
        uint8_t buff[16];

        for(int i = 0; i < 7; i++) {

            data[i] = 0;

        }

        for(int j = 0; j < 11; j++) {

            buff[j] = 0;

        }

        struct spi_buf_set tx_rec = {

		    .buffers = &(const struct spi_buf) {

			    .buf = txbuf,
			    .len = sizeof(txbuf),

		    },
		    .count = 1,

	    };
   
        struct spi_buf_set rx_rec = {

		    .buffers = &(const struct spi_buf) {

			    .buf = data,
			    .len = sizeof(data),

		    },
		    .count = 1,

	    };

        /* Receive spi data from master */
        if(spi_transceive(spi_slave, &spi_cfg, NULL, &rx_rec) > 0) {

            /* process received data */
            spi_receive(&hpkt, data);
           
        }
    
        k_msleep(10);

       
        struct spi_buf_set rx_send = {
 
    		.buffers = &(const struct spi_buf) {

	    		.buf = rxbuf,
		    	.len = sizeof(rxbuf),

		    },
		    .count = 1,
	    }; 

        /* fill up the response packet to be sent to master */
        buff[0] = hci_resp_pkt.preamble;
        buff[1] = hci_resp_pkt.payload.reg_data[0];
        buff[2] = hci_resp_pkt.payload.reg_data[1];
        buff[3] = hci_resp_pkt.payload.sid;
        buff[4] = hci_resp_pkt.payload.i2c_addr_rw_bit;
        buff[5] = hci_resp_pkt.payload.reg_addr;
        buff[6] = hci_resp_pkt.payload.reg_data[0];
        buff[7] = hci_resp_pkt.payload.reg_data[1];
      
        struct spi_buf_set tx_send = {
 
    		.buffers = &(const struct spi_buf) {

	    		.buf = buff,
		    	.len = sizeof(buff),

		    },
		    .count = 1,

	    }; 
      
      /* send response to master */
        spi_transceive(spi_slave, &spi_cfg, &tx_send, NULL);
            
        k_msleep(100);
        
       // if (hci_resp_pkt.payload.sid != HCSR04) {
        //    printk("Slave sending: %d,%d,%d,%d,%d,%d,%d,%d\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);
       // }
        // else {
        //     printk("sensor distance : %d, %d\n", buff[1], buff[2]);
        // }
     
    }    
}

/* initialise the i2c and spi devices */
void scu_init() {

    initialise_lsm6dsl();
    initialise_lps22hb();
    initialise_hts221();
    initialise_lis3mdl();
    initialise_hcsr04();

    
    dev_i2c = device_get_binding(DT_LABEL(DT_NODELABEL(i2c2)));

    if(dev_i2c == NULL) {
        printk("i2c device not available");
        return;
    }
    
    
    spi_slave = device_get_binding(DT_LABEL(DT_NODELABEL(spi1)));
    if(spi_slave == NULL) {

        printk("Device not available");
        return;

    }   
    
    /* initialise spi config structure */
	const struct spi_cs_control cs_ctrl = {

		.gpio_dev = device_get_binding("GPIO_0"),
		.gpio_dt_flags = GPIO_ACTIVE_LOW | GPIO_OPEN_DRAIN,
		.delay = 0

	};

	spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_SLAVE;
	spi_cfg.frequency = 1000000U;
    spi_cfg.cs = &cs_ctrl;

    /* create and start hci thread and get its handle */
    hci_id = k_thread_create(&rx_thread_data, rx_thread_stack,
				K_THREAD_STACK_SIZEOF(rx_thread_stack),
				hci_thd, NULL, NULL, NULL,
				RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (!hci_id) {

		printk("ERROR spawning hci thread\n");
        return;

	}

// testing only. Remove for testing with spi
    hci_req_pkt.payload.sid = 6;
    process_req();
       
}