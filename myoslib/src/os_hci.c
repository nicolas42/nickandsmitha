/**
**************************************************************************************
* @file      myoslib/src/os_hci.c
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
*                         messages for further processing messages.
***************************************************************************************
*/
#include <sys/printk.h>
#include <shell/shell.h>
#include <drivers/spi.h>
#include <shell/shell_uart.h>
#include <init.h>
#include "os_hci.h"
#include "hci_packet.h"
#include <device.h>
#include <kernel.h>

/* scheduling priority used by each thread */

#define RX_THREAD_PRIORITY 6
#define RX_THREAD_STACK_SIZE 1024
#define SENSOR_GVAL_DOUBLE  (9806650LL / 1000000.0)

/* Global Variables */
static struct spi_config spi_cfg;
static const struct device *spi_dev = NULL;
    
/* Handles for the threads */
k_tid_t hci_consumer_id;
struct k_thread rx_thread_data;

const struct shell *shell_ptr = NULL;
K_SEM_DEFINE(hci_qsem, 0, 1);
K_SEM_DEFINE(spi_sem, 0, 1);
K_MSGQ_DEFINE(msg_queue, sizeof(struct hci_packet), 10, 4);

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);

static struct hci_packet hci_rsp_pkt;
struct accel acc;

static const struct bt_data sd1[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

/* Advertising data */

static uint8_t sensor_data[ADV_LEN] = {
	0x01 /*SKD version */,
	0x83 /* STM32WB - P2P Server 1 */,
	0x20 /* GROUP A Feature  */,
	0x30 /* GROUP A Feature */,
	0x63 /* GROUP B Feature */,
	0x73 /* GROUP B Feature */,
	0x00, /* BLE MAC start -MSB */
	0x03,
	0x00,
	0x00,
	0x00,
	0x00 /* BLE MAC stop */
};

const struct bt_data ad1[] = {
//	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, sensor_data, ADV_LEN)
};

void os_process_hci_packet(struct hci_packet hci_pkt) {

    /* put message into queue */
    while(k_msgq_put(&msg_queue, &hci_pkt, K_NO_WAIT) != 0) {

        k_msgq_purge(&msg_queue);

     }
    
    k_sem_give(&hci_qsem);

}

void  os_get_accel_values(struct accel* a) {

    a->x = acc.x;
    a->y = acc.y;
    a->z = acc.z;

}

void process_resp() {

    int16_t full_reg_value = 0;
    double scaled_val;

    if(hci_rsp_pkt.preamble == 0xaa) {

        uint16_t higher_data_byte = hci_rsp_pkt.payload.reg_data[1];

        higher_data_byte = higher_data_byte << 8;

        full_reg_value = (int16_t)(higher_data_byte | hci_rsp_pkt.payload.reg_data[0]);

        /* Zephyr sample code - reference */
        scaled_val = (double)(full_reg_value) * (61LL / 1000.0) * (SENSOR_GVAL_DOUBLE / 1000);

        if(hci_rsp_pkt.payload.sid != HCSR04)  {  
            printk("%s, %s : reg data byte 0 = %d\n", getstring(hci_rsp_pkt.payload.sid), (((hci_rsp_pkt.payload.i2c_addr_rw_bit & 0x01) == 0) ? "r" : "w"), hci_rsp_pkt.payload.reg_data[0]);
        }
        else {
            printk("Distance : %d\n", hci_rsp_pkt.payload.reg_data[0]);
            sensor_data[8] = hci_rsp_pkt.payload.reg_data[0];
           
            bt_le_adv_update_data(ad1, ARRAY_SIZE(ad1), sd1, ARRAY_SIZE(sd1));


        }

        if (hci_rsp_pkt.payload.reg_addr == 0x28) {

            acc.x = scaled_val;
            printk("Scaled Accel X = %f\n", scaled_val);

        }
        else if (hci_rsp_pkt.payload.reg_addr == 0x2a) {

            acc.y = scaled_val;
            printk("Scaled Accel Y = %f\n", scaled_val);

        }
        else if (hci_rsp_pkt.payload.reg_addr == 0x2c) {

            acc.z = scaled_val;
            printk("Scaled Accel Z = %f\n", scaled_val);

        }

    }

    
}

/* Thread to handle spi calls */
void hci_consumer(void* arg1, void* arg2, void* arg3) {

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

   
    uint8_t number_of_elements = 0;
  
 
    while(1) {

        uint8_t data[7];
        uint8_t buff[16];
        struct hci_packet hci_data;
        uint8_t rxbuf[1] = {0};
        uint8_t txbuf[1] = {0};

        for(int i = 0; i < 7; i++) {
            data[i] = 0;
        }

        for(int j = 0; j < 11; j++) {
            buff[j] = 0;
        }

        if (k_sem_take(&hci_qsem, K_FOREVER) == 0) {

            k_msgq_get(&msg_queue, &hci_data, K_FOREVER);

        }

        /* Send: Send HCI request to slave via SPI. Fill Buffer to send */
       
        data[0] = hci_data.preamble;
        data[1] = hci_data.tt;
        data[2] = hci_data.payload_length;
        data[3] = (hci_data.payload).sid;
        data[4] = (hci_data.payload).i2c_addr_rw_bit;
        data[5] = (hci_data.payload).reg_addr;
        data[6] = (hci_data.payload).reg_data[0];

    /* initialise the receiver and transmit buffers for spi */
        struct spi_buf_set rx = {

            .buffers = &(const struct spi_buf){
                .buf = rxbuf,
                .len = sizeof(rxbuf),
            },
            .count = 1,

        };

        struct spi_buf_set tx = {

            .buffers = &(const struct spi_buf){
                .buf = data,
                .len = sizeof(data),
            },
            .count = 1,

        };
        

        spi_transceive(spi_dev, &spi_cfg, &tx, NULL);

        k_sem_give(&spi_sem);
        k_msleep(100);
      
        /* Receive: Get HCI response over SPI from slave */
        struct spi_buf_set tx_rec = {

            .buffers = &(const struct spi_buf){
                .buf = txbuf,
                .len = sizeof(txbuf),
            },
            .count = 1,

        };

        struct spi_buf_set rx_rec = {

            .buffers = &(const struct spi_buf){
                .buf = buff,
                .len = sizeof(buff),
            },
            .count = 1,

        };

    
       if(k_sem_take(&spi_sem, K_FOREVER) == 0) {
       
           spi_transceive(spi_dev, &spi_cfg, NULL, &rx_rec);

           k_msleep(100);

          // printk("Master received: %d,%d,%d,%d,%d,%d,%d, %d\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);

           /* Fill data from response to process */
           hci_rsp_pkt.preamble = buff[0];
           hci_rsp_pkt.tt = RESPONSE;
           hci_rsp_pkt.payload_length = 4;
           hci_rsp_pkt.payload.sid = buff[3];
           hci_rsp_pkt.payload.i2c_addr_rw_bit = buff[4];
           hci_rsp_pkt.payload.reg_addr = buff[5];
           hci_rsp_pkt.payload.reg_data[0] = buff[1];
           hci_rsp_pkt.payload.reg_data[1] = buff[2];

           process_resp();

       }
    }
}


void os_hci_init() {
    spi_dev = device_get_binding(DT_LABEL(DT_NODELABEL(spi1)));
    if (spi_dev == NULL)
    {
        printk("Device not found\n");

        return;
    }

    spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;
    spi_cfg.frequency = 1000000U;

    /* create and start consumer thread and get its handle */
    hci_consumer_id = k_thread_create(&rx_thread_data, rx_thread_stack,
                                      K_THREAD_STACK_SIZEOF(rx_thread_stack),
                                      hci_consumer, NULL, NULL, NULL,
                                      RX_THREAD_PRIORITY, 0, K_NO_WAIT);

    if (!hci_consumer_id)
    {

        printk("ERROR spawning RX thread\n");
        return;
    }
}
