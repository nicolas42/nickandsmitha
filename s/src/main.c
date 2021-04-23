/**
******************************************************************************
* @file      apps/s/src/main.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     The main thread. 
*****************************************************************************
*/
#include "scu_hci.h"

void main(void)
{
   /* initialises the SCU as SPI slave,sensors and I2C devices.*/
   scu_init();

}

