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

#ifndef HAL_HCI_H
#define HAL_HCI_H

#include <stdint.h>
#include <stdlib.h>

extern void hal_hci_init();
#endif
