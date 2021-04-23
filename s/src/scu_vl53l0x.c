/**
**************************************************************************************
* @file      s/src/scu_vl53l0x.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Initialises the Pressure/Temperature sensor device.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* initialise_vl53l0x() - initialises the vl53l0x sensor device.
***************************************************************************************
*/
#include "scu_common.h"

static const struct device *dev_vl53l0x;

void initialise_vl53l0x() {

    dev_vl53l0x = device_get_binding(DT_LABEL(DT_INST(0, st_vl53l0x)));

    if (dev_vl53l0x == NULL) {

        printf("Could not get device vl53l0x\n");
        return;
		
    }

}
