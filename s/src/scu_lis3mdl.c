/**
**************************************************************************************
* @file      apps/s/src/scu_lis3mdl.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Initialises the Magnetometer sensor device.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* initialise_lis3mdl() - initialises the lis3mdl sensor device.
***************************************************************************************
*/
#include "scu_common.h"

static const struct device *dev_lis3mdl;
void initialise_lis3mdl() {

    dev_lis3mdl = device_get_binding(DT_LABEL(DT_INST(0, st_lis3mdl_magn)));

    if (dev_lis3mdl == NULL) {

        printf("Could not get device lis3mdl\n");
        return;

    }

}


