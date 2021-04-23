/**
**************************************************************************************
* @file      apps/s/src/scu_lps22hb.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Initialises the Pressure/Temperature sensor device.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* initialise_lsm6dsl() - initialises the lps22hb sensor device.
***************************************************************************************
*/
#include "scu_common.h"

static const struct device *dev_lps22hb;
void initialise_lps22hb() {

    dev_lps22hb = device_get_binding(DT_LABEL(DT_INST(0, st_lps22hb_press)));

    if (dev_lps22hb == NULL) {

        printf("Could not get device lps22hb\n");
        return;

    }

}
