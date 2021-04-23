/**
**************************************************************************************
* @file      s/src/scu_hts221.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Initialises the Temperature/Humidity sensor device.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* initialise_hts221() - initialises the hts221 sensor device.
***************************************************************************************
*/
#include "scu_common.h"

static const struct device *dev_hts221;

void initialise_hts221() {

    dev_hts221 = device_get_binding(DT_LABEL(DT_INST(0, st_hts221)));

    if (dev_hts221 == NULL) {

        printf("Could not get device hts221\n");
        return;
		
    }
}



