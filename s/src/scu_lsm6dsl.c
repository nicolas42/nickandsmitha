/**
**************************************************************************************
* @file      apps/s/src/scu_lsm6dsl.c
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Initialises the Accelerometer/Gyrometer device.
*         REFERENCE: zephyr RTOS api and documentation.
***************************************************************************************
*                 EXTERNAL FUNCTIONS
***************************************************************************************
* initialise_lsm6dsl() - initialises the lsm6dsl sensor device.
***************************************************************************************
*/

#include "scu_common.h"

static struct sensor_value odr_attr;

static const struct device *dev_lsm6dsl;
 
void initialise_lsm6dsl() {
  

    dev_lsm6dsl = device_get_binding(DT_LABEL(DT_INST(0, st_lsm6dsl)));

    if (dev_lsm6dsl == NULL) {

        printk("Could not get device lsm6dsl\n");
        return;

    }

    /* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;
	if (sensor_attr_set(dev_lsm6dsl, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {

		printk("Cannot set sampling frequency for accelerometer.\n");
		return;

	}

	if (sensor_attr_set(dev_lsm6dsl, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {

		printk("Cannot set sampling frequency for gyro.\n");
		return;

	}

}
