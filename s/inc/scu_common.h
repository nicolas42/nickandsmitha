/**
******************************************************************************
* @file      apps/s/inc/scu_common.h
* @author    Smitha Ratnam - 44425782
* @date      29032021
* @brief     Common functions that the sources use.
*****************************************************************************
*                 EXTERNAL FUNCTIONS
*****************************************************************************
* 
*****************************************************************************
*/

#ifndef SCU_COMMON_H
#define SCU_COMMON_H

#include <drivers/sensor.h>
#include <device.h>
#include <zephyr.h>
#include <stdio.h>

void initialise_lis3mdl();
void initialise_lsm6dsl();
void initialise_hts221();
void initialise_vl53l0x();
void initialise_lps22hb();
void initialise_hcsr04();

void set_trigger();
uint32_t get_distance();
#endif