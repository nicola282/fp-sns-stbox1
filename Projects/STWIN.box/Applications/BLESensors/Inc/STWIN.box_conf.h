/**
  ******************************************************************************
  * @file    BLESensors\Inc\STWIN.box_conf.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-Oct-2023
  * @brief   STWIN.box configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STWIN_BOX_CONF_H__
#define STWIN_BOX_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "STWIN.box_bus.h"
#include "STWIN.box_errno.h"


/* BSP COM Port */
#define USE_BSP_COM_FEATURE   1
#define USE_COM_LOG           1
#define BSP_COM_BAUDRATE      115200

/* SD card interrupt priority */
#define BSP_SD_IT_PRIORITY            14U  /* Default is lowest priority level */
#define BSP_SD_RX_IT_PRIORITY         14U  /* Default is lowest priority level */
#define BSP_SD_TX_IT_PRIORITY         15U  /* Default is lowest priority level */


#define BUTTON_USER_IT_PRIORITY   14U
#define BUTTON_PWR_IT_PRIORITY    14U


/* Define 1 to use already implemented callback; 0 to implement callback
   into an application file */

#define USE_BC_TIM_IRQ_CALLBACK         0U
#define USE_BC_GPIO_IRQ_HANDLER         1U
#define USE_BC_GPIO_IRQ_CALLBACK        1U

#define USE_MOTION_SENSOR_IIS2DLPC_0    0U
#define USE_MOTION_SENSOR_IIS2MDC_0     1U
#define USE_MOTION_SENSOR_IIS3DWB_0     0U
#define USE_MOTION_SENSOR_ISM330DHCX_0  1U
#define USE_MOTION_SENSOR_IIS2ICLX_0    0U

#define USE_ENV_SENSOR_ILPS22QS_0       1U
#define USE_ENV_SENSOR_STTS22H_0        1U

#define BSP_NFCTAG_INSTANCE             0U

  
#ifdef __cplusplus
}
#endif

#endif /* STWIN_BOX_CONF_H__*/
