/**
  ******************************************************************************
  * @file    BLE_Manager_Conf.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   BLE Manager configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Manager_Conf.h.
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
#ifndef __BLE_MANAGER_CONF_H__
#define __BLE_MANAGER_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define ------------------------------------------------------------*/

/* Uncomment the following define for BlueST-SDK V2 */
#define BLE_MANAGER_SDKV2

#define BLUE_CORE BLUENRG_1_2

/* Define the Max dimesion of the Bluetooth characteristics for Debug Console services  */
#define DEFAULT_MAX_STDOUT_CHAR_LEN     155
#define DEFAULT_MAX_STDERR_CHAR_LEN     155

#define BLE_MANAGER_MAX_ALLOCABLE_CHARS 5U

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION

/* Define the Delay function to use inside the BLE Manager (HAL_Delay/osDelay) */
#define BLE_MANAGER_DELAY HAL_Delay

/****************** Memory management functions **************************/
#define BLE_MALLOC_FUNCTION      malloc
#define BLE_FREE_FUNCTION        free
#define BLE_MEM_CPY              memcpy

/*---------- Print messages from BLE Manager files at middleware level -----------*/
/* Uncomment/Comment the following define for  disabling/enabling print messages from BLE Manager files */
#define BLE_MANAGER_DEBUG

#ifdef BLE_MANAGER_DEBUG
/* Define the Verbosity level (1/2/3) */
#define BLE_DEBUG_LEVEL 1

#include "TargetFeatures.h"
#define BLE_MANAGER_PRINTF  STBOX1_PRINTF

#else /* BLE_MANAGER_DEBUG */
#define BLE_MANAGER_PRINTF(...)
#endif /* BLE_MANAGER_DEBUG */

#ifdef __cplusplus
}
#endif

#endif /* __BLE_MANAGER_CONF_H__*/

