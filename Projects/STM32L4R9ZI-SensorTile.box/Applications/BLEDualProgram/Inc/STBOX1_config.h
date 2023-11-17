/**
  ******************************************************************************
  * @file    BLEDualProgram\Inc\STBOX1_config.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   FP-SNS-STBOX1 configuration
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
#ifndef __STBOX1_CONFIG_H
#define __STBOX1_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define ------------------------------------------------------------*/

/* For enabling the printf using USBD.
 * After the Initialization of USBD the board will wait 5 seconds for having
 * the possibility to take a look on the Initialization phase */
#define STBOX1_ENABLE_PRINTF

/**************************************
  *  Lab/Experimental section defines  *
  ***************************************/
/* For enabling the capability to restart in DFU mode */
#define STBOX1_RESTART_DFU

/* For enabling Secure connection */
#define STBOX1_BLE_SECURE_CONNECTION

/**************************************
  * Don't Change the following defines *
  ***************************************/

/* Package Version only numbers 0->9 */
#define STBOX1_VERSION_MAJOR '1'
#define STBOX1_VERSION_MINOR '6'
#define STBOX1_VERSION_PATCH '0'

/* Package Name */
#define STBOX1_PACKAGENAME "BLEDualProgram"

#ifdef STBOX1_ENABLE_PRINTF
#define STBOX1_PRINTF(...) {\
                             VComBytesToWrite = sprintf((char *)VComBufferToWrite, __VA_ARGS__);\
                             CDC_Fill_Buffer(VComBufferToWrite, VComBytesToWrite);\
                             }
#else /* STBOX1_ENABLE_PRINTF */
#define STBOX1_PRINTF(...)
#endif /* STBOX1_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif

#endif /* __STBOX1_CONFIG_H */

