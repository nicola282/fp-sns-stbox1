/**
  ******************************************************************************
  * @file    BLEDualProgram\Inc\bluenrg_conf.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief  BlueNRG-1 Configuration file
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
#ifndef __BLUENRG_CONF_H
#define __BLUENRG_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32l4xx_hal.h"
#include <string.h>

/*---------- Print messages from BLE2 files at user level -----------*/
#define BLE2_DEBUG      1
/*---------- Print the data travelling over the SPI in the .csv format compatible with the ST BlueNRG GUI -----------*/
#define PRINT_CSV_FORMAT      0
/*---------- Print messages from BLE2 files at middleware level -----------*/
#define BLUENRG2_DEBUG      1
 
/*---------- Number of Bytes reserved for HCI Read Packet -----------*/
#define HCI_READ_PACKET_SIZE      200
/*---------- Number of Bytes reserved for HCI Max Payload -----------*/
#define HCI_MAX_PAYLOAD_SIZE      255

#define HCI_DEFAULT_TIMEOUT_MS        1000

#define BLUENRG_memcpy                memcpy
#define BLUENRG_memset                memset
#define BLUENRG_memcmp                memcmp

#if (BLE2_DEBUG == 1)
  #include "TargetFeatures.h"
  #define PRINT_DBG(...)              STBOX1_PRINTF(__VA_ARGS__)
#else
  #define PRINT_DBG(...)
#endif

#if PRINT_CSV_FORMAT
  #include "TargetFeatures.h"
  #define PRINT_CSV(...)              STBOX1_PRINTF(__VA_ARGS__)
  void print_csv_time(void);
#else
  #define PRINT_CSV(...)
#endif

#if BLUENRG2_DEBUG
  #include "TargetFeatures.h"
  #define BLUENRG_PRINTF(...)         STBOX1_PRINTF(__VA_ARGS__)
#else
  #define BLUENRG_PRINTF(...)
#endif

#ifdef __cplusplus
}
#endif
#endif /*__BLUENRG_CONF_H */

