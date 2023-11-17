/**
  ******************************************************************************
  * @file    BLE_Implementation.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   BLE Implementation header template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.h.
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
#ifndef _BLE_IMPLEMENTATION_H_
#define _BLE_IMPLEMENTATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "BLE_Battery.h"
#include "BLE_SensorFusion.h"
/* necessary for using floor on BLE_Manager.h */
#include <math.h>

/* Exported Defines --------------------------------------------------------*/

/* Package Version firmware */
#define BLE_VERSION_FW_MAJOR    STBOX1_VERSION_MAJOR
#define BLE_VERSION_FW_MINOR    STBOX1_VERSION_MINOR
#define BLE_VERSION_FW_PATCH    STBOX1_VERSION_PATCH

/* Firmware Package Name */
#define BLE_FW_PACKAGENAME      "BFF_150"

/* Custom Fw Id */
#define STBOX1_BLUEST_SDK_FW_ID 0x0F


/* Exported Variables ------------------------------------------------------- */
extern uint32_t ConnectionBleStatus;
extern volatile uint8_t  connected;
extern volatile uint8_t  paired;
extern volatile uint32_t RebootBoard;
extern volatile uint32_t SwapBanks;
extern volatile int32_t  PowerButtonPressed;
extern volatile uint32_t NeedToClearSecureDB;

/* Exported functions ------------------------------------------------------- */
extern void BLE_InitCustomService(void);
extern void BLE_SetCustomAdvertiseData(uint8_t *manuf_data);
extern void BluetoothInit(void);
/* Exported macro ------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _BLE_IMPLEMENTATION_H_ */

