/**
  ******************************************************************************
  * @file    BLELowPowerRToS\Inc\PowerControl.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Header for BLELowPowerRToS\Src\PowerControl.c module
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
#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
int32_t initPowerController(void);

/* Exported defines and variables  ------------------------------------------------------- */
typedef enum
{
  RUN                   = 0,
  IDLE_WFI              = 1,
  IDLE_WFI_TICK_SUPRESS = 2,
  IDLE_SLEEP_STOP       = 3,
  STAND_BY              = 4,
  SHUTDOWNN             = 5
} powerState_t;

/*exported for IT handler */
extern RTC_HandleTypeDef RtcHandle;

int32_t  SetMinPowerMode(powerState_t);
powerState_t GetMinPowerMode(void);
int32_t PowerCtrlLock(void);
int32_t PowerCtrlUnLock(void);
int32_t PowerCtrlGetState(void);

#endif /* __POWERCONTROL */

