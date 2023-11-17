/**
  ******************************************************************************
  * @file    SDDataLogRToS\Inc\main.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   This file contains all the functions prototypes for the SDDataLogRToS\Src\main.c
  *          file.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal_def.h"
#include "SensorTile.box.h"
#include "cmsis_os.h"
#include "STBOX1_config.h"
  
  
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Exported variables---------------------------------------------------------*/
extern osMessageQId dataQueue_id;
extern uint32_t MaxWriteTimeAudio;
extern int32_t PoolAllocated;
extern int32_t PoolReleased;
extern int32_t PoolMaxSize;
extern int32_t MessagePushed;
extern int32_t MessageRemoved;
extern int32_t MessageMaxSize;
  
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

