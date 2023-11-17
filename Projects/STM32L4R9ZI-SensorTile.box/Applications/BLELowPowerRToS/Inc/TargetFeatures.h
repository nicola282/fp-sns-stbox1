/**
  ******************************************************************************
  * @file    BLELowPowerRToS\Inc\TargetFeatures.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Specification of the HW Features for each target platform
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
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"

#include "STBOX1_config.h"

#include "SensorTile.box.h"
#include "SensorTile.box_env_sensors.h"
#include "SensorTile.box_env_sensors_ex.h"
#include "SensorTile.box_motion_sensors.h"
#include "SensorTile.box_motion_sensors_ex.h"
#include "SensorTile.box_audio.h"
#include "SensorTile.box_bc.h"

#ifdef STBOX1_ENABLE_PRINTF
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#endif /* STBOX1_ENABLE_PRINTF */


/* Exported defines ------------------------------------------------------- */

/* @brief  Check Values for understanding if one MEMS Sensor is present or Not */
#define STBOX1_SNS_NOT_VALID 9999

/* Exported macros ------------------------------------------------------- */
#define MCR_HEART_BIT() \
  { \
    BSP_LED_On(LED_BLUE);\
    BSP_LED_On(LED_GREEN);\
    HAL_Delay(200);\
    BSP_LED_Off(LED_BLUE);\
    BSP_LED_Off(LED_GREEN);\
    HAL_Delay(400);\
    BSP_LED_On(LED_BLUE);\
    BSP_LED_On(LED_GREEN);\
    HAL_Delay(200);\
    BSP_LED_Off(LED_BLUE);\
    BSP_LED_Off(LED_GREEN);\
    HAL_Delay(1000);\
  }

#define LED_ON_TARGET_PLATFORM() BSP_LED_On(LED_BLUE)
#define LED_OFF_TARGET_PLATFORM() BSP_LED_Off(LED_BLUE)
#define LED_TOGGLE_TARGET_PLATFORM() BSP_LED_Toggle(LED_BLUE)

#ifdef STBOX1_RESTART_DFU
/* Board DFU Magic Number Position */
extern uint32_t DFU_Var;

/* Board  DFU Magic Number */
#define DFU_MAGIC_NUM 0xABEDBABE
#endif /* STBOX1_RESTART_DFU */

/* Exported types ------------------------------------------------------- */

/**
  * @brief  Target's Features data structure definition
  */
typedef struct
{
  uint32_t HandleTempSensor;
  uint32_t HandlePressSensor;
  uint32_t HandleHumSensor;

  uint32_t HandleAccSensor;
  uint32_t HandleGyroSensor;
  uint32_t HandleMagSensor;

  float DefaultAccODR;
  float DefaultGyroODR;
  float DefaultMagODR;

  uint32_t AudioVolume;
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

#ifdef STBOX1_ENABLE_PRINTF
extern uint8_t VComBufferToWrite[];
extern int32_t VComBytesToWrite;
#endif /* STBOX1_ENABLE_PRINTF */

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(void);

extern void InitMics(uint32_t AudioFreq);
extern void DeInitMics(void);

extern void LedInitTargetPlatform(void);

extern void EnableEnvSensors(void);
extern void DisableEnvSensors(void);
extern void EnableMotionSensors(void);
extern void DisableMotionSensors(void);

#ifdef STBOX1_ENABLE_PRINTF
extern uint32_t VCOM_read(char *buffer, uint32_t len_max);
#endif /* define STBOX1_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

