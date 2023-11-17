/**
  ******************************************************************************
  * @file    BLEMLC\Inc\TargetFeatures.h
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
#include "SensorTile.box_bc.h"
#include "SensorTile.box_motion_sensors.h"
#include "SensorTile.box_motion_sensors_ex.h"

#ifdef STBOX1_ENABLE_PRINTF
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#endif /* STBOX1_ENABLE_PRINTF */

/* Imported variables ------------------------------------------------------ */
extern void *MotionCompObj[];

/* Exported defines ------------------------------------------------------- */
/* Every second */
#define STBOX1_UPDATE_LED 10000
/* Every second */
#define STBOX1_UPDATE_BATTERY 10000

/* Update Inertial Features @20HZ */
#define STBOX1_UPDATE_INV 500

/* Every 5 mSec */
#define STBOX1_UPDATE_VCOM 50

/* Context of LSM6DSOX */
#define LSM6DSOX_CONTEX (&(((LSM6DSOX_Object_t *)MotionCompObj[LSM6DSOX_0])->Ctx))

/* Exported macros ------------------------------------------------------- */
#define MCR_HEART_BIT()  \
  {                        \
    BSP_LED_On(LED_BLUE);  \
    BSP_LED_On(LED_GREEN); \
    HAL_Delay(200);        \
    BSP_LED_Off(LED_BLUE); \
    BSP_LED_Off(LED_GREEN);\
    HAL_Delay(400);        \
    BSP_LED_On(LED_BLUE);  \
    BSP_LED_On(LED_GREEN); \
    HAL_Delay(200);        \
    BSP_LED_Off(LED_BLUE); \
    BSP_LED_Off(LED_GREEN);\
    HAL_Delay(1000);       \
  }

#ifdef STBOX1_RESTART_DFU
/* Board DFU Magic Number Position */
extern uint32_t DFU_Var;

/* Board  DFU Magic Number */
#define DFU_MAGIC_NUM 0xABEDBABE
#endif /* STBOX1_RESTART_DFU */

/* Exported variables ------------------------------------------------------- */
extern TIM_HandleTypeDef    TimCCHandle;
#ifdef STBOX1_ENABLE_PRINTF
extern uint8_t VComBufferToWrite[];
extern int32_t VComBytesToWrite;
#endif /* STBOX1_ENABLE_PRINTF */

extern ucf_line_t *MLCCustomUCFFile;
extern uint32_t MLCCustomUCFFileLength;
extern ucf_line_t *FSMCustomUCFFile;
extern uint32_t FSMCustomUCFFileLength;
extern char *MLCCustomLabels;
extern uint32_t MLCCustomLabelsLength;
extern char *FSMCustomLabels;
extern uint32_t FSMCustomLabelsLength;

/* Exported functions ------------------------------------------------------- */
extern void InitTargetPlatform(void);
extern void LedInitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);
extern void LedToggleTargetPlatform(void);
extern void Init_MEMS(void);
extern void Init_MEMS_MLC(uint32_t UseCustomIfAvailable);
extern void Init_MEMS_FSM(uint32_t UseCustomIfAvailable);
extern void DeInit_MEMS(void);

#ifdef STBOX1_ENABLE_PRINTF
extern uint32_t VCOM_read(char *buffer, uint32_t len_max);
#endif /* define STBOX1_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */
