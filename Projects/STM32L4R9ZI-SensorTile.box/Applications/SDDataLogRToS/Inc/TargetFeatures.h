/**
  ******************************************************************************
  * @file    SDDataLogRToS\Inc\TargetFeatures.h
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

#include "SensorTile.box.h"
#include "SensorTile.box_env_sensors.h"
#include "SensorTile.box_env_sensors_ex.h"
#include "SensorTile.box_motion_sensors.h"
#include "SensorTile.box_motion_sensors_ex.h"
#include "SensorTile.box_audio.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio_SensorTile.box.h"


/* Exported defines ------------------------------------------------------- */
/* Sensor data acquisition period [ms] */
#define DATA_PERIOD_MS     10
#define DATAQUEUE_SIZE     200

#define AUDIO_VOLUME_VALUE       32
#define AUDIO_CHANNELS           1
#define PCM_AUDIO_IN_SAMPLES     (AUDIO_SAMPLING_FREQUENCY / 1000)
#define AUDIO_BUFF_SIZE (PCM_AUDIO_IN_SAMPLES *1024)
#define AUDIO_BUFF_SIZE_MASK (AUDIO_BUFF_SIZE-1)


#define MSG_ENABLE_DISABLE 0x07
#define MSG_AUDIO_SAVE     0x01

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

/* Exported types ------------------------------------------------------- */

/**
  * @brief  Errors type Definitions
  */
typedef enum
{
  STBOX1_NO_ERROR = 0,
  STBOX1_INIT_ERROR,
  STBOX1_MEMS_ERROR,
  STBOX1_AUDIO_ERROR,
  STBOX1_FATFS,
  STBOX1_OS,
} ErrorType_t;

/* Exported variables ------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */
extern void ErrorHanlder(ErrorType_t ErrorType);
extern void InitTargetPlatform(void);
#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */


