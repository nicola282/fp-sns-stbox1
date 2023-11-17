/**
  ******************************************************************************
  * @file    SDDataLogRToS\Inc\datalog_application.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Header for SDDataLogRToS\Src\datalog_application.c module.
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
#ifndef __DATALOG_APPLICATION_H
#define __DATALOG_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_conf.h"

/* Exported Types     --------------------------------------------------------*/
typedef struct
{
  uint32_t ms_counter;
  float pressure;
  float humidity;
  float temperature;
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyro;
  BSP_MOTION_SENSOR_Axes_t mag;
} T_SensorsData;

/* Exported Variables --------------------------------------------------------*/
extern uint32_t SD_LogAudio_Enabled;

/* Exported Functions --------------------------------------------------------*/
extern void DATALOG_SD_Init(void);
extern uint8_t DATALOG_SD_Log_Enable(void);
extern uint8_t DATALOG_SD_writeBuf(char *s, uint32_t size);
extern void DATALOG_SD_Log_Disable(void);
extern void DATALOG_SD_DeInit(void);
extern int32_t getSensorsData(T_SensorsData *mptr);

extern uint8_t DATALOG_SD_LogAudio_Enable(void);
extern void closeFileAudio(void);
extern void AudioProcess_SD_Recording(uint16_t *pInBuff, uint32_t len);
extern void SaveAudioData(void);
extern void PrintSummary(void);

#ifdef __cplusplus
}
#endif

#endif /* __DATALOG_APPLICATION_H */

