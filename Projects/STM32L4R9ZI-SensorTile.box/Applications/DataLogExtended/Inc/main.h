/**
  ******************************************************************************
  * @file    DataLogExtended\Inc\main.h
  * @brief   Header for main.c
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
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "SensorTile.box_bus.h"
#include "SensorTile.box_errno.h"
#include "cube_hal.h"
#include "SensorTile.box_env_sensors.h"
#include "SensorTile.box_env_sensors_ex.h"
#include "SensorTile.box_motion_sensors.h"
#include "SensorTile.box_motion_sensors_ex.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/

/* Enable sensor masks */
#define PRESSURE_SENSOR_ENABLED                 0x00000001U
#define TEMPERATURE_SENSOR_ENABLED              0x00000002U
#define HUMIDITY_SENSOR_ENABLED                 0x00000004U
#define UV_SENSOR_ENABLED                       0x00000008U /* for future use */
#define ACCELEROMETER_SENSOR_ENABLED            0x00000010U
#define GYROSCOPE_SENSOR_ENABLED                0x00000020U
#define MAGNETIC_SENSOR_ENABLED                 0x00000040U
#define INTERRUPTS_ENABLED                      0x00000100U
#define FSM_ENABLED                             0x00000200U
#define MLC_ENABLED                             0x00000400U
  
extern uint32_t SensorsEnabled;

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void Error_Handler(void);
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);
uint32_t DWT_GetTickUS(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

