/**
  ******************************************************************************
  * @file    DataLogExtended\Inc\cube_hal.h
  * @brief   Header for cube_hal_l4.c
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
#ifndef CUBE_HAL_H
#define CUBE_HAL_H

/* Includes ------------------------------------------------------------------*/
#ifdef STM32_SENSORTILEBOX
#include "stm32l4xx_hal.h"
#include "SensorTile.box.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal_def.h"
#endif /* STM32_SENSORTILEBOX */

/* Exported constants --------------------------------------------------------*/

#ifdef STM32_SENSORTILEBOX
/* RTC Clock Source */
#define RTC_ASYNCH_PREDIV_LSI  0x7F
#define RTC_SYNCH_PREDIV_LSI   0xF9

#define RTC_ASYNCH_PREDIV_LSE  0x7F
#define RTC_SYNCH_PREDIV_LSE   0xFF
#endif /* STM32_SENSORTILEBOX */

void SystemClock_Config(void);
uint32_t Get_DMA_Flag_Status(DMA_HandleTypeDef *handle_dma);
uint32_t Get_DMA_Counter(DMA_HandleTypeDef *handle_dma);
void Config_DMA_Handler(DMA_HandleTypeDef *handle_dma);

#endif /* CUBE_HAL_H */

