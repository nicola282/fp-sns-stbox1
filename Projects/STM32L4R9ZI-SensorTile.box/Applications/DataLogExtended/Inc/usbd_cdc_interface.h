/**
  ******************************************************************************
  * @file    DataLogExtended\Inc\usbd_cdc_interface.h
  * @brief   Header for usbd_cdc_interface.c
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
#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Definition for TIMx clock resources */
#define TIMx                             TIM3
#define TIMx_CLK_ENABLE                  __HAL_RCC_TIM3_CLK_ENABLE
#define TIMx_FORCE_RESET()               __HAL_RCC_USART3_FORCE_RESET()
#define TIMx_RELEASE_RESET()             __HAL_RCC_USART3_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                        TIM3_IRQn
#define TIMx_IRQHandler                  TIM3_IRQHandler

/* Periodically, the state of the buffer "UserTxBuffer" is checked.
   The period depends on CDC_POLLING_INTERVAL */
#define CDC_POLLING_INTERVAL             5 /* in ms. The max is 65 and the min is 1 */

extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t CDC_Fill_Buffer(uint8_t *Buf, uint32_t TotalLen, uint32_t *Written);
uint8_t CDC_Next_Packet_Rx(void);
void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* USBD_CDC_IF_H */

