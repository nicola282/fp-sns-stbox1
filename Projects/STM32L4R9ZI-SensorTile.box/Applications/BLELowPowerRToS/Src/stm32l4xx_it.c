/**
  ******************************************************************************
  * @file    BLELowPowerRToS\Src\stm32l4xx_it.c 
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"
#include "stm32l4xx_it.h"
#include "cmsis_os.h"
#include "hci_tl_interface.h"

/* Imported variables ---------------------------------------------------------*/

extern RTC_HandleTypeDef RtcHandle;
#ifdef STBOX1_ENABLE_PRINTF
  extern PCD_HandleTypeDef hpcd;
#endif /* STBOX1_ENABLE_PRINTF */

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
#ifdef NO_FREE_RTOS
/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}
#endif
/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

#ifdef NO_FREE_RTOS
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  osSystickHandler();
  HAL_IncTick();
}

/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xxxx.s).                                             */
/******************************************************************************/

#ifdef STBOX1_ENABLE_PRINTF
/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}
#endif /* STBOX1_ENABLE_PRINTF */

/**
  * @brief This function handles DFSDM Left DMAinterrupt request.
  * @param None
  * @retval None
  */
void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AMic_OnBoard_DfsdmFilter.hdmaReg);
}

/**
* @brief  This function handles External line 1 interrupt request.
* @param  None
* @retval None
*/
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
}

/**
  * @brief  EXTI2_IRQHandler This function handles External line
  *         interrupt request for Power Button
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(POWER_BUTTON_PIN);
}

/**
  * @brief  EXTI3_IRQHandler This function handles External line
  *         interrupt from Battery Charger
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
  /* HW events from LSM6DSO */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/**
  * @brief  EXTI4_IRQHandler This function handles External line
  *         interrupt request for BlueNRG.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(HCI_TL_SPI_EXTI_PIN);
}

void DFSDM1_FLT1_IRQHandler(void)
{
  HAL_DFSDM_IRQHandler(&AMic_OnBoard_DfsdmFilter);
}

void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(SensorTileADC.DMA_Handle);
}

/**
  * @brief  This function handles RTC Auto wake-up interrupt request.
  * @param  None
  * @retval None
  */
void RTC_WKUP_IRQHandler(void)
{
  HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandle);
}
