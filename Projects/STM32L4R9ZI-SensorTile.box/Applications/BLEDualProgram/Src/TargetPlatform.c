/**
  ******************************************************************************
  * @file    BLEDualProgram\Src\TargetPlatform.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "OTA.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
TIM_HandleTypeDef    TimCCHandle;
#ifdef STBOX1_ENABLE_PRINTF
USBD_HandleTypeDef USBD_Device;
volatile uint8_t VCOM_RxData;
volatile uint8_t *VCOM_RxBuffer = NULL; /* Pointer to data buffer (received from USB). */
volatile uint32_t VCOM_RxLength = 0;    /* Data length (received from USB). */
uint8_t VComBufferToWrite[256];
int32_t VComBytesToWrite;
#endif /* STBOX1_ENABLE_PRINTF */

/* Current Active Bank */
int32_t CurrentActiveBank = 0;

/* Watch Dog Handle */
IWDG_HandleTypeDef   IwdgHandle;

/* Local defines -------------------------------------------------------------*/
#define DEFAULT_TIM_PERIOD 65535

/* Local function prototypes --------------------------------------------------*/
static void InitTimers(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
  /* Check if we are running from Bank1 or Bank2 */
  {
    FLASH_OBProgramInitTypeDef    OBInit;
    /* Allow Access to Flash control registers and user Flash */
    HAL_FLASH_Unlock();
    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();
    /* Get the Dual boot configuration status */
    HAL_FLASHEx_OBGetConfig(&OBInit);
    if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE)
    {
      CurrentActiveBank = 2;
    }
    else
    {
      CurrentActiveBank = 1;
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
  }

  /* Check if we comes from IDWG Reset*/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    /* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    {
      uint16_t FwId1;
      uint16_t FwId2;

      ReadFlashBanksFwId(&FwId1, &FwId2);
      if (FwId2 != OTA_OTA_FW_ID_NOT_VALID)
      {
        /* RollBack to previous code version only if there is a valid Fw presnt*/
        EnableDisableDualBoot();
      }
    }
  }

  /* Initializes the Timers */
  InitTimers();

  /* Initializes the IDWG with 10 seconds */
  InitWatchDog(10);

  /* Start the Timer for making the IDWG refresh */
  {
    uint32_t uhCapture;
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + STBOX1_UPDATE_IDWG));
  }

  /* Init the LEDs */
  LedInitTargetPlatform();

  /* Initialize User Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize the Power Button */
  BSP_PowerButton_Init();

  /* Initialize the Battery Charger */
  BSP_BC_Init();

  /* In order to be able to Read Battery Volt */
  BSP_BC_BatMS_Init();

  /* In order to Initialize the GPIO for having the battery Status */
  BSP_BC_ChrgPin_Init();

  if (CurrentActiveBank == 1)
  {
    MCR_HEART_BIT();
    MCR_HEART_BIT();
  }
  else if (CurrentActiveBank == 2)
  {
    MCR_HEART_BIT2();
    MCR_HEART_BIT2();
  }

#ifdef STBOX1_ENABLE_PRINTF
  BSP_LED_On(LED_BLUE);
  if (CurrentActiveBank == 1)
  {
    BSP_LED_On(LED_GREEN);
  }
  else
  {
    BSP_LED_On(LED_RED);
  }

  /* Enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /*** USB CDC Configuration ***/
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add Interface callbacks for CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  /* Wait 5 seconds for looking the Initialization phases */
  HAL_Delay(5000);

  BSP_LED_Off(LED_BLUE);
  if (CurrentActiveBank == 1)
  {
    BSP_LED_Off(LED_GREEN);
  }
  else
  {
    BSP_LED_Off(LED_RED);
  }
#endif /* STBOX1_ENABLE_PRINTF */

}

/**
  * @brief  Function for initializing timers:
  *  - 1 for sending Battery Information
  *  - 1 for Led Blinking
  * @param  None
  * @retval None
  */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 10000) - 1);

  /* Set TIM1 instance */
  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = DEFAULT_TIM_PERIOD;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if (HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Output Compare channels */

  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = STBOX1_UPDATE_LED_BATTERY;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = STBOX1_UPDATE_SENSOR_FUSION;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
#ifdef STBOX1_ENABLE_PRINTF
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = STBOX1_UPDATE_VCOM;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
#endif /* STBOX1_ENABLE_PRINTF */

  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = STBOX1_UPDATE_IDWG;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On(LED_BLUE);
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED_BLUE);
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
  BSP_LED_Toggle(LED_BLUE);
}

/** @brief  This function initializes the LED
  * @param  None
  * @retval None
  */
void LedInitTargetPlatform(void)
{
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
}

#ifdef STBOX1_ENABLE_PRINTF
/**
  * @brief  Read from VCOM
  * @param  char *buffer Pointer to buffer.
  * @param  uint32_t len_maxData max. length.
  * @retval Number of really read data bytes.
  */
uint32_t VCOM_read(char *buffer, uint32_t len_max)
{
  /* VCOM data receive not completed or no VCOM data received at all. */
  if (VCOM_RxData == 0)
  {
    return 0;
  }

  /* ERROR: No VCOM data ready. */
  if (VCOM_RxLength == 0 || VCOM_RxBuffer == NULL)
  {
    Error_Handler();
  }

  /* Read all data */
  if (VCOM_RxLength <= len_max)
  {
    uint32_t len = VCOM_RxLength;
    memcpy((uint8_t *)buffer, (uint8_t *)VCOM_RxBuffer, len);

    VCOM_RxData   = 0;
    VCOM_RxBuffer = NULL;
    VCOM_RxLength = 0;

    CDC_Next_Packet_Rx();
    return len;
  }
  else
  {
    /* Read part of data that fits into buffer. */
    memcpy((uint8_t *)buffer, (uint8_t *)VCOM_RxBuffer, len_max);

    VCOM_RxBuffer += len_max;
    VCOM_RxLength -= len_max;

    return len_max;
  }
}
#endif /* define STBOX1_ENABLE_PRINTF */


/**
  * @brief  Enable Disable the jump to second flash bank and reboot board
  * @param  None
  * @retval None
  */
void EnableDisableDualBoot(void)
{
  FLASH_OBProgramInitTypeDef    OBInit;
  /* Set BFB2 bit to enable boot from Flash Bank2 */
  /* Allow Access to Flash control registers and user Flash */
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  /* Allow Access to option bytes sector */
  HAL_FLASH_OB_Unlock();

  /* Get the Dual boot configuration status */
  HAL_FLASHEx_OBGetConfig(&OBInit);

  /* Enable/Disable dual boot feature */
  OBInit.OptionType = OPTIONBYTE_USER;
  OBInit.USERType   = OB_USER_BFB2;

  if (((OBInit.USERConfig) & (OB_BFB2_ENABLE)) == OB_BFB2_ENABLE)
  {
    OBInit.USERConfig = OB_BFB2_DISABLE;
    STBOX1_PRINTF("->Disable DualBoot\r\n");
  }
  else
  {
    OBInit.USERConfig = OB_BFB2_ENABLE;
    STBOX1_PRINTF("->Enable DualBoot\r\n");
  }

  if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK)
  {
    /*
    Error occurred while setting option bytes configuration.
    User can add here some code to deal with this error.
    To know the code error, user can call function 'HAL_FLASH_GetError()'
    */
    while (1);
  }

  /* Start the Option Bytes programming process */
  if (HAL_FLASH_OB_Launch() != HAL_OK)
  {
    /*
    Error occurred while reloading option bytes configuration.
    User can add here some code to deal with this error.
    To know the code error, user can call function 'HAL_FLASH_GetError()'
    */
    while (1);
  }
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
}

/**
  * @brief  Initializes the Watch Dog
  * @param  int32_t Seconds Number of seconds (1<= Sec <=32)
  * @retval None
  */
void InitWatchDog(int32_t Seconds)
{
  /* With the Prescaler ==256 the Watchdog could measure up to 32 seconds */
  if ((Seconds < 1) | (Seconds > 32))
  {
    /* Initialization Error */
    while (1);
  }

  IwdgHandle.Instance = IWDG;
  IwdgHandle.Init.Prescaler = IWDG_PRESCALER_256;
  IwdgHandle.Init.Reload = ((32000 /* Frequency LSI */) / 256) * Seconds;
  IwdgHandle.Init.Window = IWDG_WINDOW_DISABLE;

  if (HAL_IWDG_Init(&IwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    while (1);
  }
}

