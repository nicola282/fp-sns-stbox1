/**
  ******************************************************************************
  * @file    BLESensors\Src\TargetPlatform.c
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

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void InitTimers(void);
static void Init_MEM1_Sensors(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
  /* Init Led1/Led2 */
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

  /* Initialize the MEMS's Sensors */
  Init_MEM1_Sensors();

  /* Initializes the Timers */
  InitTimers();

  MCR_HEART_BIT();
  MCR_HEART_BIT();

#ifdef STBOX1_ENABLE_PRINTF
  BSP_LED_On(LED_BLUE);
  BSP_LED_On(LED_GREEN);

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
  BSP_LED_Off(LED_GREEN);

#endif /* STBOX1_ENABLE_PRINTF */
}

/** @brief Initialize all the MEMS1 sensors
  * @param None
  * @retval None
  */
static void Init_MEM1_Sensors(void)
{
  /* Accelero/Gyro  */
  if (BSP_MOTION_SENSOR_Init(LSM6DSOX_0, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Accelero/Gyroscope Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Accelero/Gyroscope Sensor\n\r");
    /* Starting Error */
    Error_Handler();
  }

  /* Magneto */
  if (BSP_MOTION_SENSOR_Init(LIS2MDL_0, MOTION_MAGNETO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Magneto Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Magneto Sensor\n\r");
    /* Starting Error */
    Error_Handler();
  }

  /* Humidity  */
  if (BSP_ENV_SENSOR_Init(HTS221_0, ENV_HUMIDITY) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Humidity Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Humidity Sensor\n\r");
    /* Starting Error */
    Error_Handler();
  }

  /* Temperature */
  if (BSP_ENV_SENSOR_Init(STTS751_0, ENV_TEMPERATURE) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Temperature Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Temperature Sensor\n\r");
    /* Starting Error */
    Error_Handler();
  }

  /* Pressure */
  if (BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_PRESSURE) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Pressure Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Pressure Sensor\n\r");
    /* Starting Error */
    Error_Handler();
  }
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
  TimCCHandle.Init.Period        = 65535;
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

  /* Output Compare Toggle Mode configuration:
   * Channel 1 for LED Blinking */
  sConfig.Pulse = STBOX1_UPDATE_LED_BATTERY;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

#ifdef STBOX1_ENABLE_PRINTF
  /* Output Compare Toggle Mode configuration:
   * Channel 2 for sending Battery information */
  sConfig.Pulse = STBOX1_UPDATE_VCOM;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
#endif /* STBOX1_ENABLE_PRINTF */

  /* Output Compare Toggle Mode configuration:
   * Channel 3 for sending Environmental Features */
  sConfig.Pulse = STBOX1_UPDATE_ENV;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Output Compare Toggle Mode configuration:
   * Channel 4 for sending Inertial Features */
  sConfig.Pulse = STBOX1_UPDATE_INV;
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

