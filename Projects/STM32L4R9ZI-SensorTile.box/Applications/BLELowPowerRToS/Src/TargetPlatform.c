/**
  ******************************************************************************
  * @file    BLELowPowerRToS\Src\TargetPlatform.c
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
TargetFeatures_t TargetBoardFeatures;
uint16_t PCM_Buffer[PCM_AUDIO_IN_SAMPLES];
BSP_AUDIO_Init_t MicParams;

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
static void Init_MEM1_Sensors(void);
static void Init_MEMS_Mics(uint32_t AudioFreq);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform()
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

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));

  /* Discovery and Initialize the MEMS's Sensors */
  Init_MEM1_Sensors();

  /* Disable all the MEMS's Sensors */
  DisableMotionSensors();
  DisableEnvSensors();

  /* Default Microphones' Audio Volume */
  TargetBoardFeatures.AudioVolume = AUDIO_VOLUME_VALUE;
}

/** @brief enable all the Inertial MEMS1 sensors
  * @param None
  * @retval None
  */
void EnableMotionSensors(void)
{
  if (TargetBoardFeatures.HandleAccSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_MOTION_SENSOR_Enable(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Enabled Accelero Sensor\r\n");
    }
  }

  if (TargetBoardFeatures.HandleGyroSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_MOTION_SENSOR_Enable(TargetBoardFeatures.HandleGyroSensor, MOTION_GYRO) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Enabled Gyroscope Sensor\r\n");
    }
  }

  if (TargetBoardFeatures.HandleMagSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_MOTION_SENSOR_Enable(TargetBoardFeatures.HandleMagSensor, MOTION_MAGNETO) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Enabled Magneto Sensor\r\n");
    }
  }
}

/** @brief disable all the Inertial MEMS1 sensors
  * @param None
  * @retval None
  */
void DisableMotionSensors(void)
{
  if (TargetBoardFeatures.HandleAccSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_MOTION_SENSOR_Disable(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Disabled Accelero Sensor\r\n");
    }
  }

  if (TargetBoardFeatures.HandleGyroSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_MOTION_SENSOR_Disable(TargetBoardFeatures.HandleGyroSensor, MOTION_GYRO) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Disabled Gyroscope Sensor\r\n");
    }
  }

  if (TargetBoardFeatures.HandleMagSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_MOTION_SENSOR_Disable(TargetBoardFeatures.HandleMagSensor, MOTION_MAGNETO) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Disabled Magneto Sensor\r\n");
    }
  }
}

/** @brief enable all the Environmental MEMS1 sensors
  * @param None
  * @retval None
  */
void EnableEnvSensors(void)
{
  if (TargetBoardFeatures.HandleHumSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_ENV_SENSOR_Enable(TargetBoardFeatures.HandleHumSensor, ENV_HUMIDITY) == BSP_ERROR_NONE)
    {
#ifdef ONE_SHOT
      STBOX1_PRINTF("Enabled Humidity Sensor (One Shot)\r\n");
#else /* ONE_SHOT */
      STBOX1_PRINTF("Enabled Humidity Sensor\r\n");
#endif /* ONE_SHOT */
    }
  }

  if (TargetBoardFeatures.HandleTempSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_ENV_SENSOR_Enable(TargetBoardFeatures.HandleTempSensor, ENV_TEMPERATURE) == BSP_ERROR_NONE)
    {
#ifdef ONE_SHOT
      STBOX1_PRINTF("Enabled Temperature Sensor (One Shot)\r\n");
#else /* ONE_SHOT */
      STBOX1_PRINTF("Enabled Temperature Sensor\r\n");
#endif /* ONE_SHOT */
    }
  }

  if (TargetBoardFeatures.HandlePressSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_ENV_SENSOR_Enable(TargetBoardFeatures.HandlePressSensor, ENV_PRESSURE) == BSP_ERROR_NONE)
    {
#ifdef ONE_SHOT
      STBOX1_PRINTF("Enabled Pressure Sensor (One Shot)\r\n");
#else /* ONE_SHOT */
      STBOX1_PRINTF("Enabled Pressure Sensor\r\n");
#endif /* ONE_SHOT */
    }
  }
}

/** @brief disable all the Environmental MEMS1 sensors
  * @param None
  * @retval None
  */
void DisableEnvSensors(void)
{
  if (TargetBoardFeatures.HandleHumSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_ENV_SENSOR_Disable(TargetBoardFeatures.HandleHumSensor, ENV_HUMIDITY) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Disabled Humidity Sensor\r\n");
    }
  }

  if (TargetBoardFeatures.HandleTempSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_ENV_SENSOR_Disable(TargetBoardFeatures.HandleTempSensor, ENV_TEMPERATURE) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Disabled Temperature Sensor\r\n");
    }
  }

  if (TargetBoardFeatures.HandlePressSensor != STBOX1_SNS_NOT_VALID)
  {
    if (BSP_ENV_SENSOR_Disable(TargetBoardFeatures.HandlePressSensor, ENV_PRESSURE) == BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Disabled Pressure Sensor\r\n");
    }
  }
}

/** @brief Initialize all the MEMS1 sensors
  * @param None
  * @retval None
  */
static void Init_MEM1_Sensors(void)
{
  /* Handles for SensorTile.box board */
  TargetBoardFeatures.HandleAccSensor  = LSM6DSOX_0;
  TargetBoardFeatures.HandleGyroSensor = LSM6DSOX_0;
  TargetBoardFeatures.HandleMagSensor  = LIS2MDL_0;

  TargetBoardFeatures.HandleHumSensor      = HTS221_0;

  TargetBoardFeatures.HandlePressSensor    = LPS22HH_0;

  TargetBoardFeatures.HandleTempSensor = STTS751_0;

  /* Accelero/Gyro It's necessary to Init the Component ONLY one Time */
  if (BSP_MOTION_SENSOR_Init(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Accelero/Gyroscope Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Accelero/Gyroscope Sensor\n\r");
    TargetBoardFeatures.HandleAccSensor  = STBOX1_SNS_NOT_VALID;
    TargetBoardFeatures.HandleGyroSensor = STBOX1_SNS_NOT_VALID;
    /* if the Acc is missing... it's blocking for this application !! */
    while (1);
  }

  /* Set Full Scale to +/-2g */
  BSP_MOTION_SENSOR_SetFullScale(TargetBoardFeatures.HandleAccSensor, MOTION_ACCELERO, 2);

  /* For accelero HW features */
  InitHWFeatures();

  if (BSP_MOTION_SENSOR_Init(TargetBoardFeatures.HandleMagSensor, MOTION_MAGNETO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Magneto Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Magneto Sensor\n\r");
    TargetBoardFeatures.HandleMagSensor = STBOX1_SNS_NOT_VALID;
  }

  if (BSP_ENV_SENSOR_Init(TargetBoardFeatures.HandleHumSensor, ENV_HUMIDITY) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Humidity Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Humidity Sensor\n\r");
    TargetBoardFeatures.HandleHumSensor      = STBOX1_SNS_NOT_VALID;
  }

  if (BSP_ENV_SENSOR_Init(TargetBoardFeatures.HandleTempSensor, ENV_TEMPERATURE) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Temperature Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Temperature Sensor\n\r");
    TargetBoardFeatures.HandleTempSensor = STBOX1_SNS_NOT_VALID;
  }

  if (BSP_ENV_SENSOR_Init(TargetBoardFeatures.HandlePressSensor, ENV_PRESSURE) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Pressure Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Pressure Sensor\n\r");
    TargetBoardFeatures.HandlePressSensor    = STBOX1_SNS_NOT_VALID;
  }

  /* Enable interruption LSM6DSOX  */
  {
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  }
}

/** @brief Initialize all the MEMS's Microphones
  * @param None
  * @retval None
  */
static void Init_MEMS_Mics(uint32_t AudioFreq)
{
  uint8_t ret;

  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr = 1;
  MicParams.Device = AMIC_ONBOARD;
  MicParams.SampleRate = AudioFreq;
  MicParams.Volume = TargetBoardFeatures.AudioVolume;

  ret = BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams);

  if (ret != BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("\nError Audio Init\r\n");
    while (1)
    {
      ;
    }
  }
  else
  {
    STBOX1_PRINTF("\nOK Audio Init\t(Audio Freq.= %ld)\r\n", AudioFreq);
  }

  /* Set the volume level */
  ret = BSP_AUDIO_IN_SetVolume(BSP_AUDIO_IN_INSTANCE, TargetBoardFeatures.AudioVolume);

  if (ret != BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("Error Audio Volume\r\n\n");

    while (1)
    {
      ;
    }
  }
  else
  {
    STBOX1_PRINTF("OK Audio Volume\t(Volume= %ld)\r\n", TargetBoardFeatures.AudioVolume);
  }
}

/** @brief Initialize all the MEMS's Microphones
  * @param None
  * @retval None
  */
void InitMics(uint32_t AudioFreq)
{
  Init_MEMS_Mics(AudioFreq);
  BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t *) PCM_Buffer, PCM_AUDIO_IN_SAMPLES * 2);
}

/** @brief DeInitialize all the MEMS's Microphones
  * @param None
  * @retval None
  */
void DeInitMics(void)
{
  uint8_t ret = BSP_ERROR_NONE;

  BSP_AUDIO_IN_Stop(BSP_AUDIO_IN_INSTANCE);

  /* The ADC could be used also for Battery Charger */
  ret = BSP_AUDIO_IN_DeInit(BSP_AUDIO_IN_INSTANCE);

  if (ret != BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("Error Audio DeInit\r\n");
    while (1);
  }
  else
  {
    STBOX1_PRINTF("OK Audio DeInit\r\n");
  }
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
