/**
  ******************************************************************************
  * @file    SDDataLogRToS\Src\TargetPlatform.c
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
#include "datalog_application.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
uint16_t PCM_Buffer[AUDIO_CHANNELS * PCM_AUDIO_IN_SAMPLES];
BSP_AUDIO_Init_t MicParams;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform()
{
  /* Init Led1/Led2/Led3 */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  MCR_HEART_BIT();

  /* Initialize User Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize the MEMS's Sensors */
  /* Magneto */
  BSP_MOTION_SENSOR_Init(LIS2MDL_0, MOTION_MAGNETO);
  BSP_MOTION_SENSOR_SetOutputDataRate(LIS2MDL_0, MOTION_MAGNETO, 100.0f);
  /* FS = 50gauss */
  BSP_MOTION_SENSOR_SetFullScale(LIS2MDL_0, MOTION_MAGNETO, 50);

  /* Acc/Gyro */
  BSP_MOTION_SENSOR_Init(LSM6DSOX_0, MOTION_ACCELERO | MOTION_GYRO);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSOX_0, MOTION_GYRO, 104.0f);
  /* FS = 2000dps */
  BSP_MOTION_SENSOR_SetFullScale(LSM6DSOX_0, MOTION_GYRO, 2000);
  BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSOX_0, MOTION_ACCELERO, 100.0f);
  /* FS = 2g */
  BSP_MOTION_SENSOR_SetFullScale(LSM6DSOX_0, MOTION_ACCELERO, 2);

  /* Humidity */
  BSP_ENV_SENSOR_Init(HTS221_0, ENV_HUMIDITY);
  BSP_ENV_SENSOR_SetOutputDataRate(HTS221_0, ENV_HUMIDITY, 12.5f);

  /* Temperature */
  BSP_ENV_SENSOR_Init(STTS751_0, ENV_TEMPERATURE);
  BSP_ENV_SENSOR_SetOutputDataRate(STTS751_0, ENV_TEMPERATURE, 32.0f);

  /* Pressure */
  BSP_ENV_SENSOR_Init(LPS22HH_0, ENV_PRESSURE);
  BSP_ENV_SENSOR_SetOutputDataRate(LPS22HH_0, ENV_PRESSURE, 100.0f);

  /* Initialize the local filesystem and volume driver */
  DATALOG_SD_Init();

  /* Initializes the Microphone */
  MicParams.BitsPerSample = 16;
  MicParams.ChannelsNbr   = AUDIO_CHANNELS;
  MicParams.Device        = AMIC_ONBOARD;
  MicParams.SampleRate    = AUDIO_SAMPLING_FREQUENCY;
  MicParams.Volume        = AUDIO_VOLUME_VALUE;

  if (BSP_AUDIO_IN_Init(BSP_AUDIO_IN_INSTANCE, &MicParams) != BSP_ERROR_NONE)
  {
    ErrorHanlder(STBOX1_AUDIO_ERROR);
  }

  /* Just for signalizing that everything is ok */
  MCR_HEART_BIT();
}

/** @brief  This function Blink the LED in function of Error Code
  * @param  ErrorType_t ErrorType Error Code
  * @retval None
  */
void ErrorHanlder(ErrorType_t ErrorType)
{
  ErrorType_t CountError;
  while (1)
  {
    for (CountError = STBOX1_INIT_ERROR; CountError < ErrorType; CountError++)
    {
      BSP_LED_On(LED_RED);
      HAL_Delay(200);
      BSP_LED_Off(LED_RED);
      HAL_Delay(1000);
    }
    HAL_Delay(5000);
  }
}

