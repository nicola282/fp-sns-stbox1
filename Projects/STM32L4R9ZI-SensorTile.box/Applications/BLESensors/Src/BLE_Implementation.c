/**
  ******************************************************************************
  * @file    BLE_Implementation.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   BLE Implementation template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.c.
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
#include <stdio.h>
#include "BLE_Manager.h"
#include "main.h"
#include "STBOX1_config.h"

/* Exported Variables --------------------------------------------------------*/
volatile uint8_t connected = FALSE;

uint8_t BatteryTimerEnabled = 0;

/* Private variables ------------------------------------------------------------*/
static uint8_t EnvironmentalTimerEnabled = 0;
static uint8_t InertialTimerEnabled = 0;

/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length);
static void ReadRequestEnvFunction(int32_t *Press, uint16_t *Hum, int16_t *Temp1, int16_t *Temp2);
static void DisconnectionCompletedFunction(void);
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6]);

static void NotifyEventBattery(BLE_NotifyEvent_t Event);
static void NotifyEventEnv(BLE_NotifyEvent_t Event);
static void NotifyEventInertial(BLE_NotifyEvent_t Event);

static void TIM1_CHANNEL_1_StartStop(BLE_NotifyEvent_t Event);
static void TIM1_CHANNEL_3_StartStop(BLE_NotifyEvent_t Event);
static void TIM1_CHANNEL_4_StartStop(BLE_NotifyEvent_t Event);

/* Private defines -----------------------------------------------------------*/
/* Default Secure PIN */
#define SECURE_PIN 123456

/**********************************************************************************************
  * Callback functions prototypes to manage the extended configuration characteristic commands *
  **********************************************************************************************/
static void ExtExtConfigUidCommandCallback(uint8_t **UID);
static void ExtConfigInfoCommandCallback(uint8_t *Answer);
static void ExtConfigHelpCommandCallback(uint8_t *Answer);
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer);

/** @brief Initialize the BlueNRG stack and services
  * @param  None
  * @retval None
  */
void BluetoothInit(void)
{
  /* BlueNRG stack setting */

  BLE_StackValue.ConfigValueOffsets                   = CONFIG_DATA_PUBADDR_OFFSET;
  BLE_StackValue.ConfigValuelength                    = CONFIG_DATA_PUBADDR_LEN;
  BLE_StackValue.GAP_Roles                            = GAP_PERIPHERAL_ROLE;
  BLE_StackValue.IO_capabilities                      = IO_CAP_DISPLAY_ONLY;
  BLE_StackValue.AuthenticationRequirements           = BONDING;
  BLE_StackValue.MITM_ProtectionRequirements          = MITM_PROTECTION_REQUIRED;
  BLE_StackValue.SecureConnectionSupportOptionCode    = SC_IS_SUPPORTED;
  BLE_StackValue.SecureConnectionKeypressNotification = KEYPRESS_IS_NOT_SUPPORTED;

  /* Use BLE Random Address */
  BLE_StackValue.OwnAddressType = RANDOM_ADDR;

  /* To set the TX power level of the bluetooth device ( -2,1 dBm )*/
  BLE_StackValue.EnableHighPowerMode = 1; /*  High Power */

  /* Values: 0x00 ... 0x31 - The value depends on the device */
  BLE_StackValue.PowerAmplifierOutputLevel = 4;

  /* BlueNRG services setting */
  BLE_StackValue.EnableConfig    = 1;
  BLE_StackValue.EnableConsole   = 1;
  BLE_StackValue.EnableExtConfig = 1;

  /* For Enabling the Secure Connection */
  BLE_StackValue.EnableSecureConnection = 0;
  /* Default Secure PIN */
  BLE_StackValue.SecurePIN = SECURE_PIN;
  /* For creating a Random Secure PIN */
  BLE_StackValue.EnableRandomSecurePIN = 0;

  BLE_StackValue.AdvertisingFilter    = NO_WHITE_LIST_USE;

  if (BLE_StackValue.EnableSecureConnection)
  {
    /* Using the Secure Connection, the Rescan should be done by BLE chip */
    BLE_StackValue.ForceRescan = 0;
  }
  else
  {
    BLE_StackValue.ForceRescan = 1;
  }

  /* Set the BLE Board Name */
  sprintf(BLE_StackValue.BoardName, "%s%c%c%c", "BBS_",
          BLE_VERSION_FW_MAJOR,
          BLE_VERSION_FW_MINOR,
          BLE_VERSION_FW_PATCH);

#ifdef STBOX1_ENABLE_PRINTF
#ifdef STBOX1_RESTART_DFU
  /* Ask if we want to boot in DFU mode */
  {
    int32_t Answer = 0;
    char Buffer[16];
    uint32_t InitTick;

    BLE_MANAGER_PRINTF("Do you want to reboot in DFU mode (y/n) [5sec]?");
    InitTick = HAL_GetTick();
    while (Answer == 0)
    {
      if (VCOM_read(Buffer, 16))
      {
        if (Buffer[0] == 'y')
        {
          DFU_Var = DFU_MAGIC_NUM;
          HAL_NVIC_SystemReset();
        }
        else
        {
          BLE_MANAGER_PRINTF("\r\n\n");
          Answer = 1;
        }
      }
      else
      {
        HAL_Delay(10);
        if ((HAL_GetTick() - InitTick) > 5000)
        {
          BLE_MANAGER_PRINTF("\r\n\tTimeOut\r\n");
          Answer = 1;
        }
      }
    }
  }
#endif /* STBOX1_RESTART_DFU */

  /* Ask if we want to change the board Name (timeout of 5 seconds )*/
  {
    int32_t Answer = 0;
    char Buffer[16];
    uint32_t InitTick;

    BLE_MANAGER_PRINTF("Default BLE board Name [%s]\r\n", BLE_StackValue.BoardName);
    BLE_MANAGER_PRINTF("\tDo you want change it (y/n) [5sec]?");
    InitTick = HAL_GetTick();
    while (Answer == 0)
    {
      if (VCOM_read(Buffer, 16))
      {
        if (Buffer[0] == 'y')
        {
          int32_t NumBytes = 0;
          BLE_MANAGER_PRINTF("\r\n\tWrite the Name (7 Chars): _______\b\b\b\b\b\b\b");
          while (NumBytes != 7)
          {
            if (VCOM_read(Buffer, 16))
            {
              if (Buffer[0] != '\b')
              {
                if (Buffer[0] == '\r')
                {
                  for (; NumBytes < 7; NumBytes++)
                  {
                    BLE_StackValue.BoardName[NumBytes] = ' ';
                  }
                }
                else
                {
                  BLE_StackValue.BoardName[NumBytes] = Buffer[0];
                  NumBytes++;
                }
              }
              else
              {
                if (NumBytes > 0)
                {
                  NumBytes--;
                }
              }
            }
          }
          BLE_MANAGER_PRINTF("\r\n\tNew Name=[%s]\r\n\n", BLE_StackValue.BoardName);
          Answer = 1;
        }
        else
        {
          BLE_MANAGER_PRINTF("\r\n\n");
          Answer = 1;
        }
      }
      else
      {
        HAL_Delay(10);
        if ((HAL_GetTick() - InitTick) > 5000)
        {
          BLE_MANAGER_PRINTF("\r\n\tTimeOut\r\n");
          Answer = 1;
        }
      }
    }
  }
#endif /* STBOX1_ENABLE_PRINTF */

  BLE_StackValue.BoardId = BLE_MANAGER_SENSOR_TILE_BOX_PLATFORM;

  InitBleManager();
}

/**
  * @brief  Custom Service Initialization.
  * @param  None
  * @retval None
  */
void BLE_InitCustomService(void)
{
  /* Define Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = &DebugConsoleCommandParsing;

  /* Define Custom Function for Connection Completed */
  CustomConnectionCompleted = &ConnectionCompletedFunction;

  /* Define Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = &DisconnectionCompletedFunction;

  /**************************************************
    * Callback functions to manage the notify events *
    **************************************************/
  CustomNotifyEventBattery =             &NotifyEventBattery;
  CustomNotifyEventEnv =                 &NotifyEventEnv;
  CustomNotifyEventInertial =            &NotifyEventInertial;

  /***********************************************************************************
    * Callback functions to manage the extended configuration characteristic commands *
    ***********************************************************************************/
  CustomExtConfigUidCommandCallback  = &ExtExtConfigUidCommandCallback;
  CustomExtConfigInfoCommandCallback = &ExtConfigInfoCommandCallback;
  CustomExtConfigHelpCommandCallback = &ExtConfigHelpCommandCallback;
  CustomExtConfigVersionFwCommandCallback = &ExtConfigVersionFwCommandCallback;

  /**
    * For each features, user can assign here the pointer at the function for the read request data.
    * For example for the environmental features:
    *
    * CustomReadRequestEnv = &ReadRequestEnvFunction;
    *
    * User can define and insert in the BLE_Implementation.c source code the functions for the read request data
    * ReadRequestEnvFunction function is already defined.
    *
  */

  /* Define Custom Function for Read Request Environmental Data */
  CustomReadRequestEnv = &ReadRequestEnvFunction;

  /*******************
    * User code begin *
    *******************/

  /**
    * User can added here the custom service initialization for the selected BLE features.
    * For example for the environmental features:
    *
    * //BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled)
    * BleManagerAddChar(BleCharPointer= BLE_InitEnvService(1, 1, 1));
  */

  /* Service initialization and adding for the environmental features */
  /* BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled) */
  BleManagerAddChar(BLE_InitEnvService(1, 1, 1));

  /* Service initialization and adding  for the inertial features */
  /* BLE_InitInertialService(AccEnable,GyroEnable,MagEnabled) */
  BleManagerAddChar(BLE_InitInertialService(1, 1, 1));

  /* Custom service initialization for the battery features */
  BleManagerAddChar(BLE_InitBatteryService());

  /*****************
    * User code end *
    *****************/
}

/**
  * @brief  Set Custom Advertize Data.
  * @param  uint8_t *manuf_data: Advertize Data
  * @retval None
  */
void BLE_SetCustomAdvertiseData(uint8_t *manuf_data)
{
  /**
    * User can add here the custom advertize data setting  for the selected BLE features.
    * For example for the environmental features:
    *
    * BLE_SetCustomEnvAdvertizeData(manuf_data);
  */

#ifndef BLE_MANAGER_SDKV2
  /* Custom advertize data setting for the environmental features */
  BLE_SetEnvAdvertizeData(manuf_data);

  /* Custom advertize data setting for the inertial features */
  BLE_SetInertialAdvertizeData(manuf_data);

  /* Custom advertize data setting for the battery features */
  BLE_SetBatteryAdvertizeData(manuf_data);

#else /* BLE_MANAGER_SDKV2 */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1] = 0x12;
#endif /* BLE_MANAGER_SDKV2 */
}

/**
  * @brief  This function makes the parsing of the Debug Console Commands
  * @param  uint8_t *att_data attribute data
  * @param  uint8_t data_length length of the data
  * @retval uint32_t SendBackData true/false
  */
static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

  /* Help Command */
  if (!strncmp("help", (char *)(att_data), 4))
  {
    /* Print Legend */
    SendBackData = 0;

    BytesToWrite = sprintf((char *)BufferToWrite,
                           "info\n"
#ifdef STBOX1_RESTART_DFU
                           "DFU\n"
#endif /* STBOX1_RESTART_DFU */
                           "Off\n");
    Term_Update(BufferToWrite, BytesToWrite);

  }
  else if (!strncmp("info", (char *)(att_data), 4))
  {
    SendBackData = 0;

    BytesToWrite = sprintf((char *)BufferToWrite, "\r\nSTMicroelectronics %s:\n"
                           "\tVersion %c.%c.%c\n"
                           "\tSTM32L4R9ZI-SensorTile.box board"
                           "\n",
                           BLE_FW_PACKAGENAME,
                           BLE_VERSION_FW_MAJOR, BLE_VERSION_FW_MINOR, BLE_VERSION_FW_PATCH);
    Term_Update(BufferToWrite, BytesToWrite);

    BytesToWrite = sprintf((char *)BufferToWrite, "\t(HAL %ld.%ld.%ld_%ld)\n"
                           "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                           " (IAR)\n",
#elif defined (__ARMCC_VERSION)
                           " (KEIL)\n",
#elif defined (__GNUC__)
                           " (STM32CubeIDE)\n",
#endif /* IDE */
                           HAL_GetHalVersion() >> 24,
                           (HAL_GetHalVersion() >> 16) & 0xFF,
                           (HAL_GetHalVersion() >> 8) & 0xFF,
                           HAL_GetHalVersion()      & 0xFF,
                           __DATE__, __TIME__);
    Term_Update(BufferToWrite, BytesToWrite);
#ifdef STBOX1_RESTART_DFU
  }
  else if (!strncmp("DFU", (char *)(att_data), 3))
  {
    SendBackData = 0;
    BytesToWrite = sprintf((char *)BufferToWrite, "\r\n5 sec for restarting\r\n\tin DFU mode\r\n");
    Term_Update(BufferToWrite, BytesToWrite);
    HAL_Delay(5000);
    DFU_Var = DFU_MAGIC_NUM;
    HAL_NVIC_SystemReset();
#endif /* STBOX1_RESTART_DFU */
  }
  else if (!strncmp("uid", (char *)(att_data), 3))
  {
    /* Write back the STM32 UID */
    uint8_t *uid = (uint8_t *)STM32_UUID;
    uint32_t MCU_ID = STM32_MCU_ID[0] & 0xFFF;
    BytesToWrite = sprintf((char *)BufferToWrite, "%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\n",
                           uid[ 3], uid[ 2], uid[ 1], uid[ 0],
                           uid[ 7], uid[ 6], uid[ 5], uid[ 4],
                           uid[11], uid[ 10], uid[9], uid[8],
                           MCU_ID);
    Term_Update(BufferToWrite, BytesToWrite);
    SendBackData = 0;
  }
  else if (!strncmp("Off", (char *)(att_data), 3))
  {
    PowerButtonPressed = 1;
    SendBackData = 0;
  }
  return SendBackData;
}

/**
  * @brief  This function is called when there is a Bluetooth Read request.
  * @param  int32_t *Press Pressure Value
  * @param  uint16_t *Hum Humidity Value
  * @param  int16_t *Temp1 Temperature Number 1
  * @param  int16_t *Temp2 Temperature Number 2
  * @retval None
  */
static void ReadRequestEnvFunction(int32_t *Press, uint16_t *Hum, int16_t *Temp1, int16_t *Temp2)
{
  /* Read Request for Pressure,Humidity, and Temperatures*/
  int32_t PressToSend;
  uint16_t HumToSend;
  int16_t TempToSend;

  /* Read all the Environmental Sensors */
  ReadEnvironmentalData(&PressToSend, &HumToSend, &TempToSend);

  *Press = PressToSend;
  *Hum   = HumToSend;
  *Temp1 = TempToSend;
  *Temp2 = 0;

  BLE_MANAGER_PRINTF("Read for Env\r\n");
}

/**
  * @brief  This function is called when the peer device get disconnected.
  * @param  None
  * @retval None
  */
static void DisconnectionCompletedFunction(void)
{
  connected = FALSE;

  /* Disable all timer */
  if (BatteryTimerEnabled)
  {
    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    BatteryTimerEnabled = 0;
  }

  if (EnvironmentalTimerEnabled)
  {
    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    EnvironmentalTimerEnabled = 0;
  }

  if (InertialTimerEnabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    InertialTimerEnabled = 0;
  }

  BLE_MANAGER_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
  * @brief  This function is called when there is a LE Connection Complete event.
  * @param  None
  * @retval None
  */
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6])
{
  connected = TRUE;

  /* Stop the TIM Base generation in interrupt mode for Led Blinking*/
  if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Stopping Error */
    Error_Handler();
  }

  /* Just in order to be sure to switch off the User Led */
  LedOffTargetPlatform();

  BLE_MANAGER_PRINTF("Call to ConnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**************************************************
  * Callback functions to manage the notify events *
  **************************************************/

/**
  * @brief  Callback Function for Un/Subscription Battery Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventBattery(BLE_NotifyEvent_t Event)
{
  /* Battery Features */
  if (Event != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_1_StartStop(Event);
  }
}

/**
  * @brief  Callback Function for Un/Subscription Environmental Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventEnv(BLE_NotifyEvent_t Event)
{
  /* Environmental Features */
  if (Event != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_3_StartStop(Event);
  }
}

/**
  * @brief  Callback Function for Un/Subscription Inertial Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventInertial(BLE_NotifyEvent_t Event)
{
  /* Inertial Features */
  if (Event != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_4_StartStop(Event);
  }
}

/**********************************/
/* Characteristics Notify Service */
/**********************************/
/**
  * @brief  Enable/Disable TIM1 Channel 1
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_1_StartStop(BLE_NotifyEvent_t Event)
{
  if ((Event == BLE_NOTIFY_SUB) &&
      (!BatteryTimerEnabled))
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    BSP_BC_CmdSend(BATMS_ON);
    BatteryTimerEnabled = 1;

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED_BATTERY));
  }

  if ((Event == BLE_NOTIFY_UNSUB) &&
      (BatteryTimerEnabled))
  {
    BSP_BC_CmdSend(BATMS_OFF);

    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    BatteryTimerEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 3
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_3_StartStop(BLE_NotifyEvent_t Event)
{
  if ((Event == BLE_NOTIFY_SUB) &&
      (!EnvironmentalTimerEnabled))
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + STBOX1_UPDATE_ENV));

    EnvironmentalTimerEnabled = 1;
  }

  if ((Event == BLE_NOTIFY_UNSUB) &&
      (EnvironmentalTimerEnabled))
  {
    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    EnvironmentalTimerEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 4
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_4_StartStop(BLE_NotifyEvent_t Event)
{
  if ((Event == BLE_NOTIFY_SUB) &&
      (!InertialTimerEnabled))
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + STBOX1_UPDATE_INV));

    InertialTimerEnabled = 1;
  }

  if ((Event == BLE_NOTIFY_UNSUB) &&
      (InertialTimerEnabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    InertialTimerEnabled = 0;
  }
}

/***********************************************************************************
  * Callback functions to manage the extended configuration characteristic commands *
  ***********************************************************************************/

/**
  * @brief  Callback Function for answering to the UID command
  * @param  uint8_t **UID STM32 UID Return value
  * @retval None
  */
static void ExtExtConfigUidCommandCallback(uint8_t **UID)
{
  *UID = (uint8_t *)STM32_UUID;
}


/**
  * @brief  Callback Function for answering to Info command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
static void ExtConfigInfoCommandCallback(uint8_t *Answer)
{
  uint8_t  hwVersion;
  uint16_t fwVersion;

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  sprintf((char *)Answer, "STMicroelectronics %s:\n"
          "Version %c.%c.%c\n"
          "%s board\n"
          "BlueNRG-2 HW ver%d.%d\n"
          "BlueNRG-2 FW ver%d.%d.%c\n"
          "(HAL %ld.%ld.%ld_%ld)\n"
          "Compiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
          " (IAR)"
#elif defined (__ARMCC_VERSION)
          " (KEIL)"
#elif defined (__GNUC__)
          " (STM32CubeIDE)"
#endif /* IDE */
          "\n",
          BLE_FW_PACKAGENAME,
          BLE_VERSION_FW_MAJOR,
          BLE_VERSION_FW_MINOR,
          BLE_VERSION_FW_PATCH,
          BLE_STM32_BOARD,
          ((hwVersion >> 4) & 0x0F),
          (hwVersion & 0x0F),
          (fwVersion >> 8) & 0xF,
          (fwVersion >> 4) & 0xF,
          ('a' + (fwVersion & 0xF)),
          HAL_GetHalVersion() >> 24,
          (HAL_GetHalVersion() >> 16) & 0xFF,
          (HAL_GetHalVersion() >> 8) & 0xFF,
          HAL_GetHalVersion()      & 0xFF,
          __DATE__, __TIME__);
}

/**
  * @brief  Callback Function for answering to Help command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
static void ExtConfigHelpCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer, "List of available command:\n"
          "1) Board Report\n"
          "- STM32 UID\n"
          "- Version Firmware\n"
          "- Info\n"
          "- Help\n\n");
}

/**
  * @brief  Callback Function for answering to VersionFw command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer, "%s_%s_%c.%c.%c",
          BLE_STM32_MICRO,
          BLE_FW_PACKAGENAME,
          BLE_VERSION_FW_MAJOR,
          BLE_VERSION_FW_MINOR,
          BLE_VERSION_FW_PATCH);
}

