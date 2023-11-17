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
#include "OTA.h"
#include "main.h"

/* Exported Variables --------------------------------------------------------*/
volatile uint8_t  connected = FALSE;
volatile uint8_t  paired = FALSE;

uint32_t SizeOfUpdateBlueFW = 0;
uint32_t ConnectionBleStatus = 0;
volatile uint32_t RebootBoard  = 0;
volatile uint32_t SwapBanks   = 0;
/* Imported Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef TimCCHandle;
/* Private variables ---------------------------------------------------------*/
static uint32_t NeedToRebootBoard = 0;
static uint32_t NeedToSwapBanks = 0;

/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleParsing(uint8_t *att_data, uint8_t data_length);
static void DisconnectionCompletedFunction(void);
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t Addr[6]);
static void PairingCompletedFunction(uint8_t PairingStatus);

static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length);

static void NotifyEventBattery(BLE_NotifyEvent_t Event);
static void NotifyEventSensorFusion(BLE_NotifyEvent_t Event);

/* Private defines -----------------------------------------------------------*/
/* Default Secure PIN */
#define SECURE_PIN 123456

/* STM32 Unique ID */
#define STM32_UUID ((uint32_t *)0x1FFF7590)

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)

/**********************************************************************************************
  * Callback functions prototypes to manage the extended configuration characteristic commands *
  **********************************************************************************************/
static void ExtExtConfigUidCommandCallback(uint8_t **UID);
static void ExtConfigInfoCommandCallback(uint8_t *Answer);
static void ExtConfigHelpCommandCallback(uint8_t *Answer);
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer);
static void ExtConfigClearDBCommandCallback(void);
static void ExtConfigPowerOffCommandCallback(void);
#ifdef STBOX1_RESTART_DFU
static void ExtConfigRebootOnDFUModeCommandCallback(void);
#endif /* STBOX1_RESTART_DFU */

static void ExtConfigReadBanksFwIdCommandCallback(uint8_t *CurBank, uint16_t *FwId1, uint16_t *FwId2);
static void ExtConfigBanksSwapCommandCallback(void);


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

  /* Set the Board Name */
  sprintf(BLE_StackValue.BoardName, "%s", BLE_FW_PACKAGENAME);

  /* To set the TX power level of the bluetooth device ( -2 dBm )*/
  BLE_StackValue.EnableHighPowerMode = 0; /*  Low Power */

  /* Values: 0x00 ... 0x31 - The value depends on the device */
  BLE_StackValue.PowerAmplifierOutputLevel = 25;

  /* BlueNRG services setting */
  BLE_StackValue.EnableConfig    = 0;
  BLE_StackValue.EnableConsole   = 1;
  BLE_StackValue.EnableExtConfig = 1;

  /* For Enabling the Secure Connection */
  BLE_StackValue.EnableSecureConnection = 1;
  /* Default Secure PIN */
  BLE_StackValue.SecurePIN = SECURE_PIN;
  /* For creating a Random Secure PIN */
  BLE_StackValue.EnableRandomSecurePIN = 0;


  if (BLE_StackValue.EnableSecureConnection)
  {
    /* Using the Secure Connection, the Rescan should be done by BLE chip */
    BLE_StackValue.ForceRescan = 0;
  }
  else
  {
    BLE_StackValue.ForceRescan = 1;
  }

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
  /* Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = &DebugConsoleParsing;

  /* Custom Function for Connection Completed */
  CustomConnectionCompleted = &ConnectionCompletedFunction;

  /* Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = &DisconnectionCompletedFunction;

  /* Custom Pairing Completed Function */
  CustomPairingCompleted = &PairingCompletedFunction;

  /* Service initialization for Sensor Fusion feature */
  if (BleManagerAddChar(BLE_InitSensorFusionService(1)) == 0)
  {
    PRINT_DBG("Error adding SensorFusion Service\r\n");
  }
  CustomNotifyEventSensorFusion = &NotifyEventSensorFusion;

  /* Service initialization for Battery feature */
  if (BleManagerAddChar(BLE_InitBatteryService()) == 0)
  {
    PRINT_DBG("Error adding Battery Service\r\n");
  }
  CustomNotifyEventBattery = &NotifyEventBattery;

  /***********************************************************************************
    * Callback functions to manage the extended configuration characteristic commands *
    ***********************************************************************************/
  CustomExtConfigUidCommandCallback       = &ExtExtConfigUidCommandCallback;
  CustomExtConfigInfoCommandCallback      = &ExtConfigInfoCommandCallback;
  CustomExtConfigHelpCommandCallback      = &ExtConfigHelpCommandCallback;
  CustomExtConfigVersionFwCommandCallback = &ExtConfigVersionFwCommandCallback;
  CustomExtConfigClearDBCommandCallback   = &ExtConfigClearDBCommandCallback;
  CustomExtConfigPowerOffCommandCallback  = &ExtConfigPowerOffCommandCallback;

#ifdef STBOX1_RESTART_DFU
  CustomExtConfigRebootOnDFUModeCommandCallback = &ExtConfigRebootOnDFUModeCommandCallback;
#endif /* STBOX1_RESTART_DFU */

  CustomExtConfigReadBanksFwIdCommandCallback       = &ExtConfigReadBanksFwIdCommandCallback;
  {
    uint16_t FwId1;
    uint16_t FwId2;

    ReadFlashBanksFwId(&FwId1, &FwId2);
    if (FwId2 != OTA_OTA_FW_ID_NOT_VALID)
    {
      /* Enable the BanksSwap only if there is a valid fw on second bank */
      CustomExtConfigBanksSwapCommandCallback = &ExtConfigBanksSwapCommandCallback;
    }
  }
}

/**
  * @brief  Set Custom Advertise Data.
  * @param  uint8_t *manuf_data: Advertise Data
  * @retval None
  */
void BLE_SetCustomAdvertiseData(uint8_t *manuf_data)
{
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1] = STBOX1_BLUEST_SDK_FW_ID;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD2] = CurrentActiveBank;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD3] = 0x00; /* Not Used */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD4] = 0x00; /* Not Used */
}

/**
  * @brief  Callback Function for Un/Subscription SensorFusion Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventSensorFusion(BLE_NotifyEvent_t Event)
{
  if (Event == BLE_NOTIFY_SUB)
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + STBOX1_UPDATE_SENSOR_FUSION));
    STBOX1_PRINTF("Start Fusion\r\n");
  }
  else if (Event == BLE_NOTIFY_UNSUB)
  {

    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }
    STBOX1_PRINTF("Stop Fusion\r\n");
  }
}

/**
  * @brief  Callback Function for Un/Subscription Battery Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventBattery(BLE_NotifyEvent_t Event)
{
  if (Event == BLE_NOTIFY_SUB)
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED_BATTERY));

    /* For enabling the Voltage Output */
    BSP_BC_CmdSend(BATMS_ON);

    STBOX1_PRINTF("Start Battery\r\n");
  }
  else if (Event == BLE_NOTIFY_UNSUB)
  {

    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    /* For Disabling the Voltage Output */
    BSP_BC_CmdSend(BATMS_OFF);

    STBOX1_PRINTF("Stop Battery\r\n");
  }
}

/**
  * @brief  This function makes the parsing of the Debug Console
  * @param  uint8_t *att_data attribute data
  * @param  uint8_t data_length length of the data
  * @retval uint32_t SendBackData true/false
  */
static uint32_t DebugConsoleParsing(uint8_t *att_data, uint8_t data_length)
{
  /* By default Answer with the same message received */
  uint32_t SendBackData = 1;

  if (SizeOfUpdateBlueFW != 0)
  {
    /* Firmware update */
    int8_t RetValue = UpdateFWBlueMS(&SizeOfUpdateBlueFW, att_data, data_length, 1);
    if (RetValue != 0)
    {
      Term_Update(((uint8_t *)&RetValue), 1);
      if (RetValue == 1)
      {
        /* if OTA checked */
        PRINT_DBG("%s will restart after the disconnection\r\n", STBOX1_PACKAGENAME);
        HAL_Delay(1000);
        NeedToSwapBanks = 1;
      }
    }
    SendBackData = 0;
  }
  else
  {
    /* Received one write from Client on Terminal characteristc */
    SendBackData = DebugConsoleCommandParsing(att_data, data_length);
  }

  return SendBackData;
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
                           "clearDB\n");
    Term_Update(BufferToWrite, BytesToWrite);

  }
  else if (!strncmp("versionFw", (char *)(att_data), 9))
  {
    BytesToWrite = sprintf((char *)BufferToWrite, "%s_%s_%c.%c.%c\r\n",
                           "L4R9",
                           STBOX1_PACKAGENAME,
                           STBOX1_VERSION_MAJOR,
                           STBOX1_VERSION_MINOR,
                           STBOX1_VERSION_PATCH);
    Term_Update(BufferToWrite, BytesToWrite);
    SendBackData = 0;
  }
  else if (!strncmp("info", (char *)(att_data), 4))
  {
    SendBackData = 0;

    BytesToWrite = sprintf((char *)BufferToWrite, "\r\nSTMicroelectronics %s:\n"
                           "\tVersion %c.%c.%c\n"
                           "\tSTM32L4R9ZI-SensorTile.box board"
                           "\n",
                           STBOX1_PACKAGENAME,
                           STBOX1_VERSION_MAJOR, STBOX1_VERSION_MINOR, STBOX1_VERSION_PATCH);
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
    BytesToWrite = sprintf((char *)BufferToWrite, "Current Bank =%ld\n", CurrentActiveBank);
    Term_Update(BufferToWrite, BytesToWrite);
  }
  else if (!strncmp("upgradeFw", (char *)(att_data), 9))
  {
    uint32_t uwCRCValue;
    uint8_t *PointerByte = (uint8_t *) &SizeOfUpdateBlueFW;

    PointerByte[0] = att_data[ 9];
    PointerByte[1] = att_data[10];
    PointerByte[2] = att_data[11];
    PointerByte[3] = att_data[12];

    /* Check the Maximum Possible OTA size */
    if (SizeOfUpdateBlueFW > OTA_MAX_PROG_SIZE)
    {
      PRINT_DBG("OTA %s SIZE=%ld > %d Max Allowed\r\n", STBOX1_PACKAGENAME, SizeOfUpdateBlueFW, OTA_MAX_PROG_SIZE);
      /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
      BufferToWrite[0] = att_data[13];
      BufferToWrite[1] = (att_data[14] != 0) ? 0 : 1; /* In order to be sure to have a wrong CRC */
      BufferToWrite[2] = att_data[15];
      BufferToWrite[3] = att_data[16];
      BytesToWrite = 4;
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      PointerByte = (uint8_t *) &uwCRCValue;
      PointerByte[0] = att_data[13];
      PointerByte[1] = att_data[14];
      PointerByte[2] = att_data[15];
      PointerByte[3] = att_data[16];

      PRINT_DBG("OTA %s SIZE=%ld uwCRCValue=%lx\r\n", STBOX1_PACKAGENAME, SizeOfUpdateBlueFW, uwCRCValue);

      /* Reset the Flash */
      StartUpdateFWBlueMS(SizeOfUpdateBlueFW, uwCRCValue);

#if 0
      /* Reduce the connection interval */
      {
        tBleStatus ret = aci_l2cap_connection_parameter_update_req(
                           ConnectionHandle,
                           6 /* interval_min*/,
                           6 /* interval_max */,
                           0   /* slave_latency */,
                           400 /*timeout_multiplier*/);
                           /* Go to infinite loop if there is one error */
                           if (ret != BLE_STATUS_SUCCESS)
      {
        while (1)
          {
            PRINT_DBG("Problem Changing the connection interval\r\n");
          }
        }
      }
#endif /* 0 */

      /* Signal that we are ready sending back the CRC value*/
      BufferToWrite[0] = PointerByte[0];
      BufferToWrite[1] = PointerByte[1];
      BufferToWrite[2] = PointerByte[2];
      BufferToWrite[3] = PointerByte[3];
      BytesToWrite = 4;
      Term_Update(BufferToWrite, BytesToWrite);
    }

    SendBackData = 0;
#ifdef STBOX1_RESTART_DFU
  }
  else if (!strncmp("DFU", (char *)(att_data), 3))
  {
    SendBackData = 0;
    BytesToWrite = sprintf((char *)BufferToWrite, "\n5 sec for restarting\n\tin DFU mode\n");
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
    BytesToWrite = sprintf((char *)BufferToWrite, "\n2 sec for switching off\n");
    Term_Update(BufferToWrite, BytesToWrite);
    HAL_Delay(2000);
    PowerButtonPressed = 1;
    SendBackData = 0;
  }
  else  if (!strncmp("clearDB", (char *)(att_data), 7))
  {
    BytesToWrite = sprintf((char *)BufferToWrite, "\nThe Secure database will be cleared\n");
    Term_Update(BufferToWrite, BytesToWrite);
    NeedToClearSecureDB = 1;
    SendBackData = 0;
  }

  return SendBackData;
}


/**
  * @brief  This function is called when the peer device get disconnected.
  * @param  None
  * @retval None
  */
static void DisconnectionCompletedFunction(void)
{
  connected = FALSE;
  paired    = FALSE;
  set_connectable = TRUE;

  /* Make the device connectable again */
  ConnectionBleStatus = 0;

  /* Reset for any problem during FOTA update */
  SizeOfUpdateBlueFW = 0;

  if (NeedToRebootBoard)
  {
    NeedToRebootBoard = 0;
    RebootBoard = 1;
  }

  if (NeedToSwapBanks)
  {
    NeedToSwapBanks = 0;
    SwapBanks = 1;
  }

  HAL_Delay(100);
}

/**
  * @brief  This function is called when there is a LE Connection Complete event.
  * @param  None
  * @retval None
  */
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t Addr[6])
{
  connected = TRUE;
  ConnectionBleStatus = 0;

#ifdef STBOX1_BLE_SECURE_CONNECTION
  /* Check if the device is already bonded */
  if (aci_gap_is_device_bonded(Address_Type, Addr) == BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Device already bounded\r\n");
    paired = TRUE;
  }
#else
  paired = TRUE;
#endif /* STBOX1_BLE_SECURE_CONNECTION */

  /* Stop the TIM Base generation in interrupt mode for Led Blinking*/
  if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Stopping Error */
    Error_Handler();
  }

  /* Just in order to be sure to switch off the User Led */
  LedOffTargetPlatform();

  HAL_Delay(100);
}

/**
  * @brief  This function is called when there is a Pairing Complete event.
  * @param  uint8_t PairingStatus
  * @retval None
  */
static void PairingCompletedFunction(uint8_t PairingStatus)
{
  /* Pairing Status:
   * 0x00 -> "Success",
   * 0x01 -> "Timeout",
   * 0x02 -> "Pairing Failed",
   * 0x03 -> "Encryption failed, LTK missing on local device",
   * 0x04 -> "Encryption failed, LTK missing on peer device",
   * 0x05 -> "Encryption not supported by remote device"
   */
  if (PairingStatus == 0x00 /* Success */)
  {
    paired = TRUE;
  }
  else
  {
    paired = FALSE;
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
  * @brief  Callback Function for answering to ClearDB Command
  * @param  None
  * @retval None
  */
static void ExtConfigClearDBCommandCallback(void)
{
  NeedToClearSecureDB = 1;
}

/**
  * @brief  Callback Function for answering to Power Off Command
  * @param  None
  * @retval None
  */
static void ExtConfigPowerOffCommandCallback(void)
{
  HAL_Delay(5000);
  PowerButtonPressed = 1;
}

#ifdef STBOX1_RESTART_DFU
/**
  * @brief  Callback Function for answering to Reboot on DFU Mode Command
  * @param  None
  * @retval None
  */
static void ExtConfigRebootOnDFUModeCommandCallback(void)
{
  HAL_Delay(5000);
  DFU_Var = DFU_MAGIC_NUM;
  HAL_NVIC_SystemReset();
}
#endif /* STBOX1_RESTART_DFU */

/**
  * @brief  Callback Function for answering to Info command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
static void ExtConfigInfoCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer, "\r\nSTMicroelectronics %s:\n"
          "\tVersion %c.%c.%c\n"
          "\tSTM32L4R9ZI-SensorTile.box board"
          "\n\t(HAL %ld.%ld.%ld_%ld)\n"
          "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
          " (IAR)\n"
#elif defined (__ARMCC_VERSION)
          " (KEIL)\n"
#elif defined (__GNUC__)
          " (STM32CubeIDE)\n"
#endif /* IDE */
          "\tCurrent Bank =%ld\n",
          STBOX1_PACKAGENAME,
          STBOX1_VERSION_MAJOR, STBOX1_VERSION_MINOR, STBOX1_VERSION_PATCH,
          HAL_GetHalVersion() >> 24,
          (HAL_GetHalVersion() >> 16) & 0xFF,
          (HAL_GetHalVersion() >> 8) & 0xFF,
          HAL_GetHalVersion()      & 0xFF,
          __DATE__, __TIME__,
          CurrentActiveBank);
}

/**
  * @brief  Callback Function for answering to Help command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
static void ExtConfigHelpCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer, "Help Message.....");
}

/**
  * @brief  Callback Function for answering to VersionFw command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
static void ExtConfigVersionFwCommandCallback(uint8_t *Answer)
{
  sprintf((char *)Answer, "%s_%s_%c.%c.%c\r\n",
          "L4R9",
          STBOX1_PACKAGENAME,
          STBOX1_VERSION_MAJOR,
          STBOX1_VERSION_MINOR,
          STBOX1_VERSION_PATCH);
}

/**
  * @brief  Callback Function for answering to ReadBanksFwId command
  * @param  uint8_t *CurBank Number Current Bank
  * @param  uint16_t *FwId1 Bank1 Firmware Id
  * @param  uint16_t *FwId2 Bank2 Firmware Id
  * @retval None
  */
static void ExtConfigReadBanksFwIdCommandCallback(uint8_t *CurBank, uint16_t *FwId1, uint16_t *FwId2)
{
  ReadFlashBanksFwId(FwId1, FwId2);
  *CurBank = CurrentActiveBank;
}

/**
  * @brief  Callback Function for answering to BanksSwap command
  * @param  None
  * @retval None
  */
static void ExtConfigBanksSwapCommandCallback(void)
{
  uint16_t FwId1;
  uint16_t FwId2;

  ReadFlashBanksFwId(&FwId1, &FwId2);
  if (FwId2 != OTA_OTA_FW_ID_NOT_VALID)
  {
    PRINT_DBG("Swapping to Bank%d\n", (CurrentActiveBank == 1) ? 0 : 1);
    PRINT_DBG("%s will restart after the disconnection\r\n", STBOX1_PACKAGENAME);
    HAL_Delay(1000);
    NeedToSwapBanks = 1;
  }
  else
  {
    PRINT_DBG("Not Valid fw on Bank%d\n\tCommand Rejected\n", (CurrentActiveBank == 1) ? 0 : 1);
    PRINT_DBG("\tLoad a Firmware on Bank%d\n", (CurrentActiveBank == 1) ? 0 : 1);
  }
}
