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

#include "uzlib.h"

/* Exported Variables --------------------------------------------------------*/
volatile uint8_t connected = FALSE;

uint8_t BatteryTimerEnabled = 0;

uint8_t AR_Enabled = 0;
uint8_t MLC_Enabled = 0;
uint8_t FSM_Enabled = 0;

/* Private variables ------------------------------------------------------------*/
static uint8_t InertialTimerEnabled = 0;

/* This array Maps the output of .ucf file to the activities knowed by ST BLE Sensor application:
 *  - 0 : Stationary
 *  - 1 : Walking
 *  - 4 : Jogging
 *  - 8 : Biking
 *  - 12: Driving
 */
BLE_AR_output_t BLE_MappingToHAR_ouput_t[13] =
{
  BLE_AR_STATIONARY, /* 0 */
  BLE_AR_WALKING,    /* 1 */
  BLE_AR_ERROR,      /* 2 */
  BLE_AR_ERROR,      /* 3 */
  BLE_AR_JOGGING,    /* 4 */
  BLE_AR_ERROR,      /* 5 */
  BLE_AR_ERROR,      /* 6 */
  BLE_AR_ERROR,      /* 7 */
  BLE_AR_BIKING,     /* 8 */
  BLE_AR_ERROR,      /* 9 */
  BLE_AR_ERROR,      /* 10 */
  BLE_AR_ERROR,      /* 11 */
  BLE_AR_DRIVING     /* 12 */
};

/* Private functions ---------------------------------------------------------*/

/* Write request function */
static uint32_t DebugConsoleParsing(uint8_t *att_data, uint8_t data_length);

/* Read request function */
void ActivityRecognitionReadRequest(BLE_AR_output_t *ActivityCode, BLE_AR_algoIdx_t *Algorithm);
void MachineLearningCoreReadRequest(uint8_t *mlc_out, uint8_t *mlc_status_mainpage);
void FiniteStateMachineReadRequest(uint8_t *fsm_out, uint8_t *fsm_status_a_mainpage, uint8_t *fsm_status_b_mainpage);


/* Notify request function */
static void NotifyEventBattery(BLE_NotifyEvent_t Event);
static void NotifyEventInertial(BLE_NotifyEvent_t Event);
static void NotifyEventActRec(BLE_NotifyEvent_t Event);
static void NotifyEventMachineLearningCore(BLE_NotifyEvent_t Event);
static void NotifyEventFiniteStateMachine(BLE_NotifyEvent_t Event);

static void DisconnectionCompletedFunction(void);
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6]);

static void TIM1_CHANNEL_2_StartStop(BLE_NotifyEvent_t Event);
static void TIM1_CHANNEL_4_StartStop(BLE_NotifyEvent_t Event);

static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length, uint32_t *DecodingOneStream,
                                           int32_t *StreamLength, uint8_t **CompressedData);
static uint32_t GetUncompressedSize(uint8_t *compressed, uint32_t size);
static uint8_t *Decompress(uint8_t *compressed, uint32_t size, uint32_t *UnComSize);
static void FromHexToUCF(const char *In, uint32_t len, ucf_line_t *UCFProgram);

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
  sprintf(BLE_StackValue.BoardName, "%s%c%c%c", "MLC_",
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
  /* Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = &DebugConsoleParsing;

  /* Custom Function for Connection Completed */
  CustomConnectionCompleted = &ConnectionCompletedFunction;

  /* Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = &DisconnectionCompletedFunction;

  /* Custom Function for Activity Recognition Read Request */
  CustomReadRequestActRec = &ActivityRecognitionReadRequest;

  /* Custom Function for Machine Learning Core Read Request */
  CustomReadMachineLearningCore = &MachineLearningCoreReadRequest;

  /* Custom Function for Finite State Machine Read Request */
  CustomReadFiniteStateMachine = &FiniteStateMachineReadRequest;

  /**************************************************
    * Callback functions to manage the notify events *
    **************************************************/
  CustomNotifyEventBattery =             &NotifyEventBattery;
  CustomNotifyEventInertial =            &NotifyEventInertial;
  CustomNotifyEventActRec =              &NotifyEventActRec;
  CustomNotifyEventFiniteStateMachine =  &NotifyEventFiniteStateMachine;
  CustomNotifyEventMachineLearningCore = &NotifyEventMachineLearningCore;

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

  /* Characteristc allocation for inertial features */
  /* BLE_InitInertialService(AccEnable,GyroEnable,MagEnabled) */
  if ((BleManagerAddChar(BLE_InitInertialService(1, 1, 0))) == 0)
  {
    BLE_MANAGER_PRINTF("Error adding Inertial Service\r\n");
  }

  /* Characteristc allocation for activity recognition features */
  if ((BleManagerAddChar(BLE_InitActRecService())) == 0)
  {
    BLE_MANAGER_PRINTF("Error adding Activity Recognition\r\n");
  }

  /* Characteristic allocation for FSM 16 Registers */
  if ((BleManagerAddChar(BLE_InitFiniteStateMachineService(BLE_FSM_16_REG))) == 0)
  {
    BLE_MANAGER_PRINTF("Error adding FSM Service\r\n");
  }


  /* Characteristic allocation for MLC 8 Registers */
  if ((BleManagerAddChar(BLE_InitMachineLearningCoreService(BLE_MLC_8_REG))) == 0)
  {
    BLE_MANAGER_PRINTF("Error adding MLC Service\r\n");
  }


  /* Characteristc allocation for battery features */
  if ((BleManagerAddChar(BLE_InitBatteryService())) == 0)
  {
    BLE_MANAGER_PRINTF("Error adding Battery Service\r\n");
  }


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
  /* Custom advertize data setting for the inertial features */
  BLE_SetInertialAdvertizeData(manuf_data);

  /* Custom advertize data setting for the battery features */
  BLE_SetBatteryAdvertizeData(manuf_data);

  /* Custom advertize data setting for the activity recognition features */
  BLE_SetActRecAdvertizeData(manuf_data);

#else /* BLE_MANAGER_SDKV2 */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1] = 0x11;
#endif /* BLE_MANAGER_SDKV2 */
}

/**
  * @brief This function Decompresses one buffer
  * @param uint8_t *compressed buffer
  * @param uint32_t size dimension of the compressed buffer
  * @param uint32_t *UnComSize Size of uncompressed buffer
  * @retval uint8_t *Pointer to uncompressed buffer
  */
static uint8_t *Decompress(uint8_t *compressed, uint32_t size, uint32_t *UnComSize)
{
  struct uzlib_uncomp dest_struct;
  uint8_t *uncompressed;
  int32_t res;
  uint32_t un_size;
  unsigned int chunk_len;

#define OUT_CHUNK_SIZE 1

  un_size = GetUncompressedSize(compressed, size);
  *UnComSize = un_size;
  BLE_MANAGER_PRINTF("Uncompressed Size =%ld\r\n", un_size);

  uncompressed = (uint8_t *)calloc(un_size, sizeof(uint8_t));
  if (uncompressed == NULL)
  {
    BLE_MANAGER_PRINTF("Error in memory allocation for decompression\r\n");
    return NULL;
  }

  uzlib_uncompress_init(&dest_struct, NULL, 0);

  dest_struct.source = compressed;
  dest_struct.source_limit = compressed + size - 4;
  dest_struct.source_read_cb = NULL;

  res = uzlib_gzip_parse_header(&dest_struct);

  if (res != TINF_OK)
  {
    BLE_MANAGER_PRINTF("Error in decompressing regConfig\r\n");
    return NULL;
  }

  dest_struct.dest_start = dest_struct.dest = uncompressed;

  chunk_len = 0;

  while (un_size)
  {
    chunk_len = un_size < OUT_CHUNK_SIZE ? un_size : OUT_CHUNK_SIZE;
    dest_struct.dest_limit = dest_struct.dest + chunk_len;
    res = uzlib_uncompress_chksum(&dest_struct);
    un_size -= chunk_len;
    if (res != TINF_OK)
    {
      break;
    }
  }

  if (res != TINF_DONE)
  {
    BLE_MANAGER_PRINTF("Error in decompressing regConfig\r\n");
    return NULL;
  }
  return uncompressed;
}

/**
  * @brief This function return the size of uncompressed buffer
  * @param uint8_t *compressed buffer
  * @param uint32_t size dimension of the compressed buffer
  * @retval uint32_t size of uncompressed buffer
  */
static uint32_t GetUncompressedSize(uint8_t *compressed, uint32_t size)
{
  uint32_t dlen =     compressed[size - 1];
  dlen = (256 * dlen) + compressed[size - 2];
  dlen = (256 * dlen) + compressed[size - 3];
  dlen = (256 * dlen) + compressed[size - 4];
  return dlen + 1;
}

#ifdef STBOX1_ENABLE_PRINTF
/* For searching how many bytes are allocated and if the code release all the memory */
static int32_t malloc_count = 0;
static int32_t malloc_size = 0;
static void *counted_malloc(size_t size)
{
  void *res = malloc(size);
  if (res != NULL)
  {
    malloc_count++;
    malloc_size += size;
  }
  return res;
}

static void counted_free(void *ptr)
{
  if (ptr != NULL)
  {
    malloc_count--;
  }
  free(ptr);
}
#endif /* STBOX1_ENABLE_PRINTF */

/**
  * @brief  This function Converts one string to UCF program
  * @param const char *In Input char string
  * @param uint32_t len length of the input char string
  * @param ucf_line_t *UCFProgram pointer to output UCF program
  * @retval None
  */
static void FromHexToUCF(const char *In, uint32_t len, ucf_line_t *UCFProgram)
{
  uint32_t i;
  uint32_t AH;
  uint32_t AL;
  uint32_t DH;
  uint32_t DL;

  for (i = 0; i < len; i += 4)
  {
    char In1 = *In++;
    char In2 = *In++;
    char In3 = *In++;
    char In4 = *In++;
    AH = (In1 > '9') ? (In1 - 'A' + 10) : (In1 - '0');
    AL = (In2 > '9') ? (In2 - 'A' + 10) : (In2 - '0');
    DH = (In3 > '9') ? (In3 - 'A' + 10) : (In3 - '0');
    DL = (In4 > '9') ? (In4 - 'A' + 10) : (In4 - '0');
    UCFProgram->address = (AH << 4) | AL;
    UCFProgram->data    = (DH << 4) | DL;
    UCFProgram++;
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

  static uint32_t DecodingOneStream = 0;
  static int32_t StreamLength = -1; /* Nothing to Decode */
  static uint8_t *CompressedData = NULL;
  static int32_t PointerToCompressData = 0;
  static uint8_t *DeCompressedData = NULL;

  if (DecodingOneStream)
  {
    /* If we are decoding one stream */
    /* we receive packets of 20 bytes */
    /* BLE_MANAGER_PRINTF("Stream StreamLength=%d packet=%d\r\n",StreamLength,data_length); */
    memcpy(CompressedData + PointerToCompressData, att_data, data_length);
    StreamLength -= data_length;
    PointerToCompressData += data_length;
    /* Return Message */
    Term_Update(att_data, data_length);
    SendBackData = 0;
  }
  else
  {
    /* Received one write from Client on Terminal characteristc */
    SendBackData = DebugConsoleCommandParsing(att_data,
                                              data_length,
                                              &DecodingOneStream,
                                              &StreamLength,
                                              &CompressedData);
  }

  /* Decode the Stream full received */
  if (StreamLength == 0)
  {
    /* Decode the stream */
    uint32_t UnComSize;
    BLE_MANAGER_PRINTF("End of Stream\r\n");
    /* Set Nothing to Receive */
    StreamLength = -1;
    DecodingOneStream = 0;
    /* Decompressed the Data */
    BLE_MANAGER_PRINTF("--- Json Start Decompression ---\r\n");
    DeCompressedData = Decompress(CompressedData, PointerToCompressData, &UnComSize);
    BLE_MANAGER_PRINTF("--- Json End Decompression ---\r\n");
    /* Free Memory For Compressed Data*/
    free(CompressedData);
    CompressedData = NULL;
    PointerToCompressData = 0;

    if (DeCompressedData == NULL)
    {
      /* Return Message */
      BytesToWrite = sprintf((char *)BufferToWrite, "Flow_parse_ko");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      JSON_Value *root_value = NULL;
      BLE_MANAGER_PRINTF("--- Json Start Parsing ---\r\n");
#ifdef STBOX1_ENABLE_PRINTF
      json_set_allocation_functions(counted_malloc, counted_free);
#endif /* STBOX1_ENABLE_PRINTF */
      /* Parse the Decompressed Data */
      root_value = json_parse_string((char *)DeCompressedData);
      /* Free Memory For Decompressed Data*/
      free(DeCompressedData);
      DeCompressedData = NULL;
      if (json_value_get_type(root_value) == JSONArray)
      {
        BLE_MANAGER_PRINTF("root_value==ARRAY\r\n");
      }
      JSON_Array *flows;
      int32_t NumFlows;
      flows = json_value_get_array(root_value);
      BLE_MANAGER_PRINTF("Num flows=%d\r\n", json_array_get_count(flows));
      for (NumFlows = 0; NumFlows < json_array_get_count(flows); NumFlows++)
      {
        JSON_Object *flow;
        int32_t NumElementFlow;
        flow = json_array_get_object(flows, NumFlows);
        BLE_MANAGER_PRINTF("\tNumElementFlow=%d\r\n", json_object_get_count(flow));
        for (NumElementFlow = 0; NumElementFlow < json_object_get_count(flow); NumElementFlow++)
        {
          BLE_MANAGER_PRINTF("\tElementName=%s\r\n", json_object_get_name(flow, NumElementFlow));
          if (!strncmp("sensors", json_object_get_name(flow, NumElementFlow), 7))
          {
            JSON_Array  *SensorsArray;
            int32_t NumSensors;
            SensorsArray = json_object_get_array(flow, "sensors");
            BLE_MANAGER_PRINTF("\t\tNumSensors=%d\r\n", json_array_get_count(SensorsArray));
            for (NumSensors = 0; NumSensors < json_array_get_count(SensorsArray); NumSensors++)
            {
              JSON_Object *Sensor;
              Sensor = json_array_get_object(SensorsArray, NumSensors);
              if (!strncmp("S12", json_object_get_string(Sensor, "id"), 3))
              {
                /* Check if we have one program for MLC */
                const char *regConfig;
                const char *mlcLabels;
                BLE_MANAGER_PRINTF("\t\t\tMLC Sensor ID found\r\n");
                regConfig = json_object_dotget_string(Sensor, "configuration.regConfig");
                if (regConfig != NULL)
                {
                  uint32_t Length = strlen(regConfig);
                  BLE_MANAGER_PRINTF("\t\t\tMLC Reg Config [%ld] found\r\n", Length);
                  /* Allocate the Memory for the CustomUCF Program for MLC */
                  /* Length should be a multiple of 4 */
                  if (Length & 0x3)
                  {
                    /* Error */
                    BLE_MANAGER_PRINTF("Error Reg Config length not multiple of 4\r\n");
                  }
                  else
                  {
                    if (MLCCustomUCFFile != NULL)
                    {
                      /* if there is already one MLCCustomUCFFile...Release the Memory before */
                      free(MLCCustomUCFFile);
                      MLCCustomUCFFile = NULL;
                      MLCCustomUCFFileLength = 0;
                    }
                    MLCCustomUCFFile = (ucf_line_t *)calloc(Length >> 2, sizeof(ucf_line_t));
                    if (MLCCustomUCFFile == NULL)
                    {
                      BLE_MANAGER_PRINTF("Error in memory allocation MLCCustomUCFFile\r\n");
                    }
                    else
                    {
                      MLCCustomUCFFileLength = Length >> 2;
                      FromHexToUCF(regConfig, Length, MLCCustomUCFFile);
                    }
                  }
                }
                mlcLabels = json_object_dotget_string(Sensor, "configuration.mlcLabels");
                if (mlcLabels != NULL)
                {
                  uint32_t Length = strlen(mlcLabels);
                  BLE_MANAGER_PRINTF("\t\t\tMLC Labels [%ld] found\r\n", Length);
                  /* Allocate the Memory for the Custom Labels for MLC */
                  extern char *MLCCustomLabels;
                  if (MLCCustomLabels != NULL)
                  {
                    /* if there is already one MLCCustomLabels...Release the Memory before */
                    free(MLCCustomLabels);
                    MLCCustomLabels = NULL;
                  }
                  MLCCustomLabels = (char *)calloc(Length + 1, sizeof(char));
                  if (MLCCustomLabels == NULL)
                  {
                    BLE_MANAGER_PRINTF("Error in memory allocation MLCCustomLabels\r\n");
                  }
                  else
                  {
                    MLCCustomLabelsLength = Length + 1;
                    memcpy(MLCCustomLabels, mlcLabels, Length);
                    /* Put Termination */
                    MLCCustomLabels[Length] = '\n';
                  }
                }
              }
              else if (!strncmp("S13", json_object_get_string(Sensor, "id"), 3))
              {
                /* Check if we have one program for FSM */
                const char *regConfig;
                const char *fsmLabels;
                BLE_MANAGER_PRINTF("\t\t\tFSM Sensor ID found\r\n");
                regConfig = json_object_dotget_string(Sensor, "configuration.regConfig");
                if (regConfig != NULL)
                {
                  uint32_t Length = strlen(regConfig);
                  BLE_MANAGER_PRINTF("\t\t\tFSM Reg Config [%ld] found\r\n", Length);
                  /* Allocate the Memory for the CustomUCF Program for FSM */
                  /* Length should be a multiple of 4 */
                  if (Length & 0x3)
                  {
                    /* Error */
                    BLE_MANAGER_PRINTF("Error Reg Config length not multiple of 4\r\n");
                  }
                  else
                  {
                    if (FSMCustomUCFFile != NULL)
                    {
                      /* if there is already one FSMCustomUCFFile...Release the Memory before */
                      free(FSMCustomUCFFile);
                      FSMCustomUCFFile = NULL;
                      FSMCustomUCFFileLength = 0;
                    }
                    FSMCustomUCFFile = (ucf_line_t *)calloc(Length >> 2, sizeof(ucf_line_t));
                    if (FSMCustomUCFFile == NULL)
                    {
                      BLE_MANAGER_PRINTF("Error in memory allocation FSMCustomUCFFile\r\n");
                    }
                    else
                    {
                      FSMCustomUCFFileLength = Length >> 2;
                      FromHexToUCF(regConfig, Length, FSMCustomUCFFile);
                    }
                  }
                }
                fsmLabels = json_object_dotget_string(Sensor, "configuration.fsmLabels");
                if (fsmLabels != NULL)
                {
                  uint32_t Length = strlen(fsmLabels);
                  BLE_MANAGER_PRINTF("\t\t\tFSM Labels [%ld] found\r\n", Length);
                  /* Allocate the Memory for the Custom Labels for FSM */
                  extern char *FSMCustomLabels;
                  if (FSMCustomLabels != NULL)
                  {
                    /* if there is already one FSMCustomLabels...Release the Memory before */
                    free(FSMCustomLabels);
                    FSMCustomLabels = NULL;
                  }
                  FSMCustomLabels = (char *)calloc(Length + 1, sizeof(char));
                  if (FSMCustomLabels == NULL)
                  {
                    BLE_MANAGER_PRINTF("Error in memory allocation FSMCustomLabels\r\n");
                  }
                  else
                  {
                    FSMCustomLabelsLength = Length + 1;
                    memcpy(FSMCustomLabels, fsmLabels, Length);
                    /* Put Termination */
                    FSMCustomLabels[Length] = '\n';
                  }
                }
              }
            }
          }
        }
      }
      json_value_free(root_value);
      BLE_MANAGER_PRINTF("%ld Alloc Not Released\r\nTotal Mem Used=%ld\r\n", malloc_count, malloc_size);
      BLE_MANAGER_PRINTF("--- Json End Parsing ---\r\n");
      /* Return Message */
      BytesToWrite = sprintf((char *)BufferToWrite, "Flow_parse_ok");
      Term_Update(BufferToWrite, BytesToWrite);
    }
  }

  return SendBackData;
}

/**
  * @brief  This function makes the parsing of the Debug Console Commands
  * @param  uint8_t *att_data attribute data
  * @param  uint8_t data_length length of the data
  * @retval uint32_t SendBackData true/false
  */
static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length, uint32_t *DecodingOneStream,
                                           int32_t *StreamLength, uint8_t **CompressedData)
{
  uint32_t SendBackData = 1;

  /* Help Command */
  if (!strncmp("help", (char *)(att_data), 4))
  {
    /* Print Legend */
    SendBackData = 0;

    BytesToWrite = sprintf((char *)BufferToWrite,
                           "info\n"
                           "getMLCLabels\n"
                           "delMLCCustom\n"
                           "delFSMCustom\n"
                           "getFSMLabels\n"
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
    PowerOff = 1;
    SendBackData = 0;
  }
  else if (!strncmp("getFSMLabels", (char *)(att_data), 12))
  {
    if (FSMCustomLabels == NULL)
    {
      BytesToWrite = sprintf((char *)BufferToWrite,
                             "<FSM_OUTS1>4D PosRec,0='N/A',16='Portrait Down',32='Portrait Up',64='Landscape Right',128='Landscape Left';\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      uint32_t Counter;
      for (Counter = 0; Counter < FSMCustomLabelsLength; Counter += MaxBleCharStdOutLen)
      {
        uint32_t MinSize;
        MinSize = FSMCustomLabelsLength - Counter;
        MinSize = (MinSize > MaxBleCharStdOutLen) ?  MaxBleCharStdOutLen : MinSize;
        Term_Update((uint8_t *)(FSMCustomLabels + Counter), MinSize);
      }
    }
    SendBackData = 0;
  }
  else if (!strncmp("getMLCLabels", (char *)(att_data), 12))
  {
    if (MLCCustomLabels == NULL)
    {
      BytesToWrite = sprintf((char *)BufferToWrite,
                             "<MLC0_SRC>Activity Recognition,0='Stationary',1='Walking',4='Jogging',8='Biking',12='Driving';\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      uint32_t Counter;
      for (Counter = 0; Counter < MLCCustomLabelsLength; Counter += MaxBleCharStdOutLen)
      {
        uint32_t MinSize;
        MinSize = MLCCustomLabelsLength - Counter;
        MinSize = (MinSize > MaxBleCharStdOutLen) ?  MaxBleCharStdOutLen : MinSize;
        Term_Update((uint8_t *)(MLCCustomLabels + Counter), MinSize);
      }
    }
    SendBackData = 0;
  }
  else if (!strncmp("delMLCCustom", (char *)(att_data), 12))
  {
    if (MLCCustomLabels != NULL)
    {
      free(MLCCustomLabels);
      MLCCustomLabels = NULL;
      MLCCustomLabelsLength = 0;
      BytesToWrite = sprintf((char *)BufferToWrite, "MLCCustomLabels Deleted\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    if (MLCCustomUCFFile != NULL)
    {
      free(MLCCustomUCFFile);
      MLCCustomUCFFile = NULL;
      MLCCustomUCFFileLength = 0;
      BytesToWrite = sprintf((char *)BufferToWrite, "MLCCustomUCFFile Deleted\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    SendBackData = 0;
  }
  else if (!strncmp("delFSMCustom", (char *)(att_data), 12))
  {
    if (FSMCustomLabels != NULL)
    {
      free(FSMCustomLabels);
      FSMCustomLabels = NULL;
      FSMCustomLabelsLength = 0;
      BytesToWrite = sprintf((char *)BufferToWrite, "FSMCustomLabels Deleted\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    if (FSMCustomUCFFile != NULL)
    {
      free(FSMCustomUCFFile);
      FSMCustomUCFFile = NULL;
      FSMCustomUCFFileLength = 0;
      BytesToWrite = sprintf((char *)BufferToWrite, "FSMCustomUCFFile Deleted\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    SendBackData = 0;
  }
  else if (!strncmp("SF", (char *)(att_data), 2))
  {
    uint32_t TimeStamp;
    uint8_t *PointerByte = (uint8_t *) StreamLength;
    PointerByte[0] = att_data[5];
    PointerByte[1] = att_data[4];
    PointerByte[2] = att_data[3];
    PointerByte[3] = att_data[2];

    PointerByte = (uint8_t *) &TimeStamp;
    PointerByte[0] = att_data[9];
    PointerByte[1] = att_data[8];
    PointerByte[2] = att_data[7];
    PointerByte[3] = att_data[6];

    /* Debug Message */
    BLE_MANAGER_PRINTF("SF command Length=%ld, TS=%lu\r\n", *StreamLength, TimeStamp);

    /* Alloc buffer for storing compressed json */
    *CompressedData = (uint8_t *) malloc((*StreamLength) * sizeof(uint8_t));
    if ((*CompressedData) == NULL)
    {
      BLE_MANAGER_PRINTF("Memory Allocation error for CompressedData\r\n");
      /* Return Message */
      BytesToWrite = sprintf((char *)BufferToWrite, "Allocation Error");
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      /* Return Message */
      BytesToWrite = sprintf((char *)BufferToWrite, "Flow_Req_Received");
      Term_Update(BufferToWrite, BytesToWrite);
    }

    SendBackData = 0;
    *DecodingOneStream = 1;
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

/**
  * @brief  Callback Function for Machine Learning Core read request.
  * @param  uint8_t *mlc_out pointers to 8 MLC register
  * @param  uint8_t *mlc_status_mainpage pointer to the MLC status bits from 1 to 8
  * @retval None
  */
void MachineLearningCoreReadRequest(uint8_t *mlc_out, uint8_t *mlc_status_mainpage)
{
  lsm6dsox_all_sources_t  status;

  lsm6dsox_all_sources_get(LSM6DSOX_CONTEX, &status);

  *mlc_status_mainpage = ((status.mlc1)    | (status.mlc2 << 1) | (status.mlc3 << 2) | (status.mlc4 << 3) |
                          (status.mlc1 << 4) | (status.mlc2 << 5) | (status.mlc3 << 6) | (status.mlc4 << 7));

  STBOX1_PRINTF("Read On MLC\r\n");

  lsm6dsox_mlc_out_get(LSM6DSOX_CONTEX, mlc_out);
}

/**
  * @brief  Callback Function for Finite State Machine read request.
  * @param  uint8_t *fsm_out pointers to 16 FSM register
  * @param  uint8_t *fsm_status_a_mainpage pointer to the FSM status bits from 1 to 8
  * @param  uint8_t *fsm_status_b_mainpage pointer to the FSM status bits from 9 to 16
  * @retval None
  */
void FiniteStateMachineReadRequest(uint8_t *fsm_out, uint8_t *fsm_status_a_mainpage, uint8_t *fsm_status_b_mainpage)
{
  lsm6dsox_all_sources_t status;

  lsm6dsox_all_sources_get(LSM6DSOX_CONTEX, &status);

  *fsm_status_a_mainpage = ((status.fsm1)     | (status.fsm2 << 1)  | (status.fsm3 << 2)  | (status.fsm4 << 3)  |
                            (status.fsm5 << 4)  | (status.fsm6 << 5)  | (status.fsm7 << 6)  | (status.fsm8 << 7)  |
                            (status.fsm5 << 8)  | (status.fsm6 << 9)  | (status.fsm7 << 10) | (status.fsm8 << 11) |
                            (status.fsm5 << 12) | (status.fsm6 << 13) | (status.fsm7 << 14) | (status.fsm8 << 15));

  *fsm_status_b_mainpage = ((status.fsm1)     | (status.fsm2 << 1)  | (status.fsm3 << 2)  | (status.fsm4 << 3)  |
                            (status.fsm5 << 4)  | (status.fsm6 << 5)  | (status.fsm7 << 6)  | (status.fsm8 << 7)  |
                            (status.fsm5 << 8)  | (status.fsm6 << 9)  | (status.fsm7 << 10) | (status.fsm8 << 11) |
                            (status.fsm5 << 12) | (status.fsm6 << 13) | (status.fsm7 << 14) | (status.fsm8 << 15));

  STBOX1_PRINTF("Read On FSM\r\n");

  lsm6dsox_fsm_out_get(LSM6DSOX_CONTEX, (lsm6dsox_fsm_out_t *)fsm_out);
}

/**
  * @brief  Callback Function for Activity Recognition read request.
  * @param  BLE_AR_output_t ActivityCode Activity Recognized
  * @param  BLE_AR_algoIdx_t Algorithm Code
  * @retval None
  */
void ActivityRecognitionReadRequest(BLE_AR_output_t *ActivityCode, BLE_AR_algoIdx_t *Algorithm)
{
  lsm6dsox_all_sources_t  status;
  uint8_t mlc_out[8];

  BLE_MANAGER_PRINTF("Read On AR\r\n");

  lsm6dsox_all_sources_get(LSM6DSOX_CONTEX, &status);
  lsm6dsox_mlc_out_get(LSM6DSOX_CONTEX, mlc_out);

  *ActivityCode = BLE_MappingToHAR_ouput_t[mlc_out[0]];
  *Algorithm = HAR_MLC_LSM6DSOX_ID;
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
    TIM1_CHANNEL_2_StartStop(Event);
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

/**
  * @brief  Callback Function for Un/Subscription Activity Recognition Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventActRec(BLE_NotifyEvent_t Event)
{
  if (Event == BLE_NOTIFY_SUB)
  {
    AR_Enabled = 1;

    /* Start the Activity Recognition of LSM6DSOX MLC */
    Init_MEMS_MLC(0 /* Use Default UCF program -> Activity Recognition */);

    BLE_MANAGER_PRINTF("Start MLC\r\n");

    BLE_ActRecUpdate(BLE_AR_NOACTIVITY, HAR_MLC_LSM6DSOX_ID);

  }
  else if (Event == BLE_NOTIFY_UNSUB)
  {
    AR_Enabled = 0;

    /* Stop the LSM6DSOX MLC */
    DeInit_MEMS();

    BLE_MANAGER_PRINTF("Stop MLC\r\n");
  }
}

/**
  * @brief  Callback Function for Un/Subscription MLC Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventMachineLearningCore(BLE_NotifyEvent_t Event)
{
  if (Event == BLE_NOTIFY_SUB)
  {
    MLC_Enabled = 1;

    /* Start the LSM6DSOX MLC */
    /* Use Custom UCF if it's available */
    Init_MEMS_MLC(1);

    BLE_MANAGER_PRINTF("Start MLC\r\n");
  }
  else if (Event == BLE_NOTIFY_UNSUB)
  {
    MLC_Enabled = 0;

    /* Stop the LSM6DSOX MLC */
    DeInit_MEMS();

    BLE_MANAGER_PRINTF("Stop MLC\r\n");
  }
}

/**
  * @brief  Callback Function for Un/Subscription FSM Feature
  * @param  BLE_NotifyEvent_t Event Sub/Unsub
  * @retval None
  */
static void NotifyEventFiniteStateMachine(BLE_NotifyEvent_t Event)
{
  if (Event == BLE_NOTIFY_SUB)
  {
    FSM_Enabled = 1;

    /* Start the LSM6DSOX FSM */
    /* Use Custom UCF if it's available */
    Init_MEMS_FSM(0);

    BLE_MANAGER_PRINTF("Start FSM\r\n");
  }
  else if (Event == BLE_NOTIFY_UNSUB)
  {

    FSM_Enabled = 0;

    /* Stop the LSM6DSOX FSM */
    DeInit_MEMS();
    BLE_MANAGER_PRINTF("Stop FSM\r\n");
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
static void TIM1_CHANNEL_2_StartStop(BLE_NotifyEvent_t Event)
{
  if ((Event == BLE_NOTIFY_SUB) &&
      (!BatteryTimerEnabled))
  {
    uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
    BSP_BC_CmdSend(BATMS_ON);
    BatteryTimerEnabled = 1;

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + STBOX1_UPDATE_BATTERY));
  }

  if ((Event == BLE_NOTIFY_UNSUB) &&
      (BatteryTimerEnabled))
  {
    BSP_BC_CmdSend(BATMS_OFF);

    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    BatteryTimerEnabled = 0;
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

    /* Init the LSM6DSOX Acc/Gyro */
    Init_MEMS();

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

    /* Stop the LSM6DSOX Acc/Gyro */
    DeInit_MEMS();

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
#endif
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

