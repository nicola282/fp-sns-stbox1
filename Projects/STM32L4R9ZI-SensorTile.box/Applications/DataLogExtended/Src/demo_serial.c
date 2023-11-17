/**
  ******************************************************************************
  * @file    DataLogExtended\Src\DemoSerial.c
  * @brief   Handle the Serial Protocol
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
#include "demo_serial.h"
#include "main.h"
#include "vcom.h"
#include "sensor_commands.h"

/** @addtogroup SENSORTILE_Examples
  * @{
  */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
volatile uint8_t DataLoggerActive;

/* Private variables ---------------------------------------------------------*/
static uint8_t PresentationString[] = {"MEMS shield demo,101,7.1.0,0.0.0,MKSBOX1V1"};
static volatile uint8_t DataStreamingDest = 1;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Build the reply header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_REPLY_ADD;
}

/**
  * @brief  Build the nack header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_NACK;
}

/**
  * @brief  Initialize the streaming header
  * @param  Msg the pointer to the header to be initialized
  * @retval None
  */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_START_DATA_STREAMING;
  Msg->Len = 3;
}

/**
  * @brief  Initialize the streaming message
  * @param  Msg the pointer to the message to be initialized
  * @retval None
  */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint32_t i;

  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_START_DATA_STREAMING;

  for (i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }
  Msg->Len = 3;
}

/**
  * @brief  Handle a message
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int32_t HandleMSG(TMsg *Msg)
/*  DestAddr | SouceAddr | CMD | SUBCMD | PAYLOAD
        1          1        1       1        N    */
{
  uint32_t i;
  int32_t ret = 1;

  if (Msg->Len < 2U)
  {
    return 0;
  }

  if (Msg->Data[0] != DEV_ADDR)
  {
    return 0;
  }

  switch (Msg->Data[2])   /* CMD */
  {
    case CMD_PING:
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      VCOM_send_MSG(Msg);
      break;

    case CMD_ENTER_DFU_MODE:
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      break;

    case CMD_READ_PRES_STRING:
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      i = 0;
      while (i < (sizeof(PresentationString) - 1U))
      {
        Msg->Data[3U + i] = PresentationString[i];
        i++;
      }
      Msg->Len = 3U + i;
      VCOM_send_MSG(Msg);
      break;

    case CMD_CHECK_MODE_SUPPORT:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Serialize_s32(&Msg->Data[3], DATALOG_EXT_MODE, 4);
      Msg->Len = 3 + 4;
      VCOM_send_MSG(Msg);
      break;

    case CMD_SENSOR:
      /* Check if the command lenght is at least 5 bytes */
      if (Msg->Len < 5U)
      {
        return 0;
      }
      Handle_Sensor_command(Msg);
      break;

    case CMD_START_DATA_STREAMING:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      SensorsEnabled = Deserialize(&Msg->Data[3], 4);
      DataLoggerActive = 1;
      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      VCOM_send_MSG(Msg);
      break;

    case CMD_STOP_DATA_STREAMING:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      SensorsEnabled = 0;
      DataLoggerActive = 0;
      BUILD_REPLY_HEADER(Msg);
      VCOM_send_MSG(Msg);
      break;

    case CMD_SET_DATE_TIME:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_TimeRegulate(Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      VCOM_send_MSG(Msg);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
  * @}
  */

/**
  * @}
  */

