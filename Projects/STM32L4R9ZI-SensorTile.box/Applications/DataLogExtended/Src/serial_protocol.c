/**
  ******************************************************************************
  * @file    DataLogExtended\Src\serial_protocol.c
  * @brief   This file implements some utilities for the serial protocol
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

/* Standard include ----------------------------------------------------------*/
#include "serial_protocol.h"

/** @addtogroup SENSORTILE_Examples
  * @{
  */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Byte stuffing process for one byte
  * @param  Dest destination
  * @param  Source source
  * @retval Total number of bytes processed
  */
int32_t ByteStuffCopyByte(uint8_t *Dest, uint8_t Source)
{
  switch (Source)
  {
    case TM_SG_EOF:
      Dest[0] = TM_SG_BS;
      Dest[1] = TM_SG_BS_EOF;
      return 2;
    case TM_SG_BS:
      Dest[0] = TM_SG_BS;
      Dest[1] = TM_SG_BS;
      return 2;
    default:
      Dest[0] = Source;
      return 1;
  }
}

/**
  * @brief  Byte stuffing process for a Msg
  * @param  Dest destination
  * @param  Source source
  * @retval Total number of bytes processed
  */
int32_t ByteStuffCopy(uint8_t *Dest, TMsg *Source)
{
  int32_t i;
  int32_t Count;

  Count = 0;
  for (i = 0; i < Source->Len; i++)
  {
    Count += ByteStuffCopyByte(&Dest[Count], Source->Data[i]);
  }
  Dest[Count] = TM_SG_EOF;
  Count++;
  return Count;
}

/**
  * @brief  Reverse Byte stuffing process for one byte
  * @param  Source source
  * @param  Dest destination
  * @retval Number of input bytes processed (1 or 2) or 0 for invalid sequence
  */
int32_t ReverseByteStuffCopyByte(uint8_t *Source, uint8_t *Dest)
{
  if (Source[0] == TM_SG_BS)
  {
    if (Source[1] == TM_SG_BS)
    {
      *Dest = TM_SG_BS;
      return 2;
    }
    if (Source[1] == TM_SG_BS_EOF)
    {
      *Dest = TM_SG_EOF;
      return 2;
    }
    return 0; /* invalide sequence */
  }
  else
  {
    *Dest = Source[0];
    return 1;
  }
}

/**
  * @brief  Reverse Byte stuffing process for two input data
  * @param  Source0 input data
  * @param  Source1 input data
  * @param  Dest the destination data
  * @retval Number of input bytes processed (1 or 2) or 0 for invalid sequence
  */
int32_t ReverseByteStuffCopyByte2(uint8_t Source0, uint8_t Source1, uint8_t *Dest)
{
  if (Source0 == TM_SG_BS)
  {
    if (Source1 == TM_SG_BS)
    {
      *Dest = TM_SG_BS;
      return 2;
    }
    if (Source1 == TM_SG_BS_EOF)
    {
      *Dest = TM_SG_EOF;
      return 2;
    }
    return 0; /* invalid sequence */
  }
  else
  {
    *Dest = Source0;
    return 1;
  }
}

/**
  * @brief  Reverse Byte stuffing process for a Msg
  * @param  Dest destination
  * @param  Source source
  * @retval 1 if the operation succeeds, 0 if an error occurs
  */
int32_t ReverseByteStuffCopy(TMsg *Dest, uint8_t *Source)
{
  int32_t Count = 0;
  int32_t State = 0;

  while ((*Source) != TM_SG_EOF)
  {
    if (State == 0)
    {
      if ((*Source) == TM_SG_BS)
      {
        State = 1;
      }
      else
      {
        Dest->Data[Count] = *Source;
        Count++;
      }
    }
    else
    {
      if ((*Source) == TM_SG_BS)
      {
        Dest->Data[Count] = TM_SG_BS;
        Count++;
      }
      else
      {
        if ((*Source) == TM_SG_BS_EOF)
        {
          Dest->Data[Count] = TM_SG_EOF;
          Count++;
        }
        else
        {
          return 0; /* invalid sequence */
        }
      }
      State = 0;
    }
    Source++;
  }
  if (State != 0) { return 0; }
  Dest->Len = Count;
  return 1;
}

/**
  * @brief  Compute and add checksum
  * @param  Msg pointer to the message
  * @retval None
  */
void CHK_ComputeAndAdd(TMsg *Msg)
{
  uint8_t CHK = 0;
  int32_t i;

  for (i = 0; i < Msg->Len; i++)
  {
    CHK -= Msg->Data[i];
  }
  Msg->Data[i] = CHK;
  Msg->Len++;
}

/**
  * @brief  Compute and remove checksum
  * @param  Msg pointer to the message
  * @retval A number different from 0 if the operation succeeds, 0 if an error occurs
  */
int32_t CHK_CheckAndRemove(TMsg *Msg)
{
  uint8_t CHK = 0;
  int32_t i;

  for (i = 0; i < Msg->Len; i++)
  {
    CHK += Msg->Data[i];
  }
  Msg->Len--;
  return (CHK == 0);
}

/**
  * @brief  Build an array from the uint32_t (LSB first)
  * @param  Dest destination
  * @param  Source source
  * @param  Len number of bytes
  * @retval None
  */
void Serialize(uint8_t *Dest, uint32_t Source, uint32_t Len)
{
  int32_t i;
  for (i = 0; i < Len; i++)
  {
    Dest[i] = Source & 0xFF;
    Source >>= 8;
  }
}

/**
  * @brief  Unbuild a Number from an array (LSB first)
  * @param  Source source
  * @param  Len number of bytes
  * @retval Rebuild unsigned int variable
  */
uint32_t Deserialize(uint8_t *Source, uint32_t Len)
{
  uint32_t app;
  app = Source[--Len];
  while (Len > 0)
  {
    app <<= 8;
    app += Source[--Len];
  }
  return app;
}

/**
  * @brief  Build an array from the uint32_t (LSB first)
  * @param  Dest destination
  * @param  Source source
  * @param  Len number of bytes
  * @retval None
  */
void Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len)
{
  int32_t i;
  for (i = 0; i < Len; i++)
  {
    Dest[i] = Source & 0xFF;
    Source >>= 8;
  }
}

/**
  * @brief  Unbuild a Number from an array (LSB first)
  * @param  Source source
  * @param  Len number of bytes
  * @retval Rebuild signed int variable
  */
int32_t Deserialize_s32(uint8_t *Source, uint32_t Len)
{
  int32_t app;
  app = Source[--Len];
  while (Len > 0)
  {
    app <<= 8;
    app += Source[--Len];
  }
  return app;
}

/**
  * @}
  */

/**
  * @}
  */

