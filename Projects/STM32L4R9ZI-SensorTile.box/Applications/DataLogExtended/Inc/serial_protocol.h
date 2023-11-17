/**
  ******************************************************************************
  * @file    DataLogExtended\Inc\serial_protocol.h
  * @brief   Header for serial_protocol.c
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

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define TM_SG_EOF                0xF0
#define TM_SG_BS                 0xF1
#define TM_SG_BS_EOF             0xF2

#ifdef USE_USB_OTG_HS
#define TM_SG_MAX_LEN             512
#else /* USE_USB_OTG_HS */
#define TM_SG_MAX_LEN             256
#endif /* USE_USB_OTG_HS */

/* _NOTE_: USB buffer size is set in 'usbd_cdc_interface.c' and should be
   at least of size: (2 * TM_SG_MAX_LEN)
*/

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Serial message structure definition
  */
typedef struct
{
  uint32_t Len;
  uint8_t Data[TM_SG_MAX_LEN];
} TMsg;

/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
int32_t ByteStuffCopyByte(uint8_t *Dest, uint8_t Source);
int32_t ReverseByteStuffCopyByte2(uint8_t Source0, uint8_t Source1, uint8_t *Dest);
int32_t ByteStuffCopy(uint8_t *Dest, TMsg *Source);
int32_t ReverseByteStuffCopyByte(uint8_t *Source, uint8_t *Dest);
int32_t ReverseByteStuffCopy(TMsg *Dest, uint8_t *Source);
void CHK_ComputeAndAdd(TMsg *Msg);
int32_t CHK_CheckAndRemove(TMsg *Msg);
uint32_t Deserialize(uint8_t *Source, uint32_t Len);
int32_t Deserialize_s32(uint8_t *Source, uint32_t Len);
void Serialize(uint8_t *Dest, uint32_t Source, uint32_t Len);
void Serialize_s32(uint8_t *Dest, int32_t Source, uint32_t Len);

#endif /* SERIAL_PROTOCOL_H */

