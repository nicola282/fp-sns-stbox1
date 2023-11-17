/**
  ******************************************************************************
  * @file    DataLogExtended\Inc\vcom.h
  * @brief   Header for vcom.c
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VCOM__H
#define VCOM__H

/* Includes ------------------------------------------------------------------*/
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "serial_protocol.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Serial message status values enumeration
  */
typedef enum
{
  OK,              /* Message read OK. */
  NO_DATA,         /* No data in message buffer. */
  UNEXP_END_ERR,   /* Unexpected end of message found. */
  WRONG_SEQ_ERR,   /* Wrong sequence found. */
  WRONG_CHECK_ERR, /* Wrong message checksum. */
  MSG_LEN_ERR,     /* Message length exceeded */
  UNKNOWN_ERROR    /* Unknown error. */
} vcom_msg_status_t;

/* Exported defines ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int32_t VCOM_init(void);
vcom_msg_status_t VCOM_receive_MSG(TMsg *Msg, uint32_t len_max);
vcom_msg_status_t VCOM_send_MSG(TMsg *Msg);
uint32_t VCOM_read(char *buffer, uint32_t len_max);
uint32_t VCOM_write(char *buffer, uint32_t len_max);

#endif /* VCOM__H */

