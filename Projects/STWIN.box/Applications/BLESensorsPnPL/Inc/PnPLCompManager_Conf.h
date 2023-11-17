/**
  ******************************************************************************
  * @file    PnPLCompManager_Conf.h
  * @author  SRA
  * @brief   PnPL Components Manager configuration template file.
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

/**
  ******************************************************************************
  * This file has been auto generated from the following Device Template Model:
  * dtmi:appconfig:steval_stwinbx1:FP_SNS_STBOX1_BLESensorPnPL;1
  *
  * Created by: DTDL2PnPL_cGen version 1.1.0
  *
  * WARNING! All changes made to this file will be lost if this is regenerated
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PNPL_COMP_MANAGER_CONF_H__
#define __PNPL_COMP_MANAGER_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "STBOX1_config.h"  
#include "BLE_Manager.h"
  
#define BOARD_ID BLE_MANAGER_STEVAL_STWINBX1_PLATFORM
#define FW_ID STBOX1_BLUEST_SDK_FW_ID
  
/****************** Malloc/Free **************************/
#define pnpl_malloc malloc
#define pnpl_free   free
  
#ifdef __cplusplus
}
#endif

#endif /* __PNPL_COMP_MANAGER_CONF_H__*/
