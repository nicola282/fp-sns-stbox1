/**
  ******************************************************************************
  * @file    BLESensors\Inc\main.h 
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "hci.h"
#include "hci_tl.h"
#include "hci_tl_interface.h"
#include "STBOX1_config.h"
#include "TargetFeatures.h"
#include "BLE_Manager.h"
//#include "sensor_service.h"

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *TempToSend);
extern void ReadInertialData(BSP_MOTION_SENSOR_Axes_t *ACC_Value,BSP_MOTION_SENSOR_Axes_t *GYR_Value,BSP_MOTION_SENSOR_Axes_t *MAG_Value);

/* Exported defines and variables  ------------------------------------------------------- */
extern volatile int PowerButtonPressed;

#endif /* __MAIN_H */
