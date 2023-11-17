/**
  ******************************************************************************
  * @file    BLELowPowerRToS\Inc\main.h 
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Header for BLELowPowerRToS\Src\main.c module
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
#include "HWAdvanceFeatures.h"
#include "BLE_Manager.h"

/* Exported Types ------------------------------------------------------- */

typedef enum
{
  SET_CONNECTABLE     = 0x00,
  CONF_NOTIFY         = 0x01,
  PROCESS_EVENT       = 0x02,
  ACC                 = 0x03,
  ACC_STEP            = 0x04,
  ENV                 = 0x05,
  MOTION              = 0x06,
  AUDIO_LEV           = 0x07,
  TERM_STDOUT         = 0x08,
  TERM_STDERR         = 0x09,
  BATTERY_INFO        = 0x0A,
  BATTERY_PLUG        = 0x0B,
  NUMBER_OF_MSG_TYPE
}msgType_t;

typedef struct
{
  int32_t  press;
  uint16_t hum;
  int16_t  temp;
} envData_t;

typedef struct
{
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyr;
  BSP_MOTION_SENSOR_Axes_t mag;
} motionData_t;

typedef struct
{
  uint32_t feature;
  uint8_t  command;
  uint8_t  data;
} conf_t;

typedef struct
{
  uint8_t  length;
  uint8_t  data[DEFAULT_MAX_CONFIG_CHAR_LEN];
} term_data_t;

typedef struct
{
  uint32_t soc_status;
  uint32_t voltage;
  uint32_t level;
} battery_data_t;

#if defined (__ARMCC_VERSION)
  #pragma anon_unions
#endif

typedef struct 
{
  msgType_t type;
  union
  {
    envData_t      env     ;
    conf_t         conf    ;
    AccEventType   acc     ;
    uint16_t       stepCnt ;
    uint16_t       dB_Value[1];
    motionData_t   motion  ;
    term_data_t    term    ;
    battery_data_t batteryInfo;
  };
}msgData_t;

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *TempToSend);

/* Blinking Led functions */
extern void LedBlinkStart(void);
extern void LedBlinkStop(void);

extern unsigned char ReCallCalibrationFromMemory(uint16_t dataSize, uint32_t *data);
extern unsigned char SaveCalibrationToMemory(uint16_t dataSize, uint32_t *data);
extern int SendMsgToHost(msgData_t *mailPtr);
extern int startProc(msgType_t Type,uint32_t period);
extern int stopProc(msgType_t Type);

extern void SystemClock_Config(void);

/* Exported defines and variables  ------------------------------------------------------- */
extern volatile int PowerButtonPressed;

#define RTC_CLOCK_SOURCE_LSI
/*#define RTC_CLOCK_SOURCE_LSE*/

#endif /* __MAIN_H */

