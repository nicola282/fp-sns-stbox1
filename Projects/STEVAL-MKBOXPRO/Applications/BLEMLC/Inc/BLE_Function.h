/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BLEMLC\Inc\BLE_Function.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @brief   BLE_Function header File
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BLE_FUNCTION_H_
#define _BLE_FUNCTION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/**
* User can added here the header file for the selected BLE features.
* For example:
* #include "BLE_Environmental.h"
* #include "BLE_Inertial.h"
*/

#include "BLE_Inertial.h"
#include "BLE_FiniteStateMachine.h"
#include "BLE_MachineLearningCore.h"
#include "BLE_ActivityRecognition.h"
#include "SensorTileBoxPro_conf.h"

/* Exported Defines --------------------------------------------------------*/

/* Firmware Package Name */
#define STBOX1_FW_PACKAGENAME      "BLEMCL"

/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1)

/* Activity Recognition */
#define W2ST_CONNECT_AR_EVENT   (1<<1)

/* Machine Learning Core */
#define W2ST_CONNECT_MLC        (1<<2)

/* Finite State Machine */
#define W2ST_CONNECT_FSM        (1<<3)

/* Exported Variables ------------------------------------------------------- */
extern FinishGood_TypeDef FinishGood;
extern uint32_t ConnectionBleStatus;
extern volatile uint8_t  connected;
extern volatile uint32_t RebootBoard;
extern volatile uint32_t SwapBanks;

/* Private functions ---------------------------------------------------------*/
FinishGood_TypeDef BSP_CheckFinishGood(void);
void ReadRequestActRec(BLE_AR_output_t *ActCode, BLE_AR_algoIdx_t *Algorithm);
void ReadMachineLearningCore(uint8_t *mlc_out, uint8_t *mlc_status_mainpage);
void ReadFiniteStateMachine(uint8_t *fsm_out, uint8_t *fsm_status_a_mainpage,uint8_t *fsm_status_b_mainpage);
void ExtConfigBanksSwapCommandCallback(void);

/* Exported functions ------------------------------------------------------- */
extern void BLE_InitCustomService(void);
extern void BLE_SetCustomAdvertiseData(uint8_t *manuf_data);
extern void EnableDisableDualBoot(void);

/* Exported macro ------------------------------------------------------------*/
#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#ifdef __cplusplus
}
#endif

#endif /* _BLE_FUNCTION_H_ */

