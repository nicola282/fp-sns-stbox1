/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    NFC_FTM\Inc\st25ftm_process.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V2.0.0
  * @date    10-Jun-2024
  * @brief   FTM Process APIs
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
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __ST25FTM_PROCESS_H__
#define __ST25FTM_PROCESS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ----------------------------------------------------------- */

/* Exported constants ------------------------------------------------------- */

/* Exported defines --------------------------------------------------------- */

/* Exporte variables -------------------------------------------------------- */

/* Exported functions ------------------------------------------------------- */
extern void FTMManagementInit( void );
extern void FTMManagement(void);
extern void FTMManagementDeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __ST25FTM_PROCESS_H__*/

