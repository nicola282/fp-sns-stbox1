/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_blemlc.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @brief   Main program body
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

/**
 *
 * @page BLEMLC BLE Firmware example for Machine Learning Core and Finite State Machine
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This firmware package includes Components Device Drivers, Board Support Package
 * and example application for the following STMicroelectronics elements:
 * - STEVAL-MKBOXPRO (SensorTile.box-Pro) evaluation board that contains the following components:
 *   - MEMS sensor devices: STTS22, LPS22DF, LSM6DSV16X, LIS2DU12, LIS2MDL
 *   - Gas Gouge device: STC3115
 *   - Digital Microphone: MP23db01HP
 *   - Dynamic NFC tag: ST25DV04K
 *   - BlueNRG-LP Bluetooth Low Energy System On Chip
 *
 * <b>Example Application</b>
 *
 * This example explains how to use Machine Learning Core and the Finite State Machine
 * This example is compatible with the Firmware Over the Air Update (FoTA)
 *
 *
 * This example must be used with the related ST BLE Sensor Android/iOS application available on Play/itune store
 * (Version 4.18.0 or higher), in order to read the sent information by Bluetooth Low Energy protocol
 *
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "app_blemlc.h"
#include "STBOX1_config.h"
#include "steval_mkboxpro.h"
#include "bluenrg_conf.h"
#include "BLE_Manager.h"
#include "OTA.h"
#include "SensorTileBoxPro_motion_sensors.h"
#include "SensorTileBoxPro_motion_sensors_ex.h"
#include "BLE_Function.h"

/* Default program for Machine Learning Core */
#include "lsm6dsv16x_activity_4classes.h"

/* Default program for Finite State Machine */
#include "lsm6dsv16x_four_d.h"

/* Exported variables --------------------------------------------------------*/
EXTI_HandleTypeDef H_EXTI_INT_LSM6DSV16X = {.Line = INT_LSM6DSV16X_EXTI_LINE};

uint16_t ConnectionHandle = 0;

int32_t CurrentActiveBank = 0;

void *HandleGGComponent;

BLE_AR_output_t ActivityCode=BLE_AR_ERROR;

FinishGood_TypeDef FinishGood;

/* Imported variables --------------------------------------------------------*/
volatile uint32_t hci_event = 0;

/* Private typedef -----------------------------------------------------------*/
/* This array Maps the output of .ucf filt to the activities knowed by ST BLE Sensor application:
 *  - 1 : Stationary
 *  - 4 : Walking
 *  - 8 : Running
 *  - 12: Driving
 */
BLE_AR_output_t MappingToHAR_ouput_t[13]={
  BLE_AR_NOACTIVITY  ,//0
  BLE_AR_STATIONARY  ,//1
  BLE_AR_ERROR       ,//2
  BLE_AR_ERROR       ,//3
  BLE_AR_WALKING     ,//4
  BLE_AR_ERROR       ,//5
  BLE_AR_ERROR       ,//6
  BLE_AR_ERROR       ,//7
  BLE_AR_JOGGING     ,//8
  BLE_AR_ERROR       ,//9
  BLE_AR_ERROR       ,//10
  BLE_AR_ERROR       ,//11
  BLE_AR_DRIVING     //12
};

/* Custom UCF files for MLC and FSM */
ucf_line_t *MLCCustomUCFFile = NULL;
uint32_t MLCCustomUCFFileLength=0;
ucf_line_t *FSMCustomUCFFile = NULL;
uint32_t FSMCustomUCFFileLength=0;

/* Labels for Custom UCF Files form MLC and FSM */
char *MLCCustomLabels=NULL;
char *FSMCustomLabels=NULL;
uint32_t MLCCustomLabelsLength = 0;
uint32_t FSMCustomLabelsLength = 0;

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t UserButtonPressed = 0;
static volatile uint32_t MEMSInterrupt   = 0;

static volatile uint32_t BlinkLed        = 0;
static volatile uint32_t SendAccGyroMag  = 0;

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);

static void InitTimers(void);
static void PrintInfo(void);
static void InitMems(void);
static void MEMSCallback(void);
static void hexti_callback(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
void MX_BLEMLC_Init(void)
{
  /* Set a random seed */
  srand(HAL_GetTick());

  User_Init();

  STBOX1_PRINTF("\033[2J"); /* serial console clear screen */
  STBOX1_PRINTF("\033[H");  /* serial console cursor to home */
  PrintInfo();

  BSP_LED_On(LED_BLUE);

  /* Init Magneto and Interrupt for Acc/Gyro */
  InitMems();

  /* Characteristic allocation for Activity Recognition */
  CustomReadRequestActRec = ReadRequestActRec;

  /* Characteristic allocation for FSM 8 Registers */
  CustomReadFiniteStateMachine = ReadFiniteStateMachine;

  /* Characteristic allocation for MLC 4 Registers */
  CustomReadMachineLearningCore = ReadMachineLearningCore;

  /* Init BLE */
  STBOX1_PRINTF("\r\nInitializing Bluetooth\r\n");
  BluetoothInit();
  {
	uint16_t FwId1,FwId2;

	ReadFlashBanksFwId(&FwId1,&FwId2);
	if(FwId2!=OTA_OTA_FW_ID_NOT_VALID) {
		/* Enable the Banks Swap only if there is a valid fw on second bank */
		CustomExtConfigBanksSwapCommandCallback           = ExtConfigBanksSwapCommandCallback;
	}
  }

  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_GREEN);

  /* Short delay before starting the user application process */
  HAL_Delay(500);
  STBOX1_PRINTF("BLE Stack Initialized & Device Configured\r\n");
}

/*
 * FP-SNS-STBOX1 background task
 */
void MX_BLEMLC_Process(void)
   {
    /* BLE Event */
    {
      hci_event=0;
      hci_user_evt_proc();
    }

    /* Make the device discoverable */
    if(set_connectable)
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TIM_CC_HANDLE);
      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TIM_CC_HANDLE, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        STBOX1_Error_Handler(STBOX1_ERROR_TIMER,__FILE__,__LINE__);
      }
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED));

      setConnectable();
      set_connectable = FALSE;
    }

    if (SendAccGyroMag) {
      BLE_MANAGER_INERTIAL_Axes_t x_axes;
      BLE_MANAGER_INERTIAL_Axes_t g_axes;
      BLE_MANAGER_INERTIAL_Axes_t m_axes;
      SendAccGyroMag=0;

     BSP_MOTION_SENSOR_GetAxes(LSM6DSV16X_0, MOTION_ACCELERO,(BSP_MOTION_SENSOR_Axes_t*)&x_axes);
     BSP_MOTION_SENSOR_GetAxes(LSM6DSV16X_0, MOTION_GYRO,(BSP_MOTION_SENSOR_Axes_t*)&g_axes);
     BSP_MOTION_SENSOR_GetAxes(LIS2MDL_0, MOTION_MAGNETO,(BSP_MOTION_SENSOR_Axes_t*)&m_axes);
     BLE_AccGyroMagUpdate(&x_axes,&g_axes,&m_axes);
    }

    /* Reboot the Board */
    if(RebootBoard) {
      RebootBoard=0;
      HAL_NVIC_SystemReset();
    }

    /* Swap the Flash Banks */
    if(SwapBanks) {
      EnableDisableDualBoot();
      SwapBanks=0;
    }

    /* Handle the user button */
    if(UserButtonPressed) {
      UserButtonPressed=0;
      STBOX1_PRINTF("User Button pressed...\r\n");
    }

    /* Handle the MEMS interrupt */
    if(MEMSInterrupt) {
      MEMSInterrupt=0;
      MEMSCallback();
    }

    /* Blinking the Led */
    if(BlinkLed) {
      BlinkLed = 0;
      BSP_LED_Toggle(LED_GREEN);
    }

    /* Wait next event */
    __WFI();
  }

/**
* @brief  Init Magneto and Interrupt for Acc/Gyro
* @param  None
* @retval None
*/
static void InitMems(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level 5-> BSP_LSM6DSV16X_CS_PIN 7-> BSP_LIS2DU12_CS_PIN*/
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pins : PI5 PI7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

#ifndef ALL_SENSORS_I2C
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
#endif

  /* Magneto */
  if(BSP_MOTION_SENSOR_Init(LIS2MDL_0, MOTION_MAGNETO)==BSP_ERROR_NONE) {
    if(BSP_MOTION_SENSOR_SetOutputDataRate(LIS2MDL_0, MOTION_MAGNETO, LIS2MDL_MAG_ODR)==BSP_ERROR_NONE) {
      if(BSP_MOTION_SENSOR_SetFullScale(LIS2MDL_0, MOTION_MAGNETO, LIS2MDL_MAG_FS)==BSP_ERROR_NONE) {
        STBOX1_PRINTF("LIS2MDL_0 OK\r\n");
      } else {
        STBOX1_PRINTF("Error: LIS2MDL_0 KO\r\n");
      }
    }else {
      STBOX1_PRINTF("Error: LIS2MDL_0 KO\r\n");
    }
  } else {
    STBOX1_PRINTF("Error: LIS2MDL_0 KO\r\n");
  }

  {
    /* Enable interrupts from INT1 LSM6DSV16X  */
    GPIO_InitTypeDef GPIO_InitStruct;

    INT_LSM6DSV16X_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = INT_LSM6DSV16X_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(INT_LSM6DSV16X_GPIO_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_EXTI_GetHandle(&H_EXTI_INT_LSM6DSV16X, INT_LSM6DSV16X_EXTI_LINE);
    HAL_EXTI_RegisterCallback(&H_EXTI_INT_LSM6DSV16X, HAL_EXTI_COMMON_CB_ID, hexti_callback);
    HAL_NVIC_SetPriority(INT_LSM6DSV16X_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(INT_LSM6DSV16X_EXTI_IRQn);

    STBOX1_PRINTF("Enabled LSM6DSV16X INT1 Detection \n\r");
  }
}

/** @brief Initialize the LSM6DSV16X MEMS Sensor
 * @param None
 * @retval None
 */
void InitAcc(void)
{
  /* Acc/Gyro */
  if(BSP_MOTION_SENSOR_Init(LSM6DSV16X_0, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE) {
    if(BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSV16X_0, MOTION_ACCELERO, LSM6DSV16X_ACC_ODR)==BSP_ERROR_NONE) {
      if(BSP_MOTION_SENSOR_SetFullScale(LSM6DSV16X_0, MOTION_ACCELERO, LSM6DSV16X_ACC_FS)==BSP_ERROR_NONE) {
        if(BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSV16X_0, MOTION_GYRO, LSM6DSV16X_GYRO_ODR)==BSP_ERROR_NONE) {
          if(BSP_MOTION_SENSOR_SetFullScale(LSM6DSV16X_0, MOTION_GYRO, LSM6DSV16X_GYRO_FS)==BSP_ERROR_NONE) {
            STBOX1_PRINTF("LSM6DSV16X_0 OK\r\n");
          } else {
            STBOX1_PRINTF("Error: LSM6DSV16X_0 KO\r\n");
          }
        } else {
          STBOX1_PRINTF("Error: LSM6DSV16X_0 KO\r\n");
        }
      } else {
        STBOX1_PRINTF("Error: LSM6DSV16X_0 KO\r\n");
      }
    } else {
      STBOX1_PRINTF("Error: LSM6DSV16X_0 KO\r\n");
    }
  } else {
    STBOX1_PRINTF("Error: LSM6DSV16X_0 KO\r\n");
  }
}

/** @brief DeInitialize the LSM6DSV16X MEMS Sensor
 * @param None
 * @retval None
 */
void DeInit_Acc(void)
{
  /* DeInit Accelero */
  if (BSP_MOTION_SENSOR_DeInit(LSM6DSV16X_0) == BSP_ERROR_NONE) {
    STBOX1_PRINTF("OK Deinit LSM6DSV16X_0  Sensor\n\r");
  } else {
    STBOX1_PRINTF("Error Deinit LSM6DSV16X_0 Sensor\n\r");
    STBOX1_Error_Handler(STBOX1_ERROR_SENSOR,__FILE__,__LINE__);
  }
}

/** @brief Initialize the LSM6DSV16X MEMS Sensor for MLC
 * @param uint32_t UseCustomIfAvailableflag for Using or not the Custom UCF file
 * @retval None
 */
void InitAcc_MLC(uint32_t UseCustomIfAvailable)
{
  ucf_line_t *ProgramPointer;
  int32_t LineCounter;
  int32_t TotalNumberOfLine;
  int32_t RetValue;

  /* Init Accelero */
  if (BSP_MOTION_SENSOR_Init(LSM6DSV16X_0, MOTION_ACCELERO) == BSP_ERROR_NONE) {
    STBOX1_PRINTF("OK Init Accelero  Sensor\n\r");
  } else {
    STBOX1_PRINTF("Error Init Accelero Sensor\n\r");
    STBOX1_Error_Handler(STBOX1_ERROR_SENSOR,__FILE__,__LINE__);
  }

  /* Feed the program to Machine Learning Core */
  if((UseCustomIfAvailable==1) & (MLCCustomUCFFile!=NULL)) {
    ProgramPointer    = MLCCustomUCFFile;
    TotalNumberOfLine = MLCCustomUCFFileLength;
    STBOX1_PRINTF("-->Custom UCF Program for LSM6DSOX MLC\r\n");
  } else {
    /* Activity Recognition Default program */
    ProgramPointer = (ucf_line_t *)HAR_DSV16X_4classes;
    TotalNumberOfLine = sizeof(HAR_DSV16X_4classes) / sizeof(ucf_line_t);
    STBOX1_PRINTF("-->Activity Recognition for LSM6DSV16X MLC\r\n");
    STBOX1_PRINTF("UCF Number Line=%ld\r\n",TotalNumberOfLine);
  }

  for (LineCounter=0; LineCounter<TotalNumberOfLine; LineCounter++) {
    RetValue = BSP_MOTION_SENSOR_Write_Register(LSM6DSV16X_0,
                                            ProgramPointer[LineCounter].address,
                                            ProgramPointer[LineCounter].data);
    if(RetValue!=BSP_ERROR_NONE) {
      STBOX1_PRINTF("Error loading the Program to LSM6DSV16X [%ld]->%lx\n\r",LineCounter,RetValue);
      STBOX1_Error_Handler(STBOX1_ERROR_SENSOR,__FILE__,__LINE__);
    }
  }

  STBOX1_PRINTF("Program loaded inside the LSM6DSV16X MLC\n\r");
}

/** @brief Initialize the LSM6DSV16X MEMS Sensor for FSM
 * @param uint32_t UseCustomIfAvailable flag for Using or not the Custom UCF file
 * @retval None
 */
void InitAcc_FSM(uint32_t UseCustomIfAvailable)
{
   ucf_line_t *ProgramPointer;
  int32_t LineCounter;
  int32_t TotalNumberOfLine;
  int32_t RetValue;

  /* Init Accelero */
  if (BSP_MOTION_SENSOR_Init(LSM6DSV16X_0, MOTION_ACCELERO) == BSP_ERROR_NONE) {
    STBOX1_PRINTF("OK Init Accelero  Sensor\n\r");
  } else {
    STBOX1_PRINTF("Error Init Accelero Sensor\n\r");
    STBOX1_Error_Handler(STBOX1_ERROR_SENSOR,__FILE__,__LINE__);
  }

  /* Feed the program to FiniteStateMachine */
  if((UseCustomIfAvailable==1) & (FSMCustomUCFFile!=NULL)) {
    ProgramPointer    = FSMCustomUCFFile;
    TotalNumberOfLine = FSMCustomUCFFileLength;
    STBOX1_PRINTF("-->Custom UCF Program for LSM6DSV16X FSM\r\n");
  } else {
    /* 4D position recognition Default program */
    ProgramPointer = (ucf_line_t *)lsm6dsv16x_four_d;
    TotalNumberOfLine = sizeof(lsm6dsv16x_four_d) / sizeof(ucf_line_t);
    STBOX1_PRINTF("-->4D position recognition for LSM6DSV16X FSM\r\n");
    STBOX1_PRINTF("UCF Number Line=%ld\r\n",TotalNumberOfLine);
  }

  for (LineCounter=0; LineCounter<TotalNumberOfLine; LineCounter++) {
    RetValue = BSP_MOTION_SENSOR_Write_Register(LSM6DSV16X_0,
                                            ProgramPointer[LineCounter].address,
                                            ProgramPointer[LineCounter].data);
    if(RetValue!=BSP_ERROR_NONE) {
      STBOX1_PRINTF("Error loading the Program to LSM6DSV16X [%ld]->%lx\n\r",LineCounter,RetValue);
      STBOX1_Error_Handler(STBOX1_ERROR_SENSOR,__FILE__,__LINE__);
    }
  }
  STBOX1_PRINTF("Program loaded inside the LSM6DSV16X FSM\n\r");
}

/**
  * @brief  Callback Interrupt 4
  * @param  None
  * @retval None
  */
static void hexti_callback(void)
{
  MEMSInterrupt=1;
}

/**
  * @brief  Callback for LSM6DS0X interrupt
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  lsm6dsv16x_all_sources_t      status;
  uint8_t MLCStatus;
  uint8_t FSMStatus;

  STBOX1_PRINTF("MEMSCallback\n\r");

  lsm6dsv16x_all_sources_get(LSM6DSV16X_Contex, &status);

  MLCStatus = (status.mlc1) | (status.mlc2<<1) | (status.mlc3<<2) | (status.mlc4<<3);

  FSMStatus = ((status.fsm1) | (status.fsm2<<1) | (status.fsm3<<2) | (status.fsm4<<3) |
                         (status.fsm5<<4) | (status.fsm6<<5) | (status.fsm7<<6) | (status.fsm8<<7));

  if(MLCStatus){
    lsm6dsv16x_mlc_out_t mlc_out;
    lsm6dsv16x_mlc_out_get(LSM6DSV16X_Contex, &mlc_out);

//    printf("MLC %d %d %d %d\r\n",mlc_out.mlc1_src,mlc_out.mlc2_src,mlc_out.mlc3_src,mlc_out.mlc4_src);

    /* Check if we need to update the MLC BLE char */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_MLC)) {
      BLE_MachineLearningCoreUpdate((uint8_t *) &mlc_out,&MLCStatus);
    }

    /* Check if we need to update the Activity Recognition BLE char */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR_EVENT)) {
      if (status.mlc1) {
        ActivityCode = MappingToHAR_ouput_t[mlc_out.mlc1_src];
        if(ActivityCode!=BLE_AR_ERROR) {
           BLE_ActRecUpdate(ActivityCode, HAR_ALGO_IDX_NONE);
          if( BLE_StdTerm_Service == BLE_SERV_ENABLE) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Rec ActivityCode %02X [%02X]\n",ActivityCode,mlc_out.mlc1_src);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            STBOX1_PRINTF("Rec ActivityCode %02X [%02X]\r\n",ActivityCode,mlc_out.mlc1_src);
          }
        } else {
          if(BLE_StdTerm_Service == BLE_SERV_ENABLE) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Wrong ActivityCode %02X [%02X]\n",ActivityCode,mlc_out.mlc1_src);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            STBOX1_PRINTF("Wrong ActivityCode %02X [%02X]\r\n",ActivityCode,mlc_out.mlc1_src);
          }
        }
      }
    }
  } else if(FSMStatus){
    lsm6dsv16x_fsm_out_t fsm_out;
    lsm6dsv16x_fsm_out_get(LSM6DSV16X_Contex, &fsm_out);

//    printf("FSM %d %d %d %d %d %d %d %d\r\n",fsm_out.fsm_outs1,fsm_out.fsm_outs2,fsm_out.fsm_outs3,fsm_out.fsm_outs4,
//           fsm_out.fsm_outs5,fsm_out.fsm_outs6,fsm_out.fsm_outs7,fsm_out.fsm_outs8);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_FSM)) {
      BLE_FiniteStateMachineUpdate((uint8_t *)&fsm_out,&FSMStatus,/* this is dummy */ &FSMStatus);
    }
  }
}

/**
* @brief  Initialize Timers
*
* @param  None
* @retval None
*/
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM instance */
  TIM_CC_HANDLE.Instance = TIM_CC_INSTANCE;
  TIM_CC_HANDLE.Init.Period        = 65535;
  TIM_CC_HANDLE.Init.Prescaler     = uwPrescalerValue;
  TIM_CC_HANDLE.Init.ClockDivision = 0;
  TIM_CC_HANDLE.Init.CounterMode   = TIM_COUNTERMODE_UP;

   if(HAL_TIM_OC_DeInit(&TIM_CC_HANDLE) != HAL_OK) {
    /* Initialization Error */
    STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }

  if(HAL_TIM_OC_Init(&TIM_CC_HANDLE) != HAL_OK) {
    /* Initialization Error */
    STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }

  /* Configure the Output Compare channels */

  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  sConfig.Pulse = STBOX1_UPDATE_LED;
  if(HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    /* Configuration Error */
    STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }

  sConfig.Pulse = STBOX1_UPDATE_INV;
  if(HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfig, TIM_CHANNEL_3) != HAL_OK) {
    /* Configuration Error */
    STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }
}

/**
* @brief  Initialize User process
*
* @param  None
* @retval None
*/
static void User_Init(void)
{
  /* Enable Button in Interrupt mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Init the Led */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_BLUE);

  /* why RED is activated by default? */
  BSP_LED_Off(LED_RED);

   /* Check if we are running from Bank1 or Bank2 */
  {
    FLASH_OBProgramInitTypeDef    OBInit;
    /* Allow Access to Flash control registers and user Flash */
    HAL_FLASH_Unlock();
    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();
    /* Get the Dual boot configuration status */
    HAL_FLASHEx_OBGetConfig(&OBInit);
    if (((OBInit.USERConfig) & (OB_SWAP_BANK_ENABLE)) == OB_SWAP_BANK_ENABLE) {
      CurrentActiveBank= 2;
      MCR_HEART_BIT2();
    } else {
      CurrentActiveBank= 1;
      MCR_HEART_BIT();
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
  }

  BSP_COM_Init(COM1);

  /* Check the board Type */
  FinishGood = BSP_CheckFinishGood();

  //Update the Current Fw ID saved in flash if it's neceessary
  if(FinishGood==FINISHA) {
    UpdateCurrFlashBankFwIdBoardName(STBOX1A_BLUEST_SDK_FW_ID,NULL);
  } else {
    UpdateCurrFlashBankFwIdBoardName(STBOX1B_BLUEST_SDK_FW_ID,NULL);
  }

  InitTimers();
}

/**
* @brief  Output Compare callback in non blocking mode
* @param  TIM_HandleTypeDef *htim TIM OC handle
* @retval None
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  /* TIM1_CH1 toggling with frequency = 0.5Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED));
    BlinkLed =1;
  }

  /* TIM1_CH2 toggling with frequency = 1Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + STBOX1_UPDATE_INV));
    SendAccGyroMag=1;
  }
}

/**
* @brief  BSP Push Button callback
*
* @param  Button Specifies the pin connected EXTI line
* @retval None
*/
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Set the User Button flag */
  UserButtonPressed = 1;
}

/**
* @brief  Print Bunner
* @param  None
* @retval None
*/
static void PrintInfo(void)
{
  if(FinishGood==FINISH_ERROR) {
   STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }

  STBOX1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
            "\tVersion %c.%c.%c\r\n"
              "\tSTM32U585AI-SensorTile.box-Pro (%c) board"
                "\r\n",
                STBOX1_PACKAGENAME,
                STBOX1_VERSION_MAJOR,STBOX1_VERSION_MINOR,STBOX1_VERSION_PATCH,
                  (FinishGood==FINISHA) ? 'A' : 'B');
  STBOX1_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
            "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
              " (IAR)\r\n",
#elif defined (__ARMCC_VERSION)
              " (KEIL)\r\n",
#elif defined (__GNUC__)
              " (STM32CubeIDE)\r\n",
#endif
              HAL_GetHalVersion() >>24,
              (HAL_GetHalVersion() >>16)&0xFF,
              (HAL_GetHalVersion() >> 8)&0xFF,
              HAL_GetHalVersion()      &0xFF,
              __DATE__,__TIME__);
   STBOX1_PRINTF("Current Bank =%ld\r\n",CurrentActiveBank);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param int32_t ErrorCode Error Code
  * @retval None
  */
void STBOX1_Error_Handler(int32_t ErrorCode,char *File,int32_t Line)
{
  /* User can add his own implementation to report the HAL error return state */
  BSP_LED_Off(LED_RED);
  STBOX1_PRINTF("Error at %ld at %s\r\n",Line,File);
  while (1){
    int count;
    for(count=0;count<ErrorCode;count++) {
      BSP_LED_On(LED_RED);
      HAL_Delay(500);
      BSP_LED_Off(LED_RED);
      HAL_Delay(2000);
    }
    BSP_LED_On(LED_GREEN);
    BSP_LED_On(LED_YELLOW);
    HAL_Delay(2000);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_YELLOW);
  }
}

/******************************************************************************/
/* STM32U5xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32u5xx.s).                    */
/******************************************************************************/

/**
 * @brief  This method the Finish Good type
 * @retval FinishGood value
 */
FinishGood_TypeDef BSP_CheckFinishGood(void) {

  #define ST25_ADDR_DATA_I2C                ((uint8_t)0xAE)
  #define ST25_ICREF_REG                    ((uint16_t)0x0017)
  /* ST25DVxxKC 4Kbits ICref */
  #define IAM_ST25DV04KC                        0x50U
  /* ST25DVxxKC 16/64Kbits ICref */
  #define IAM_ST25DV64KC                        0x51U
  /* @brief ST25DV 4Kbits ICref */
  #define IAM_ST25DV04                        0x24
  /* @brief ST25DV 16/64Kbits ICref */
  #define IAM_ST25DV64                        0x26

  FinishGood_TypeDef FinishGood = FINISH_ERROR;
  uint8_t nfctag_id;
  BSP_ST25DV_I2C_INIT();

  BSP_ST25DV_I2C_READ_REG_16(ST25_ADDR_DATA_I2C, ST25_ICREF_REG, &nfctag_id, 1);

  if((nfctag_id == IAM_ST25DV04KC) | (nfctag_id == IAM_ST25DV64KC)) {
    FinishGood = FINISHB;
  } else if((nfctag_id == IAM_ST25DV04) | (nfctag_id == IAM_ST25DV64)) {
    FinishGood = FINISHA;
  }

  BSP_ST25DV_I2C_DEINIT();

  #undef ST25_ADDR_DATA_I2C
  #undef ST25_ICREF_REG
  #undef IAM_ST25DV04KC
  #undef IAM_ST25DV64KC
  #undef IAM_ST25DV04
  #undef IAM_ST25DV64

  return FinishGood;
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

