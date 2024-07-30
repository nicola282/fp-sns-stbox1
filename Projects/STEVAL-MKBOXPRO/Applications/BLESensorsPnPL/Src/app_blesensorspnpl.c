/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_blesensorspnpl.c
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
 * @page BLESensorsPnPL BLE Firmware example using PnP-Like messages
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

 * <b>Example Application</b>

 * This example shows how it's possible to customize the demo present on ST BLE Sensors Application using the PnP-Like Messages
 * This example is compatible with the Firmware Over the Air Update (FoTA)
 *
 *
 * This example must be used with the related ST BLE Sensor Android/iOS application available on Play/itune store
 * (Version 4.18.0 or higher), in order to read the sent information by Bluetooth Low Energy protocol
 *
 *
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "app_blesensorspnpl.h"
#include "steval_mkboxpro.h"
#include "bluenrg_conf.h"
#include "BLE_Manager.h"
#include "OTA.h"
#include "SensorTileBoxPro_env_sensors.h"
#include "SensorTileBoxPro_motion_sensors.h"
#include "SensorTileBoxPro_gg.h"
#include "BLE_Function.h"
#include "STBOX1_config.h"

#include "PnPLCompManager.h"
#include "IControl.h"
#include "Configuration_PnPL.h"
#include "Control_PnPL.h"
#include "Environmental_PnPL.h"
#include "Inertial_PnPL.h"
#include "Deviceinformation_PnPL.h"

/* Exported variables --------------------------------------------------------*/
uint16_t ConnectionHandle = 0;

int32_t CurrentActiveBank = 0;

void *HandleGGComponent;
uint8_t BatteryPresent=0;
STBOX1_Acc_t CurrentAccType = STBOX1_ACC_LSM6DSV16X;

FinishGood_TypeDef FinishGood;

/* Imported variables --------------------------------------------------------*/
volatile uint32_t hci_event;
extern IPnPLComponent_t *pConfigurationPnPLObj;
extern IPnPLComponent_t *pControlPnPLObj;
extern IPnPLComponent_t *pEnvironmentalPnPLObj;
extern IPnPLComponent_t *pInertialPnPLObj;
extern IPnPLComponent_t *pDeviceInformationPnPLObj;
extern IControl_t iControl;

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t user_button_pressed = 0;

static volatile uint32_t BlinkLed        = 0;
static volatile uint32_t SendEnv         = 0;
static volatile uint32_t SendAccGyroMag  = 0;
static volatile uint32_t SendBatteryInfo = 0;

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);

static void InitTimers(void);
static void SendBatteryInfoData(void);
static void PrintInfo(void);
static void InitMemsSensors(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval none
  */
void MX_BLESensorsPnPL_Init(void)
{
  /* Set a random seed */
  srand(HAL_GetTick());

  User_Init();

  STBOX1_PRINTF("\033[2J"); /* serial console clear screen */
  STBOX1_PRINTF("\033[H");  /* serial console cursor to home */
  PrintInfo();

  BSP_LED_On(LED_BLUE);

  /* Init Mems Sensors */
  InitMemsSensors();

  /* Init Gas Gouge */
  {
   uint32_t flag;
    DrvStatusTypeDef InitType = BSP_GG_Init(&HandleGGComponent);

    switch(InitType) {
      case COMPONENT_OK:
        STBOX1_PRINTF("Gas Gouge Component OK\n\r");
        BSP_GG_GetPresence(HandleGGComponent,&flag);
        if(flag) {
          STBOX1_PRINTF("\tBattery present\n\r");
          BatteryPresent=1;
        } else {
          STBOX1_PRINTF("\tBattery not present\n\r");
          BatteryPresent=0;
        }
      break;
      case COMPONENT_BATT_FAIL:
        STBOX1_PRINTF("Gas Gouge Component OK\n\r");
        STBOX1_PRINTF("\tBattery not present\n\r");
        BatteryPresent=0;
      break;
      default:
        STBOX1_PRINTF("Gas Gouge Component ERROR\n\r");
        BatteryPresent=0;
    }
  }

  /* Init BLE */
  STBOX1_PRINTF("\r\nInitializing Bluetooth\r\n");
  BluetoothInit();
  /* For Receiving information on Response Event for a MTU Exchange Event */
  CustomMTUExchangeRespEvent = MTUExcahngeRespEvent;
  /* For Receiving information on aci_gatt_tx_pool_available_event */
  CustomAciGattTxPoolAvailableEvent = TxPoolAvailableEvent;

  /* PnP-L Components Allocation */
  pConfigurationPnPLObj = Configuration_PnPLAlloc();
  pControlPnPLObj = Control_PnPLAlloc();
  pEnvironmentalPnPLObj = Environmental_PnPLAlloc();
  pInertialPnPLObj = Inertial_PnPLAlloc();
  pDeviceInformationPnPLObj = Deviceinformation_PnPLAlloc();

  /* Init&Add PnP-L Components */
  Configuration_PnPLInit(pConfigurationPnPLObj);
  Control_PnPLInit(pControlPnPLObj, &iControl);
  Environmental_PnPLInit(pEnvironmentalPnPLObj);
  Inertial_PnPLInit(pInertialPnPLObj);
  Deviceinformation_PnPLInit(pDeviceInformationPnPLObj);

  if(FinishGood==FINISHA) {
    PnPLSetBOARDID(BLE_MANAGER_SENSOR_TILE_BOX_PRO_PLATFORM);
    PnPLSetFWID(STBOX1A_BLUEST_SDK_FW_ID);
  } else {
    PnPLSetBOARDID(BLE_MANAGER_SENSOR_TILE_BOX_PRO_B_PLATFORM);
    PnPLSetFWID(STBOX1B_BLUEST_SDK_FW_ID);
  }

  /* FOTA and Dual Banks Section */
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
void MX_BLESensorsPnPL_Process(void)
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
      __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED_BATTERY));

      setConnectable();
      set_connectable = FALSE;
    }

    /*  Update sensor value */
    if (SendEnv) {
      float Temp,Pressure;
      SendEnv=0;

      BSP_ENV_SENSOR_GetValue(STTS22H_0, ENV_TEMPERATURE, &Temp);
      BSP_ENV_SENSOR_GetValue(LPS22DF_0, ENV_PRESSURE, &Pressure);
      BLE_EnvironmentalUpdate((int32_t)(Pressure *100),0,(int16_t)(Temp * 10),0);
    }

    if (SendAccGyroMag) {
      BLE_MANAGER_INERTIAL_Axes_t x_axes;
      BLE_MANAGER_INERTIAL_Axes_t g_axes;
      BLE_MANAGER_INERTIAL_Axes_t m_axes;
      SendAccGyroMag=0;

      if(CurrentAccType == STBOX1_ACC_LSM6DSV16X) {
        BSP_MOTION_SENSOR_GetAxes(LSM6DSV16X_0, MOTION_ACCELERO,(BSP_MOTION_SENSOR_Axes_t*)&x_axes);
      } else {
        BSP_MOTION_SENSOR_GetAxes(LIS2DU12_0, MOTION_ACCELERO,(BSP_MOTION_SENSOR_Axes_t*)&x_axes);
      }

      BSP_MOTION_SENSOR_GetAxes(LSM6DSV16X_0, MOTION_GYRO,(BSP_MOTION_SENSOR_Axes_t*)&g_axes);
      BSP_MOTION_SENSOR_GetAxes(LIS2MDL_0, MOTION_MAGNETO,(BSP_MOTION_SENSOR_Axes_t*)&m_axes);
      BLE_AccGyroMagUpdate(&x_axes,&g_axes,&m_axes);
    }

    /* Send Battery Info */
    if(SendBatteryInfo) {
      SendBatteryInfo=0;
      SendBatteryInfoData();
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
    if(user_button_pressed) {
      user_button_pressed=0;
      STBOX1_PRINTF("User Button pressed...\r\n");
//      if(connected == FALSE) {
//        /* Just for testing that the BLE_Manager is able to restart */
//        RestartBLE_Manager();
//      }
    }

    /* Blinking the Led */
    if(BlinkLed) {
      BlinkLed = 0;
      BSP_LED_Toggle(LED_GREEN);
    }

    /* Check if we need to send a Chunck of Data for PnPL */
    if(JSON_string_command_wTP!=NULL) {
      PnPLikeSendChunckData();
    }

    /* Wait next event */
    __WFI();
}

/**
* @brief  Init Mems Sensors
* @param  None
* @retval None
*/
static void InitMemsSensors(void)
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

  /* Acc2 */
  if(BSP_MOTION_SENSOR_Init(LIS2DU12_0, MOTION_ACCELERO)==BSP_ERROR_NONE) {
    if(BSP_MOTION_SENSOR_SetOutputDataRate(LIS2DU12_0, MOTION_ACCELERO, LIS2DU12_ACC_ODR)==BSP_ERROR_NONE) {
      if(BSP_MOTION_SENSOR_SetFullScale(LIS2DU12_0, MOTION_ACCELERO, LIS2DU12_ACC_FS)==BSP_ERROR_NONE) {
         STBOX1_PRINTF("LIS2DU12_0 OK\r\n");
      } else {
        STBOX1_PRINTF("Error: LIS2DU12_0 KO\r\n");
      }
    } else {
      STBOX1_PRINTF("Error: LIS2DU12_0 KO\r\n");
    }
  } else {
    STBOX1_PRINTF("Error: LIS2DU12_0 KO\r\n");
  }

  /* Pressure */
  if(BSP_ENV_SENSOR_Init(LPS22DF_0, ENV_PRESSURE)==BSP_ERROR_NONE) {
    if(BSP_ENV_SENSOR_SetOutputDataRate(LPS22DF_0, ENV_PRESSURE, LPS22DF_ODR)==BSP_ERROR_NONE) {
      STBOX1_PRINTF("LPS22DF_0 OK\r\n");
    } else {
      STBOX1_PRINTF("Error: LPS22DF_0 KO\r\n");
    }
  } else {
    STBOX1_PRINTF("Error: LPS22DF_0 KO\r\n");
  }

  /* Temperature 2 */
  if(BSP_ENV_SENSOR_Init(STTS22H_0, ENV_TEMPERATURE)==BSP_ERROR_NONE) {
    if(BSP_ENV_SENSOR_SetOutputDataRate(STTS22H_0, ENV_TEMPERATURE, STTS22H_ODR)==BSP_ERROR_NONE) {
      STBOX1_PRINTF("STTS22H_0 OK\r\n");
    } else {
      STBOX1_PRINTF("Error: STTS22H_0 KO\r\n");
    }
  } else {
    STBOX1_PRINTF("Error: STTS22H_0 KO\r\n");
  }
}

/**
* @brief  Send Battery Info Data (Voltage/Current/Soc) to BLE
* @param  None
* @retval None
*/
static void SendBatteryInfoData(void)
{
  uint32_t BatteryLevel;
  int32_t Current= 0;

  uint32_t Voltage;
  uint8_t v_mode;
  uint32_t Status  = 0x04; /* Unknown */

  if(BatteryPresent) {
    /* Update Gas Gouge Status */
    BSP_GG_Task(HandleGGComponent,&v_mode);

    /* Read the Gas Gouge Status */
    BSP_GG_GetVoltage(HandleGGComponent, &Voltage);
    BSP_GG_GetCurrent(HandleGGComponent, &Current);
    BSP_GG_GetSOC(HandleGGComponent, &BatteryLevel);

    /* if it's < 15% Low Battery */
    if(BatteryLevel<15) {
      Status = 0x00; /* Low Battery */
    } else {
      if(Current < 0) {
        Status = 0x01; /* Discharging */
      } else  {
        Status = 0x03; /* Charging */
      }
    }
  } else {
    Status = 0x02; //Plugged Not Charging
    Current = 0x8000;
    Voltage=0;
    BatteryLevel=0;
  }
  BLE_BatteryUpdate(BatteryLevel, Voltage, Current/10, Status);
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

  /* Set TIM1 instance */
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

  sConfig.Pulse = STBOX1_UPDATE_LED_BATTERY;
  if(HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
    /* Configuration Error */
    STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }

  switch(CurrentEnvUpdateEnumValue) {
    case 1:
      sConfig.Pulse = 10000;
    break;
    case 10:
      sConfig.Pulse = 1000;
    break;
    case 20:
      sConfig.Pulse = 500;
    break;
  }
  if(HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
    /* Configuration Error */
    STBOX1_Error_Handler(STBOX1_ERROR_HW_INIT,__FILE__,__LINE__);
  }

  switch(CurrentInerUpdateEnumValue) {
    case 10:
      sConfig.Pulse = 1000;
    break;
    case 20:
      sConfig.Pulse = 500;
    break;
    case 30:
      sConfig.Pulse = 333;
    break;
  }
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
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED_BATTERY));
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_BAT_EVENT)) {
      SendBatteryInfo=1;
    } else {
      BlinkLed =1;
    }
  }

  /* TIM1_CH2 toggling with frequency = 1Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    switch(CurrentEnvUpdateEnumValue) {
      case 1:
        __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_2, (uhCapture + 10000));
      break;
      case 10:
        __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_2, (uhCapture + 1000));
      break;
      case 20:
        __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_2, (uhCapture + 500));
      break;
    }
    SendEnv=1;
  }

  /* TIM1_CH3 toggling with frequency = 1Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    switch(CurrentInerUpdateEnumValue) {
      case 10:
        __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + 1000));
      break;
      case 20:
        __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + 500));
      break;
      case 30:
        __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + 333));
      break;
    }
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
  user_button_pressed = 1;
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
  * @brief This function provides accurate delay (in milliseconds) based
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
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

