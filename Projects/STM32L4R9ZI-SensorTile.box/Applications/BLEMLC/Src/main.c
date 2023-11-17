/**
  ******************************************************************************
  * @file    BLEMLC\Src\main.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Main program body
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
 *
 * @page BLEMLC Example on how to program the Machine Learning Core and Finite State Machine
 * 
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * This firmware package includes Components Device Drivers, Board Support Package
 * and example application for the following STMicroelectronics elements:
 * - STEVAL-MKSBOX1V1 (SensorTile.box) evaluation board that contains the following components:
 *     - MEMS sensor devices: HTS221, LPS22HH, LIS2MDL, LSM6DSOX
 *     - analog microphone
 *
 *
 * <b>Example Application</b>
 *
 * The Example application initializes all the Components, Loading the program on Machine Learning Core (MLC)
 * or Finite State Machine (FSM), initializes the Library creating some Custom Bluetooth services:
 * - The first service exposes all the HW/SW characteristics:
 *    - LSM6DSOX MLC results: Activity Recognition
 *    - LSM6DS0X accelerometer and Gyroscope values
 *    - LSM6DSOX MLC output registers (Only available starting from V4.6.0 of ST BLE Sensor Application)
 *    - LSM6DSOX FSM output registers (Only available starting from V4.6.0 of ST BLE Sensor Application)
 *    - Battery status
 * - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * This example must be used with the related ST BLE Sensor Android/iOS application available on Play/itune store (Version 4.6.0 or higher),
 * in order to read the sent information by Bluetooth Low Energy protocol
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "TargetFeatures.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Imported Variables --------------------------------------------------------*/
extern uint8_t set_connectable;

/* Exported Variables --------------------------------------------------------*/
uint8_t bdaddr[6];
char BoardName[8]={NAME_BLUEMS,0};

BLE_AR_output_t ActivityCode = BLE_AR_ERROR;

#ifdef STBOX1_RESTART_DFU
  #if defined (__GNUC__)
    uint32_t DFU_Var __attribute__ ((section (".noinit")));
  #elif defined (__ARMCC_VERSION)
    uint32_t DFU_Var  __attribute__((at(0x2009FFF0)));
  #elif defined (__IAR_SYSTEMS_ICC__)
    __no_init uint32_t DFU_Var;
  #endif /* defined (__GNUC__) */
#endif /* STBOX1_RESTART_DFU */

/* Private variables ---------------------------------------------------------*/
static volatile int ButtonPressed   = 0;
static volatile int SendBatteryInfo = 0;
volatile        int PowerOff        = 0;
volatile        int RebootBoard     = 0;
volatile        int hci_event       = 0;
static volatile int BlinkLed        = 0;
static volatile int MEMSInterrupt   = 0;
static volatile int SendAccGyro     = 0;

/* Private function prototypes -----------------------------------------------*/

//static void Init_BlueNRG_Custom_Services(void);
//static void Init_BlueNRG_Stack(void);

static void ButtonCallback(void);
static void MEMSCallback(void);
static void SystemClock_Config(void);
static void SendBatteryInfoData(void);
static void AccGyro_Read(BSP_MOTION_SENSOR_Axes_t *ACC_Value,BSP_MOTION_SENSOR_Axes_t *GYR_Value);

void APP_UserEvtRx(void *pData);

#ifdef STBOX1_ENABLE_PRINTF
static void DisplayFirmwareInfo(void);
#endif /* STBOX1_ENABLE_PRINTF */

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

#ifdef STBOX1_RESTART_DFU
  /* Check if we need to go to DFU */
  if(DFU_Var == DFU_MAGIC_NUM) {
    typedef void (*pFunction)(void);
    pFunction JumpToDFU;

    /* Reset the Magic Number */
    DFU_Var = 0x00000000;
    HAL_RCC_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    __disable_irq();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    JumpToDFU = (void (*)(void)) (*((uint32_t *)(0x1FFF0000 + 4)));
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) 0x1FFF0000);
    JumpToDFU();
  }
#endif /* STBOX1_RESTART_DFU */

  /* HAL_Init */
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

  InitTargetPlatform();

#ifdef STBOX1_ENABLE_PRINTF
  DisplayFirmwareInfo();
#endif /* STBOX1_ENABLE_PRINTF */

  /* Initialize the BlueNRG */
  STBOX1_PRINTF("\r\nInitializing Bluetooth\r\n");
  BluetoothInit();

  while (1){
    if(set_connectable){
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      set_connectable =0;
      setConnectable();

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }

      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED));
    }

    /* Handle BLE event */
    if(hci_event) {
      hci_event =0;
      hci_user_evt_proc();
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;
    }

    /* Power Off the SensorTile.box */
    if(PowerOff){
      BSP_BC_CmdSend(SHIPPING_MODE_ON);
      PowerOff =0;
    }

    /* Interrupt from LSM6DSOX */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt =0;
    }

    /* Reboot the Board */
    if(RebootBoard) {
      RebootBoard=0;
      HAL_NVIC_SystemReset();
    }

    /* Handle Acc/Gyro */
    if(SendAccGyro) {
      BSP_MOTION_SENSOR_Axes_t ACC_Value;
      BSP_MOTION_SENSOR_Axes_t GYR_Value;

      BLE_MANAGER_INERTIAL_Axes_t ACC_SensorValue;
      BLE_MANAGER_INERTIAL_Axes_t GYR_SensorValue;
      BLE_MANAGER_INERTIAL_Axes_t MAG_SensorValue;

      SendAccGyro =0;

      /* Read the Inertial Sensors */
      AccGyro_Read(&ACC_Value,&GYR_Value);
      
      ACC_SensorValue.Axis_x= ACC_Value.x;
      ACC_SensorValue.Axis_y= ACC_Value.y;
      ACC_SensorValue.Axis_z= ACC_Value.z;
      
      GYR_SensorValue.Axis_x= GYR_Value.x;
      GYR_SensorValue.Axis_y= GYR_Value.y;
      GYR_SensorValue.Axis_z= GYR_Value.z;

      MAG_SensorValue.Axis_x= 0;
      MAG_SensorValue.Axis_y= 0;
      MAG_SensorValue.Axis_z= 0;

      /* Send the Data with BLE */
      BLE_AccGyroMagUpdate(&ACC_SensorValue,&GYR_SensorValue,&MAG_SensorValue);
    }

    /* Battery Info Data */
    if(SendBatteryInfo) {
      SendBatteryInfo=0;
      SendBatteryInfoData();
    }

    /* Blinking the Led */
    if(BlinkLed) {
      BlinkLed = 0;
      LedToggleTargetPlatform();
    }

    /* Wait next event */
    __WFI();
  }
}

#ifdef STBOX1_ENABLE_PRINTF
/**
  * @brief Display all the Firmware Information
  * @param  None
  * @retval None
  */
static void DisplayFirmwareInfo(void)
{
    STBOX1_PRINTF("\r\n------------------------------------------------------------\r\n");
    STBOX1_PRINTF("STMicroelectronics %s\r\n"
         "\tVersion %c.%c.%c\r\n"
        "\tSTM32L4R9ZI-SensorTile.box board\r\n",
          STBOX1_PACKAGENAME,
          STBOX1_VERSION_MAJOR,STBOX1_VERSION_MINOR,STBOX1_VERSION_PATCH);

    STBOX1_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n\tCompiled %s %s"
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
  STBOX1_PRINTF("------------------------------------------------------------\r\n");
}
#endif /* STBOX1_ENABLE_PRINTF */

/**
  * @brief  Read The Inertial Data (Acc/Gyro)
  * @param  BSP_MOTION_SENSOR_Axes_t ACC_Value Structure containing acceleration value in mg
  * @param  BSP_MOTION_SENSOR_Axes_t GYR_Value Structure containing Gyroscope value
  * @retval None
 */
static void AccGyro_Read(BSP_MOTION_SENSOR_Axes_t *ACC_Value,BSP_MOTION_SENSOR_Axes_t *GYR_Value)
{
  /* Read the Acc values */
  BSP_MOTION_SENSOR_GetAxes(LSM6DSOX_0,MOTION_ACCELERO,ACC_Value);

  /* Read the Gyro values */
  BSP_MOTION_SENSOR_GetAxes(LSM6DSOX_0,MOTION_GYRO,GYR_Value);
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  STBOX1_PRINTF("User button pressed\r\n");
  STBOX1_PRINTF("The board will be switched off\r\n");
  PowerOff =1;
}

/**
  * @brief  Callback for LSM6DS0X interrupt
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  lsm6dsox_all_sources_t      status;
  uint8_t MLCStatus;
  uint8_t FSMStatus;
  
  STBOX1_PRINTF("MEMSCallback\n\r");
  
  lsm6dsox_all_sources_get(LSM6DSOX_CONTEX, &status);
  
  MLCStatus = ((status.mlc1)    | (status.mlc2<<1) | (status.mlc3<<2) | (status.mlc4<<3) |
                       (status.mlc5<<4) | (status.mlc6<<5) | (status.mlc7<<6) | (status.mlc8<<7));
  
  FSMStatus = ((status.fsm1)     | (status.fsm2<<1)  | (status.fsm3<<2)  | (status.fsm4<<3)  |
                       (status.fsm5<<4)  | (status.fsm6<<5)  | (status.fsm7<<6)  | (status.fsm8<<7)  |
                       (status.fsm9<<8)  | (status.fsm10<<9)  | (status.fsm11<<10) | (status.fsm12<<11) |
                       (status.fsm13<<12) | (status.fsm14<<13) | (status.fsm15<<14) | (status.fsm16<<15));

  if(MLCStatus!=0) {
    uint8_t mlc_out[8];
    lsm6dsox_mlc_out_get(LSM6DSOX_CONTEX, mlc_out);

    /* Check if we need to update the MLC BLE char */
    if(MLC_Enabled) {
      BLE_MachineLearningCoreUpdate(mlc_out,&MLCStatus);
    }

    /* Check if we need to update the Activity Recognition BLE char */
    if(AR_Enabled) {
      if (status.mlc1) {
        ActivityCode = BLE_MappingToHAR_ouput_t[mlc_out[0]];
        if(ActivityCode!=BLE_AR_ERROR) {
          BLE_ActRecUpdate(ActivityCode, HAR_MLC_LSM6DSOX_ID);
          if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Rec ActivityCode %02X [%02X]\n",ActivityCode,mlc_out[0]);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            STBOX1_PRINTF("Rec ActivityCode %02X [%02X]\r\n",ActivityCode,mlc_out[0]);
          }
        } else {
          if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Wrong ActivityCode %02X [%02X]\n",ActivityCode,mlc_out[0]);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            STBOX1_PRINTF("Wrong ActivityCode %02X [%02X]\r\n",ActivityCode,mlc_out[0]);
          }
        }
      }
    }
  } else  if(FSMStatus!=0) {
    lsm6dsox_fsm_out_t fsm_out;
    lsm6dsox_fsm_out_get(LSM6DSOX_CONTEX, &fsm_out);

    if(FSM_Enabled) {
      BLE_FiniteStateMachineUpdate((uint8_t *)&fsm_out,&FSMStatus,/* this is dummy */ &FSMStatus);
    }
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
  uint32_t Voltage;

  /* Read the Battery Charger voltage and Level values */
  BSP_BC_GetVoltageAndLevel(&Voltage,&BatteryLevel);

  BLE_BatteryUpdate(BatteryLevel,Voltage,0x8000 ,0x04 /* Unknown */);
}

/**
  * @brief  System Clock tree configuration
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
  
   /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|
                                     RCC_OSCILLATORTYPE_LSE  |
                                     RCC_OSCILLATORTYPE_HSE  |
                                     RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK   |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1  |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1   |
                                      RCC_PERIPHCLK_DFSDM1 |
                                      RCC_PERIPHCLK_USB    |
                                      RCC_PERIPHCLK_RTC    |
                                      RCC_PERIPHCLK_SDMMC1 |
                                      RCC_PERIPHCLK_ADC;

  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;  
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK2;  
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_PLLP;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 5;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 96;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV25;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK|RCC_PLLSAI1_SAI1CLK;  
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    /* Initialization Error */
    while(1);
  }
}

///**
//  * @brief This function provides accurate delay (in milliseconds) based
//  *        on variable incremented.
//  * @note This is a user implementation using WFI state
//  * @param Delay: specifies the delay time length, in milliseconds.
//  * @retval None
//  */
//void HAL_Delay(__IO uint32_t Delay)
//{
//  uint32_t tickstart = 0;
//  tickstart = HAL_GetTick();
//  while((HAL_GetTick() - tickstart) < Delay){
//    __WFI();
//  }
//}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  STBOX1_PRINTF("Error_Handler\r\n");
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pin connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
    case HCI_TL_SPI_EXTI_PIN:
      hci_tl_lowlevel_isr();
      hci_event=1;
    break;
    case USER_BUTTON_PIN:
      ButtonPressed = 1;    
    break;
    case GPIO_PIN_2:
    case GPIO_PIN_3:
      MEMSInterrupt = 1;
    break;
  }
}


/**
  * @brief  Output Compare callback in non blocking mode
  * @param  TIM_HandleTypeDef *htim TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  /* TIM1_CH1 toggling with frequency = 1Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED));

    BlinkLed =1;
  }

  /* TIM1_CH2 toggling with frequency = 0.5Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + STBOX1_UPDATE_BATTERY));
    SendBatteryInfo=1;
  }

#ifdef STBOX1_ENABLE_PRINTF
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + STBOX1_UPDATE_VCOM));
    CDC_PeriodElapsedCallback();
  }
#endif /* STBOX1_ENABLE_PRINTF */

  /* TIM1_CH4 toggling with frequency = 0.5Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + STBOX1_UPDATE_INV));
    SendAccGyro=1;
  }
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: STBOX1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif
