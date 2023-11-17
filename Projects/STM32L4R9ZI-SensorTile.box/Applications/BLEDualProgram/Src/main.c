/**
  ******************************************************************************
  * @file    BLEDualProgram\Src\main.c
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
 * @page BLEDualProgram Dual Boot Secure BLE Firmware Over the Air Update
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
 * <b>Example Application</b>
 *
 * The Example application initializes all the Components and Library creating some Custom Bluetooth services:
 * - The first service exposes the Battery status
 * - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 * - The last Service is used for configuration purpose
 *
 * The application allows connections only from bonded devices (PIN request) allowing them to updated its firwmare with 
 * one Over the Air update.
 *
 * Pressing the User button is possible to emulate one HW fault that will reboot the board starting from the
 * Program saved in the other flash bank
 *
 * This example must be used with the related ST BLE Sensor Android/iOS application available on Play/itune store
 * (Version 4.6.0 or higher), in order to read the sent information by Bluetooth Low Energy protocol
 *
 *                              --------------------
 *                              |  VERY IMPORTANT  |
 *                              -------------------- 
 *
 * For each IDE (IAR/µVision/STM32CubeIDE),
 * there are some scripts *.bat and *.sh that makes the following operations:
 * - Full Flash Erase
 * - Set the User Option Byte for booting from Bank 1
 * - Load the Program (after the compilation)
 * - Reset the board
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "TargetFeatures.h"
#include "main.h"

#include "OTA.h"
#include "BLE_Manager.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Imported Variables --------------------------------------------------------*/
extern uint8_t set_connectable;
/* Exported Variables --------------------------------------------------------*/

uint8_t bdaddr[6];

#ifdef STBOX1_RESTART_DFU
  #if defined (__GNUC__)
    uint32_t DFU_Var __attribute__ ((section (".noinit")));
  #elif defined (__ARMCC_VERSION)
    uint32_t DFU_Var  __attribute__((at(0x2009FFF0)));
  #elif defined (__IAR_SYSTEMS_ICC__)
    __no_init uint32_t DFU_Var;
  #endif /* defined (__GNUC__) */
#endif /* STBOX1_RESTART_DFU */

volatile int32_t PowerButtonPressed  = 0;
volatile int hci_event           = 0;
volatile int BlinkLed            = 0;
volatile uint32_t NeedToClearSecureDB = 0;

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t SendBatteryInfo = 0;
static volatile uint32_t SendQuat        = 0;
static volatile uint32_t ButtonPressed   = 0;

/* Private function prototypes -----------------------------------------------*/

static void SystemClock_Config(void);
static void SendBatteryInfoData(void);
static void Set_Random_Motion_Values(BLE_MOTION_SENSOR_Axes_t *q_axes,uint32_t cnt);
static void Reset_Motion_Values(BLE_MOTION_SENSOR_Axes_t *q_axes);

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

  /* Initialize the plaftorm */
  InitTargetPlatform();
  
  /* Set a random seed */
  srand(HAL_GetTick());

#ifdef STBOX1_ENABLE_PRINTF
  DisplayFirmwareInfo();
#endif /* STBOX1_ENABLE_PRINTF */

  /* Reset the volatile for avoid to power off the board */
  PowerButtonPressed  =0;
  
  //Update the Current Fw ID saved in flash if it's neceessary
  UpdateCurrFlashBankFwId(STBOX1_BLUEST_SDK_FW_ID);
  
  /* Init BLE */
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
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED_BATTERY));

      /* If we need to clear the Secure DataBase */
      if(NeedToClearSecureDB) {
        tBleStatus RetStatus;
        NeedToClearSecureDB =0;
        HAL_Delay(500);

        RetStatus =aci_gap_clear_security_db();
        if(RetStatus==BLE_STATUS_SUCCESS) {
          STBOX1_PRINTF("\tSecurity Data Base Cleared\r\n");
          MCR_HEART_BIT();
        } else {
          STBOX1_PRINTF("\tError Clearing the Security Data Base [%x]\r\n",RetStatus);
        }
      }
    }

    /* Handle BLE event */
    if(hci_event) {
      hci_event =0;
      hci_user_evt_proc();
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonPressed=0;
      STBOX1_PRINTF("Simulating one HW fault..\r\n");
      /* Stop the TIM Base generation in interrupt mode for IDWG for Simulating one HW problem */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }

    /* Power Off the SensorTile.box */
    if(PowerButtonPressed){
      BSP_BC_CmdSend(SHIPPING_MODE_ON);
      PowerButtonPressed =0;
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

    /* Battery Info Data */
    if(SendBatteryInfo) {
      SendBatteryInfo=0;
      if(paired == TRUE) {
        //Send values only if we are paired with the Device */
        SendBatteryInfoData();
      }
    }
    
    if (SendQuat) {
      static uint32_t counter = 0;
      static BLE_MOTION_SENSOR_Axes_t q_axes = {0,0,0};
      SendQuat=0;
      
      /* Update emulated Sensor Fusion data */
      Set_Random_Motion_Values(&q_axes,counter);
      if(paired == TRUE) {
        //Send Emulated Values only if we are paired with the Device */
        BLE_SensorFusionUpdate(&q_axes, 1);
      }
      counter++;
      if (counter == 40) {
        counter = 0;
        Reset_Motion_Values(&q_axes);
      }
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
  STBOX1_PRINTF("Current Bank =%ld\r\n",CurrentActiveBank);
  STBOX1_PRINTF("------------------------------------------------------------\r\n");
}
#endif /* STBOX1_ENABLE_PRINTF */

/**
* @brief  Set random values for Quaternion Value
* @param  BLE_MOTION_SENSOR_Axes_t *q_axes  Quaternion Value
* @param  uint32_t counter for changing the rotation direction
* @retval None
*/
static void Set_Random_Motion_Values(BLE_MOTION_SENSOR_Axes_t *q_axes,uint32_t cnt)
{
  if (cnt < 20) {
    q_axes->Axis_x -= (100  + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    q_axes->Axis_y += (100  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
    q_axes->Axis_z -= (100  + ((uint64_t)rand()*7*cnt)/RAND_MAX);
  } else {
    q_axes->Axis_x += (200 + ((uint64_t)rand()*7*cnt)/RAND_MAX);
    q_axes->Axis_y -= (150 + ((uint64_t)rand()*3*cnt)/RAND_MAX);
    q_axes->Axis_z += (10  + ((uint64_t)rand()*5*cnt)/RAND_MAX);
  }
}

/**
* @brief  Reset values for Quaternion Value
* @param  BLE_MOTION_SENSOR_Axes_t *q_axes  Quaternion Value
* @retval None
*/
static void Reset_Motion_Values(BLE_MOTION_SENSOR_Axes_t *q_axes)
{
  q_axes->Axis_x = -q_axes->Axis_x;
  q_axes->Axis_y = -q_axes->Axis_y;
  q_axes->Axis_z = -q_axes->Axis_z;
}

/**
  * @brief  Send Battery Info Data (Voltage/Current/Soc) to BLE
  * @param  None
  * @retval None
  */
static void SendBatteryInfoData(void)
{
  uint32_t Status;  
  stbc02_State_TypeDef BC_State = {(stbc02_ChgState_TypeDef)0, ""};
  uint32_t BatteryLevel;
  uint32_t Voltage;

  /* Read the Battery Charger voltage and Level values */
  BSP_BC_GetVoltageAndLevel(&Voltage,&BatteryLevel);

  BSP_BC_GetState(&BC_State);

  switch(BC_State.Id) {
    case NotValidInput:
      Status = 0x01; /* Discharging */
    break;
    case VbatLow:
      Status = 0x00; /* Low Battery */
    break;
    case EndOfCharge:
      Status = 0x02; /* End of Charging == Plugged not Charging */
    break;
    case ChargingPhase:
      Status = 0x03; /* Charging */
    break;
    default:
      /* All the Remaing Battery Status */
      Status = 0x04; /* Unknown */
  }

  BLE_BatteryUpdate(BatteryLevel,Voltage,0x8000 /* Current info not available */, Status);
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
    case POWER_BUTTON_PIN:
      PowerButtonPressed = 1;
    break;
    case STBC02_CHG_PIN:
      /* For understanding if it is under charge or not */
      BSP_BC_ChgPinHasToggled();
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
  /* TIM1_CH1 toggling with frequency = 0.5Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + STBOX1_UPDATE_LED_BATTERY));
    if(connected) {
       SendBatteryInfo=1;
    } else {
      BlinkLed =1;
    }
  }

  /* TIM1_CH2 toggling with frequency = 25Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + STBOX1_UPDATE_SENSOR_FUSION));
    SendQuat=1;
  }

#ifdef STBOX1_ENABLE_PRINTF
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + STBOX1_UPDATE_VCOM));
    CDC_PeriodElapsedCallback();
  }
#endif /* STBOX1_ENABLE_PRINTF */

  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + STBOX1_UPDATE_IDWG));
    HAL_IWDG_Refresh(&IwdgHandle);
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

