/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_nfc_ftm.c
  * @author  System Research & Applications Team - Catania Lab.
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
 * @page NFC_FTM NFC Fast Transfer Memory protocol
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

 * The Example application provides one example on how to use the Fast Memory Transfer protocol for making a firmware update using the NFC and using
 * the STMicroelectronics â€œST25 NFC Tapâ€? available for Android and iOS:
 *   - https://play.google.com/store/apps/details?id=com.st.st25nfc
 *   - https://apps.apple.com/be/app/nfc-tap/id1278913597
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "app_nfc_ftm.h"
#include "STBOX1_config.h"
#include "steval_mkboxpro.h"
#include "SensorTileBoxPro_nfctag.h"
#include "SensorTileBoxPro_nfctag_ex.h"
#include "st25ftm_config.h"
#include "st25ftm_process.h"

#ifdef STBOX1_ENABLE_START_BIP
  #include "note.h"
#endif /* STBOX1_ENABLE_START_BIP */

/* Exported variables --------------------------------------------------------*/
int32_t CurrentActiveBank = 0;

FinishGood_TypeDef FinishGood;

/* Imported variables --------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#ifdef STBOX1_ENABLE_START_BIP
  #define MCR_START_HEART_BIT()   \
  {                         \
    BSP_LED_On(LED_YELLOW); \
    BSP_LED_On(LED_GREEN);  \
    beep(NOTE_A6, 50);      \
    HAL_Delay(150);         \
    BSP_LED_Off(LED_YELLOW);\
    BSP_LED_Off(LED_GREEN); \
    HAL_Delay(400);         \
    BSP_LED_On(LED_YELLOW); \
    BSP_LED_On(LED_GREEN);  \
    beep(NOTE_G6, 50);      \
    HAL_Delay(150);         \
    BSP_LED_Off(LED_YELLOW);\
    BSP_LED_Off(LED_GREEN); \
    HAL_Delay(400);         \
    beep(NOTE_F6, 50);      \
  }

  #define MCR_START_HEART_BIT2()  \
  {                         \
    BSP_LED_On(LED_YELLOW); \
    BSP_LED_On(LED_RED);    \
    beep(NOTE_F6, 50);      \
    HAL_Delay(150);         \
    BSP_LED_Off(LED_YELLOW);\
    BSP_LED_Off(LED_RED);   \
    HAL_Delay(400);         \
    BSP_LED_On(LED_YELLOW); \
    BSP_LED_On(LED_RED);    \
    beep(NOTE_G6, 50);      \
    HAL_Delay(150);         \
    BSP_LED_Off(LED_YELLOW);\
    BSP_LED_Off(LED_RED);   \
    HAL_Delay(400);         \
    beep(NOTE_A6, 50);      \
  }
#else /* STBOX1_ENABLE_START_BIP */
  #define MCR_START_HEART_BIT MCR_HEART_BIT
  #define MCR_START_HEART_BIT2 MCR_HEART_BIT2
#endif /* STBOX1_ENABLE_START_BIP */

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t user_button_pressed = 0;
static int32_t PushButtonState = GPIO_PIN_RESET;
/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void PrintInfo(void);

#ifdef STBOX1_ENABLE_START_BIP
  static void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
  static void MX_TIMER_Init(void);
  static void beep(uint32_t freq, uint16_t time);
#endif /* STBOX1_ENABLE_START_BIP */

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval none
  */
void MX_NFC_FTM_Init(void)
{
  User_Init();

  STBOX1_PRINTF("\033[2J"); /* serial console clear screen */
  STBOX1_PRINTF("\033[H");  /* serial console cursor to home */
  PrintInfo();

  BSP_LED_On(LED_BLUE);

  /* Init ST25DV driver */
  while( BSP_NFCTAG_Init(BSP_NFCTAG_INSTANCE) != NFCTAG_OK );

  /* Setting the New Password for I2C protection */
  if(BSP_NFCTAG_ChangeI2CPassword(STBOX1_MSB_PASSWORD,STBOX1_LSB_PASSWORD)!=NFCTAG_OK ) {
    STBOX1_PRINTF("Error NFC Changing the I2C password\r\n");
    STBOX1_Error_Handler(STBOX1_ERROR_NFC,__FILE__,__LINE__);
  } else {
    STBOX1_PRINTF("NFC Changed the I2C password\r\n");
  }

  /* Enable the NFC GPIO Interrupt */
  if(BSP_NFCTAG_GPIO_Init()!=BSP_ERROR_NONE) {
    STBOX1_PRINTF("Error NFC Initializing GPIO\r\n");
    STBOX1_Error_Handler(STBOX1_ERROR_NFC,__FILE__,__LINE__);
  } else {
    STBOX1_PRINTF("NFC NFC GPIO Initialized\r\n");
  }

  BSP_LED_Off(LED_BLUE);

  STBOX1_PRINTF("Press User Button for Enabling/Disabling FTM\r\n");
}

/*
 * FP-SNS-STBOX1 background task
 */
void MX_NFC_FTM_Process(void)
{
    static uint32_t MailBoxEnabled=0;

    /* FTM Work Fuction*/
    if(MailBoxEnabled==1) {
      FTMManagement();
    }

    /* Handle the user button */
    if(user_button_pressed) {

      /* Debouncing */
      HAL_Delay(50);

      /* Wait until the button is released */
      while ((BSP_PB_GetState( BUTTON_KEY ) == PushButtonState));

      /* Debouncing */
      HAL_Delay(50);

      /* Reset Interrupt flag */
      user_button_pressed=0;

      if(MailBoxEnabled) {
        MailBoxEnabled=0;
        FTMManagementDeInit();
      } else {
        MailBoxEnabled =1;
        FTMManagementInit();
      }
    }

    /* Wait next event */
    __WFI();
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
  /* Check what is the Push Button State when the button is not pressed. It can change across families */
  PushButtonState = (BSP_PB_GetState(BUTTON_KEY)) ?  0 : 1;

  /* Init the Led */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_BLUE);

  /* why RED is activated by default? */
  BSP_LED_Off(LED_RED);

#ifdef STBOX1_ENABLE_START_BIP
  MX_TIMER_Init();
#endif /* STBOX1_ENABLE_START_BIP */

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
      MCR_START_HEART_BIT2();
    } else {
      CurrentActiveBank= 1;
      MCR_START_HEART_BIT();
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
  }

#ifdef STBOX1_ENABLE_START_BIP
  /* Reset the Timer Used for making the Beep*/
  #define SPKR_Pin GPIO_PIN_13
  #define SPKR_GPIO_Port GPIOE
  HAL_GPIO_DeInit(SPKR_GPIO_Port,SPKR_Pin);
  NFC_FTM_TIMx_FORCE_RESET();
  HAL_Delay(10);
  NFC_FTM_TIMx_RELEASE_RESET();
#endif /* STBOX1_ENABLE_START_BIP */

  BSP_COM_Init(COM1);

  /* Check the board Type */
  FinishGood = BSP_CheckFinishGood();
}

#ifdef STBOX1_ENABLE_START_BIP
  #define SPKR_Pin GPIO_PIN_13
  #define SPKR_GPIO_Port GPIOE

  static void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim->Instance==TIM_CC_INSTANCE)
    {

    /* USER CODE BEGIN TIM1_MspPostInit 0 */
      __HAL_RCC_GPIOE_CLK_ENABLE();

    /* USER CODE END TIM1_MspPostInit 0 */
      NFC_FTM_TIMx_CLK_ENABLE();
      /**TIM1 GPIO Configuration
      PE13     ------> TIM1_CH3
      */
      GPIO_InitStruct.Pin = SPKR_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = NFC_FTM_GPIO_AF1_TIMx;
      HAL_GPIO_Init(SPKR_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM1_MspPostInit 1 */

    /* USER CODE END TIM1_MspPostInit 1 */
    }

  }

  /**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
  static void MX_TIMER_Init(void)
  {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    TIM_CC_HANDLE.Instance = TIM_CC_INSTANCE;
    TIM_CC_HANDLE.Init.Prescaler = 1000;
    TIM_CC_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_CC_HANDLE.Init.Period = 65535;
    TIM_CC_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_CC_HANDLE.Init.RepetitionCounter = 0;
    TIM_CC_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TIM_CC_HANDLE) != HAL_OK)
    {
      while(1);
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&TIM_CC_HANDLE, &sClockSourceConfig) != HAL_OK)
    {
      while(1);
    }
    if (HAL_TIM_PWM_Init(&TIM_CC_HANDLE) != HAL_OK)
    {
      while(1);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&TIM_CC_HANDLE, &sMasterConfig) != HAL_OK)
    {
      while(1);
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&TIM_CC_HANDLE, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      while(1);
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&TIM_CC_HANDLE, &sBreakDeadTimeConfig) != HAL_OK)
    {
     while(1);
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&TIM_CC_HANDLE);

  }

  static void beep(uint32_t freq, uint16_t time)
  {
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint16_t period = 1000000/freq-1;
    uint16_t volume = period/16; // 6,25% DC

    if(freq==0){
      HAL_TIM_PWM_Stop(&TIM_CC_HANDLE, TIM_CHANNEL_3);
      return; //speaker off
    }

    TIM_CC_HANDLE.Instance = TIM_CC_INSTANCE;
    TIM_CC_HANDLE.Init.Prescaler = 63; //because 64MHz of timer clock, to have 1MHz of clock freq
    TIM_CC_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_CC_HANDLE.Init.Period = period;
    TIM_CC_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_CC_HANDLE.Init.RepetitionCounter = 0;
    TIM_CC_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&TIM_CC_HANDLE) != HAL_OK)
    {
     while(1);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = volume;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&TIM_CC_HANDLE, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM_CC_HANDLE, TIM_CHANNEL_3);
    if(time){ //don't stop beep if time=0
      HAL_Delay(time);
      HAL_TIM_PWM_Stop(&TIM_CC_HANDLE, TIM_CHANNEL_3);
    }
  }
#endif /* STBOX1_ENABLE_START_BIP */

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
 * @brief  BSP NFCTAG GPIO callback
 * @param  Node
 * @retval None.
 */
void BSP_NFCTAG_GPIO_Callback(void)
{
  /* Set the GPIO flag */
  GPO_Activated=1;
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
            "\tVersion %d.%d.%d\r\n"
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

