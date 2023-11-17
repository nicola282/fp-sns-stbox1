/**
  ******************************************************************************
  * @file    BLEMLC\Src\TargetPlatform.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   Initialization of the Target Platform
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

#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"

/* Default program for Machine Learning Core */
#include "lsm6dsox_activity_recognition_for_mobile.h"

/* Default program for Finite State Machine */
#include "lsm6dsox_four_d.h"


/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
TIM_HandleTypeDef    TimCCHandle;
#ifdef STBOX1_ENABLE_PRINTF
USBD_HandleTypeDef USBD_Device;
volatile uint8_t VCOM_RxData;
volatile uint8_t *VCOM_RxBuffer = NULL; /* Pointer to data buffer (received from USB). */
volatile uint32_t VCOM_RxLength = 0;    /* Data length (received from USB). */
uint8_t VComBufferToWrite[256];
int32_t VComBytesToWrite;
#endif /* STBOX1_ENABLE_PRINTF */

/* Custom UCF files for MLC and FSM */
ucf_line_t *MLCCustomUCFFile = NULL;
uint32_t MLCCustomUCFFileLength = 0;
ucf_line_t *FSMCustomUCFFile = NULL;
uint32_t FSMCustomUCFFileLength = 0;

/* Labels for Custom UCF Files form MLC and FSM */
char *MLCCustomLabels = NULL;
char *FSMCustomLabels = NULL;
uint32_t MLCCustomLabelsLength = 0;
uint32_t FSMCustomLabelsLength = 0;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void InitTimers(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
  /* Init Led1/Led2 */
  LedInitTargetPlatform();

  /* Initialize User Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize the Battery Charger */
  BSP_BC_Init();

  /* In order to be able to Read Battery Volt */
  BSP_BC_BatMS_Init();

  /* Initializes the Timers */
  InitTimers();

  MCR_HEART_BIT();
  MCR_HEART_BIT();

#ifdef STBOX1_ENABLE_PRINTF
  BSP_LED_On(LED_BLUE);
  BSP_LED_On(LED_GREEN);

  /* Enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /*** USB CDC Configuration ***/
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add Interface callbacks for CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);

  /* Wait 5 seconds for looking the Initialization phases */
  HAL_Delay(5000);

  BSP_LED_Off(LED_BLUE);
  BSP_LED_Off(LED_GREEN);

#endif /* STBOX1_ENABLE_PRINTF */
}

/** @brief DeInitialize the LSM6SDOX MEMS Sensor
  * @param None
  * @retval None
  */
void DeInit_MEMS(void)
{
  /* DeInit Accelero */
  if (BSP_MOTION_SENSOR_DeInit(LSM6DSOX_0) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Deinit Accelero  Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Deinit Accelero Sensor\n\r");
    Error_Handler();
  }

  /* Disabled all the interrupts for LSM6DSOX */
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);

  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3);
}

/** @brief Initialize the LSM6SDOX MEMS Sensor for Acc/Gyro
  * @param None
  * @retval None
  */
void Init_MEMS(void)
{
  /* Init Accelero */
  if (BSP_MOTION_SENSOR_Init(LSM6DSOX_0, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Init Acc/Gyro  Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Init Acc/Gyro Sensor\n\r");
    Error_Handler();
  }
}

/** @brief Initialize the LSM6SDOX MEMS Sensor for MLC
  * @param uint32_t UseCustomIfAvailableflag for Using or not the Custom UCF file
  * @retval None
  */
void Init_MEMS_MLC(uint32_t UseCustomIfAvailable)
{
  ucf_line_t *ProgramPointer;
  int32_t LineCounter;
  int32_t TotalNumberOfLine;
  int32_t RetValue;

  /* Init Accelero */
  if (BSP_MOTION_SENSOR_Init(LSM6DSOX_0, MOTION_ACCELERO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Init Accelero  Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Init Accelero Sensor\n\r");
    Error_Handler();
  }

  /* Feed the program to Machine Learning Core */
  if ((UseCustomIfAvailable == 1) & (MLCCustomUCFFile != NULL))
  {
    ProgramPointer    = MLCCustomUCFFile;
    TotalNumberOfLine = MLCCustomUCFFileLength;
    STBOX1_PRINTF("-->Custom UCF Program for LSM6DSOX MLC\r\n");
  }
  else
  {
    /* Activity Recognition Default program */
    ProgramPointer = (ucf_line_t *)lsm6dsox_activity_recognition_for_mobile;
    TotalNumberOfLine = sizeof(lsm6dsox_activity_recognition_for_mobile) / sizeof(ucf_line_t);
    STBOX1_PRINTF("-->Activity Recognition for LSM6DSOX MLC\r\n");
    STBOX1_PRINTF("UCF Number Line=%ld\r\n", TotalNumberOfLine);
  }

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++)
  {
    RetValue = BSP_MOTION_SENSOR_Write_Register(LSM6DSOX_0,
                                                ProgramPointer[LineCounter].address,
                                                ProgramPointer[LineCounter].data);
    if (RetValue != BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Error loading the Program to LSM6DSOX [%ld]->%lx\n\r", LineCounter, RetValue);
      Error_Handler();
    }
  }

  STBOX1_PRINTF("Program loaded inside the LSM6DSOX MLC\n\r");

  {
    /* Enable interrupts from INT1 LSM6DSOX  */
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    STBOX1_PRINTF("Enabled LSM6DSOX INT1 Detection \n\r");
  }

  {
    /* Enable interrupts from INT2 LSM6DSOX  */
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    STBOX1_PRINTF("Enabled LSM6DSOX INT2 Detection \n\r");
  }
}


/** @brief Initialize the LSM6SDOX MEMS Sensor for FSM
  * @param uint32_t UseCustomIfAvailable flag for Using or not the Custom UCF file
  * @retval None
  */
void Init_MEMS_FSM(uint32_t UseCustomIfAvailable)
{
  ucf_line_t *ProgramPointer;
  int32_t LineCounter;
  int32_t TotalNumberOfLine;
  int32_t RetValue;

  /* Init Accelero */
  if (BSP_MOTION_SENSOR_Init(LSM6DSOX_0, MOTION_ACCELERO) == BSP_ERROR_NONE)
  {
    STBOX1_PRINTF("OK Init Accelero  Sensor\n\r");
  }
  else
  {
    STBOX1_PRINTF("Error Init Accelero Sensor\n\r");
    Error_Handler();
  }

  /* Feed the program to FiniteStateMachine */
  if ((UseCustomIfAvailable == 1) & (FSMCustomUCFFile != NULL))
  {
    ProgramPointer    = FSMCustomUCFFile;
    TotalNumberOfLine = FSMCustomUCFFileLength;
    STBOX1_PRINTF("-->Custom UCF Program for LSM6DSOX FSM\r\n");
  }
  else
  {
    /* 4D position recognition Default program */
    ProgramPointer = (ucf_line_t *)lsm6dsox_four_d;
    TotalNumberOfLine = sizeof(lsm6dsox_four_d) / sizeof(ucf_line_t);
    STBOX1_PRINTF("-->4D position recognition for LSM6DSOX FSM\r\n");
    STBOX1_PRINTF("UCF Number Line=%ld\r\n", TotalNumberOfLine);
  }

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++)
  {
    RetValue = BSP_MOTION_SENSOR_Write_Register(LSM6DSOX_0,
                                                ProgramPointer[LineCounter].address,
                                                ProgramPointer[LineCounter].data);
    if (RetValue != BSP_ERROR_NONE)
    {
      STBOX1_PRINTF("Error loading the Program to LSM6DSOX [%ld]->%lx\n\r", LineCounter, RetValue);
      Error_Handler();
    }
  }

  STBOX1_PRINTF("Program loaded inside the LSM6DSOX FSM\n\r");

  {
    /* Enable interrupts from INT1 LSM6DSOX  */
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    STBOX1_PRINTF("Enabled LSM6DSOX INT1 Detection \n\r");
  }

  {
    /* Enable interrupts from INT2 LSM6DSOX  */
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    STBOX1_PRINTF("Enabled LSM6DSOX INT2 Detection \n\r");
  }
}

/**
  * @brief  Function for initializing timers:
  *  - 1 for sending Battery Information
  *  - 1 for Led Blinking
  * @param  None
  * @retval None
  */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 10000) - 1);

  /* Set TIM1 instance */
  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if (HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Output Compare channels */

  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Output Compare Toggle Mode configuration:
   * Channel 1 for LED Blinking */
  sConfig.Pulse = STBOX1_UPDATE_LED;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Output Compare Toggle Mode configuration:
   * Channel 2 for sending Battery information */
  sConfig.Pulse = STBOX1_UPDATE_BATTERY;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

#ifdef STBOX1_ENABLE_PRINTF
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = STBOX1_UPDATE_VCOM;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
#endif /* STBOX1_ENABLE_PRINTF */

  /* Output Compare Toggle Mode configuration:
   * Channel 4 for sending Acc/Gyro Values */
  sConfig.Pulse = STBOX1_UPDATE_INV;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On(LED_BLUE);
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED_BLUE);
}

/** @brief  This function toggles the LED
  * @param  None
  * @retval None
  */
void LedToggleTargetPlatform(void)
{
  BSP_LED_Toggle(LED_BLUE);
}

/** @brief  This function initializes the LED
  * @param  None
  * @retval None
  */
void LedInitTargetPlatform(void)
{
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
}

#ifdef STBOX1_ENABLE_PRINTF
/**
  * @brief  Read from VCOM
  * @param  char *buffer Pointer to buffer.
  * @param  uint32_t len_maxData max. length.
  * @retval Number of really read data bytes.
  */
uint32_t VCOM_read(char *buffer, uint32_t len_max)
{
  /* VCOM data receive not completed or no VCOM data received at all. */
  if (VCOM_RxData == 0)
  {
    return 0;
  }

  /* ERROR: No VCOM data ready. */
  if (VCOM_RxLength == 0 || VCOM_RxBuffer == NULL)
  {
    Error_Handler();
  }

  /* Read all data */
  if (VCOM_RxLength <= len_max)
  {
    uint32_t len = VCOM_RxLength;
    memcpy((uint8_t *)buffer, (uint8_t *)VCOM_RxBuffer, len);

    VCOM_RxData   = 0;
    VCOM_RxBuffer = NULL;
    VCOM_RxLength = 0;

    CDC_Next_Packet_Rx();
    return len;
  }
  else
  {
    /* Read part of data that fits into buffer. */
    memcpy((uint8_t *)buffer, (uint8_t *)VCOM_RxBuffer, len_max);

    VCOM_RxBuffer += len_max;
    VCOM_RxLength -= len_max;

    return len_max;
  }
}
#endif /* define STBOX1_ENABLE_PRINTF */
