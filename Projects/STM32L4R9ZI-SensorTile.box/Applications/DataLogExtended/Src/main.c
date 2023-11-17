/**
  ******************************************************************************
  * @file    DataLogExtended\Src\main.c
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
  * @page DataLogExtended For visualizing Sensor's data with Unicleo-GUI
  *
  * @image html st_logo.png
  *
  * <b>Introduction</b>
  *
  * Main function is to show how to use SensorTile.box to send data using USB
  * to a connected PC or Desktop and display it on specific application Unicleo-GUI V1.11.0,
  * which is developed by STMicroelectronics and provided separately, not in binary with this package.
  * After connection has been established:
  * - the user can view the data from various on-board environment sensors like Temperature,
  * Humidity, and Pressure.
  * - the user can also view data from various on-board MEMS sensors as well like Accelerometer,
  * Gyroscope, and Magnetometer.
  * - the user can also visualize this data as graphs using Unicleo application.
  *
  */

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "vcom.h"
#include "demo_serial.h"
#include "sensor_commands.h"
    
/** @addtogroup SENSORTILE_Examples
  * @{
  */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s 
{
  int8_t   sign;     /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

#define MCR_HEART_BIT()  \
{                        \
  BSP_LED_On(LED_BLUE);  \
  BSP_LED_On(LED_GREEN); \
  HAL_Delay(200);        \
  BSP_LED_Off(LED_BLUE); \
  BSP_LED_Off(LED_GREEN);\
  HAL_Delay(400);        \
  BSP_LED_On(LED_BLUE);  \
  BSP_LED_On(LED_GREEN); \
  HAL_Delay(200);        \
  BSP_LED_Off(LED_BLUE); \
  BSP_LED_Off(LED_GREEN);\
  HAL_Delay(1000);       \
}

/* Public variables ----------------------------------------------------------*/
uint8_t UpdateInterrupt = 0;
uint32_t SensorsEnabled = 0;
uint32_t StartTime = 0;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static char DataOut[MAX_BUF_SIZE];
static RTC_HandleTypeDef RtcHandle;
static volatile uint8_t TerminalActive = 0;
static volatile uint32_t IntCurrentTime1 = 0;
static volatile uint32_t IntCurrentTime2 = 0;
static uint32_t Previous_Sensors_Enabled = 0;
static uint8_t NewData = 0;
static uint16_t NewDataFlags = 0;
static uint8_t MsgIndex = 0;
static uint32_t LastUpdate = 0;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Init(void);

static void RTC_Config(void);
static void RTC_TimeStampConfig(void);

static void Initialize_All_Sensors(void);
static void Enable_Disable_Sensors(void);
static void Float_To_Int(float In, displayFloatToInt_t *OutValue, int32_t DecPrec);

static void Time_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Press_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Hum_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Temp_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Sensors_Interrupt_Handler(TMsg *Msg);
static void MLC_Handler(TMsg *Msg);
static void FSM_Handler(TMsg *Msg);

static uint32_t DWT_Delay_Init(void);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Main function is to show how to SensorTile.box to send data
  *         using a connected PC or Desktop and display it on generic applications like
  *         TeraTerm and specific application like Unicleo GUI, which is developed by STMicroelectronics
  *         and provided with a separated package.
  *
  *         After connection has been established:
  *         - the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure
  *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyrometer, and Magnetometer
  *         - the user can also visualize this data as graphs using Sensors_DataLog application provided with this package
  *
  * @param  None
  * @retval None
  */
int main(void)
{
  TMsg msg_dat;
  TMsg msg_cmd;

  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LEDs */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);

  MCR_HEART_BIT();

  BSP_LED_On(LED_GREEN);

  /* Initialize GPIO */
  GPIO_Init();
  
  /* Initialize button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize VCOM (Virtual COM Port) */
  VCOM_init();

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  
  /* Initialize cycle counter */
  DWT_Delay_Init();

  /* Initialize sensors */
  Initialize_All_Sensors();

  for (;;)
  {
    /* Process incomming messages */
    if (VCOM_receive_MSG(&msg_cmd, TM_SG_MAX_LEN) == OK)
    {
      if (msg_cmd.Data[0] == DEV_ADDR)
      {
        HandleMSG((TMsg *)&msg_cmd);
        if (DataLoggerActive)
        {
          TerminalActive = 0;
        }
      }
    }

    /* Enable/Disable sensors */
    if (Previous_Sensors_Enabled != SensorsEnabled)
    {
      Previous_Sensors_Enabled = SensorsEnabled;
      Enable_Disable_Sensors();
    }
    
    /* For terminal output enable all sensors */
    if (TerminalActive == 1U)
    {
      SensorsEnabled = 0xFFFFFFFF;
    }
    
    /* LED control */
    if ((DataLoggerActive != 0U) || (TerminalActive != 0))
    {
      BSP_LED_Toggle(LED_BLUE);
    }
    else
    {
      BSP_LED_Off(LED_BLUE);
    }

    /* Read data from enable sensors and prepare data stream */
    if ((DataLoggerActive != 0U) || (TerminalActive != 0))
    {  
      Time_Handler(&msg_dat);
      
      if ((SensorsEnabled & PRESSURE_SENSOR_ENABLED) == PRESSURE_SENSOR_ENABLED)
      {
        Press_Sensor_Handler(&msg_dat, PrsInstance);
      }
      
      if ((SensorsEnabled & TEMPERATURE_SENSOR_ENABLED) == TEMPERATURE_SENSOR_ENABLED)
      {
        Temp_Sensor_Handler(&msg_dat, TmpInstance);
      }
      
      if ((SensorsEnabled & HUMIDITY_SENSOR_ENABLED) == HUMIDITY_SENSOR_ENABLED)
      {
        Hum_Sensor_Handler(&msg_dat, HumInstance);
      }

      if ((SensorsEnabled & ACCELEROMETER_SENSOR_ENABLED) == ACCELEROMETER_SENSOR_ENABLED)
      {
        Accelero_Sensor_Handler(&msg_dat, AccInstance);
      }
      
      if ((SensorsEnabled & GYROSCOPE_SENSOR_ENABLED) == GYROSCOPE_SENSOR_ENABLED)
      {
        Gyro_Sensor_Handler(&msg_dat, GyrInstance);
      }
      
      if ((SensorsEnabled & MAGNETIC_SENSOR_ENABLED) == MAGNETIC_SENSOR_ENABLED)
      {
        Magneto_Sensor_Handler(&msg_dat, MagInstance);
      }
      
      if ((SensorsEnabled & INTERRUPTS_ENABLED) == INTERRUPTS_ENABLED)
      {
        Sensors_Interrupt_Handler(&msg_dat);
      }

      if ((SensorsEnabled & MLC_ENABLED) == MLC_ENABLED)
      {
        MLC_Handler(&msg_dat);
      }

      if ((SensorsEnabled & FSM_ENABLED) == FSM_ENABLED)
      {
        FSM_Handler(&msg_dat);
      }
    }

    /* Send data stream in a new data are available */
    if (DataLoggerActive != 0U)
    {
      if (NewData != 0)
      {
        INIT_STREAMING_HEADER(&msg_dat);
        (void)memcpy(&msg_dat.Data[7], (void *)&NewDataFlags, 2);
        msg_dat.Len = MsgIndex;
        VCOM_send_MSG(&msg_dat);
        NewData = 0;
        NewDataFlags = 0;
      }
    }
    
    /* For terminal output reduce speed */
    if (TerminalActive)
    {
      HAL_Delay(500);
    }
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Interrupt pins SensorTile.Box */
  // LSM6DSOX INT1                      PA2
  // LSM6DSOX INT2                      PE3
  // LPS22HH INT                        PD15
  // LIS2DW12 INT1                      PC5
  // LIS2DW12 INT2                      PD14
  // LIS3DHH INT1                       PC13
  // LIS3DHH INT2                       PE6
  // LIS2MDL INT                        PD12
  // HTS221                             PD13
  // STTS751                            PG5
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Configure GPIO pins : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /* Configure GPIO pins : PC5 PC13*/
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /* Configure GPIO pins : PE3 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /* Configure GPIO pins : PG5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/**
  * @brief  Initialize all sensors
  * @param  None
  * @retval None
  */
static void Initialize_All_Sensors(void)
{
  (void)BSP_MOTION_SENSOR_Init(AccInstance, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Init(GyrInstance, MOTION_GYRO);
  (void)BSP_MOTION_SENSOR_Init(MagInstance, MOTION_MAGNETO);
  (void)BSP_ENV_SENSOR_Init(HumInstance, ENV_HUMIDITY);
  (void)BSP_ENV_SENSOR_Init(TmpInstance, ENV_TEMPERATURE);
  (void)BSP_ENV_SENSOR_Init(PrsInstance, ENV_PRESSURE);
}

/**
 * @brief  Enable/disable desired sensors
 * @param  None
 * @retval None
 */
static void Enable_Disable_Sensors(void)
{
  if ((SensorsEnabled & ACCELEROMETER_SENSOR_ENABLED) == ACCELEROMETER_SENSOR_ENABLED)
  {
    (void)BSP_MOTION_SENSOR_Enable(AccInstance, MOTION_ACCELERO);
  }
  else
  {
    (void)BSP_MOTION_SENSOR_Disable(AccInstance, MOTION_ACCELERO);
  }
  
  if ((SensorsEnabled & GYROSCOPE_SENSOR_ENABLED) == GYROSCOPE_SENSOR_ENABLED)
  {
    (void)BSP_MOTION_SENSOR_Enable(GyrInstance, MOTION_GYRO);
  }
  else
  {
    (void)BSP_MOTION_SENSOR_Disable(GyrInstance, MOTION_GYRO);
  }
  
  if ((SensorsEnabled & MAGNETIC_SENSOR_ENABLED) == MAGNETIC_SENSOR_ENABLED)
  {
    (void)BSP_MOTION_SENSOR_Enable(MagInstance, MOTION_MAGNETO);
  }
  else
  {
    (void)BSP_MOTION_SENSOR_Disable(MagInstance, MOTION_MAGNETO);
  }
  
  if ((SensorsEnabled & HUMIDITY_SENSOR_ENABLED) == HUMIDITY_SENSOR_ENABLED)
  {
    (void)BSP_ENV_SENSOR_Enable(HumInstance, ENV_HUMIDITY);
  }
  else
  {
    (void)BSP_ENV_SENSOR_Disable(HumInstance, ENV_HUMIDITY);
  }
  
  if ((SensorsEnabled & TEMPERATURE_SENSOR_ENABLED) == TEMPERATURE_SENSOR_ENABLED)
  {
    (void)BSP_ENV_SENSOR_Enable(TmpInstance, ENV_TEMPERATURE);
  }
  else
  {
    (void)BSP_ENV_SENSOR_Disable(TmpInstance, ENV_TEMPERATURE);
  }
  
  if ((SensorsEnabled & PRESSURE_SENSOR_ENABLED) == PRESSURE_SENSOR_ENABLED)
  {
    (void)BSP_ENV_SENSOR_Enable(PrsInstance, ENV_PRESSURE);
  }
  else
  {
    (void)BSP_ENV_SENSOR_Disable(PrsInstance, ENV_PRESSURE);
  }
}

/**
  * @brief  Splits a float into two integer values.
  * @param  in the float value as input
  * @param  out_value the pointer to the output integer structure
  * @param  dec_prec the decimal precision to be used
  * @retval None
  */
static void Float_To_Int(float in, displayFloatToInt_t *out_value, int32_t dec_prec)
{
  if(in >= 0.0f)
  {
    out_value->sign = 0;
  }
  else
  {
    out_value->sign = 1;
    in = -in;
  }

  out_value->out_int = (int32_t)in;
  in = in - (float)(out_value->out_int);
  out_value->out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
 * @brief  Handles the precise time
 * @param  Msg the time part of the stream
 * @retval None
 */
static void Time_Handler(TMsg *Msg)
{
  uint32_t time_us;

  if (DataLoggerActive != 0U)
  {
    time_us = DWT_GetTickUS() - StartTime;
    Serialize(&Msg->Data[3], time_us, 4);
    MsgIndex = 9;
  }
  else if (TerminalActive != 0U)
  {
    time_us = DWT_GetTickUS() - StartTime;
    (void)snprintf(DataOut, MAX_BUF_SIZE, "TimeStamp: %ld\r\n", time_us);
    (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
  }
  else
  {
    /* Nothing to do */
  }
}

/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg the PRESSURE part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Press_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float pressure;
  uint8_t status = 0;

  if (BSP_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_PRESSURE, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 1U;

    (void)BSP_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure);

    if (DataLoggerActive != 0U)
    {
      (void)memcpy(&Msg->Data[MsgIndex], (void *)&pressure, sizeof(float));
      MsgIndex = MsgIndex + 4;
    }
    else if (TerminalActive != 0U)
    {
      displayFloatToInt_t out_value;
      Float_To_Int(pressure, &out_value, 2);
      (void)snprintf(DataOut, MAX_BUF_SIZE, "PRESS: %d.%02d\r\n", (int)out_value.out_int, (int)out_value.out_dec);
      (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg the TEMPERATURE part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Temp_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float temperature;
  uint8_t status = 0;
  uint8_t drdy = 0;
  static uint8_t stts751_is_busy = 0;

  if (Instance == STTS751_0)
  {
    if (BSP_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_TEMPERATURE, &status) == BSP_ERROR_NONE)
    {
      if (status == 0)
      {
        stts751_is_busy = 1;
        drdy = 0;
      }
      else
      {
        if (stts751_is_busy == 1)
        {
          stts751_is_busy = 0;
          drdy = 1;
        }
      }
    }
  }
  else
  {
    if (BSP_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_TEMPERATURE, &status) == BSP_ERROR_NONE && status == 1U)
    {
      drdy = 1;
    }
    else
    {
      drdy = 0;
    }
  }

  if (drdy == 1)
  {
    NewData++;
    NewDataFlags |= 2U;

    (void)BSP_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature);

    if (DataLoggerActive != 0U)
    {
      (void)memcpy(&Msg->Data[MsgIndex], (void *)&temperature, sizeof(float));
      MsgIndex = MsgIndex + 4;
    }
    else if (TerminalActive != 0U)
    {
      displayFloatToInt_t out_value;
      Float_To_Int(temperature, &out_value, 2);
      (void)snprintf(DataOut, MAX_BUF_SIZE, "TEMP: %c%d.%02d\r\n", ((out_value.sign != 0) ? '-' : '+'),
                     (int)out_value.out_int, (int)out_value.out_dec);
      (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg the HUMIDITY part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Hum_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float humidity;
  uint8_t status = 0;

  if (BSP_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_HUMIDITY, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 4U;

    (void)BSP_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity);

    if (DataLoggerActive != 0U)
    {
      (void)memcpy(&Msg->Data[MsgIndex], (void *)&humidity, sizeof(float));
      MsgIndex = MsgIndex + 4;
    }
    else if (TerminalActive != 0U)
    {
      displayFloatToInt_t out_value;
      Float_To_Int(humidity, &out_value, 2);
      (void)snprintf(DataOut, MAX_BUF_SIZE, "HUM: %d.%02d\r\n", (int)out_value.out_int, (int)out_value.out_dec);
      (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg the ACCELERO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  int32_t data[6];
  BSP_MOTION_SENSOR_Axes_t acceleration;
  uint8_t status = 0;

  if (BSP_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_ACCELERO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 8U;

    (void)BSP_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration);

    if (DataLoggerActive != 0U)
    {
      Serialize_s32(&Msg->Data[MsgIndex + 0], acceleration.x, 4);
      Serialize_s32(&Msg->Data[MsgIndex + 4], acceleration.y, 4);
      Serialize_s32(&Msg->Data[MsgIndex + 8], acceleration.z, 4);
      MsgIndex = MsgIndex + 12;
    }
    else if (TerminalActive != 0U)
    {
      data[0] = acceleration.x;
      data[1] = acceleration.y;
      data[2] = acceleration.z;

      (void)snprintf(DataOut, MAX_BUF_SIZE, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
      (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg the GYRO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  int32_t data[6];
  BSP_MOTION_SENSOR_Axes_t angular_velocity;
  uint8_t status = 0;

  if (BSP_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_GYRO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 16U;

    (void)BSP_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity);

    if (DataLoggerActive != 0U)
    {
      Serialize_s32(&Msg->Data[MsgIndex + 0], angular_velocity.x, 4);
      Serialize_s32(&Msg->Data[MsgIndex + 4], angular_velocity.y, 4);
      Serialize_s32(&Msg->Data[MsgIndex + 8], angular_velocity.z, 4);
      MsgIndex = MsgIndex + 12;
    }
    else if (TerminalActive != 0U)
    {
      data[0] = angular_velocity.x;
      data[1] = angular_velocity.y;
      data[2] = angular_velocity.z;

      (void)snprintf(DataOut, MAX_BUF_SIZE, "GYR_X: %d, GYR_Y: %d, GYR_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
      (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg the MAGNETO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  int32_t data[3];
  BSP_MOTION_SENSOR_Axes_t magnetic_field;
  uint8_t status = 0;

  if (BSP_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_MAGNETO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 32U;

    (void)BSP_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field);

    if (DataLoggerActive != 0U)
    {
      Serialize_s32(&Msg->Data[MsgIndex + 0], (int32_t)magnetic_field.x, 4);
      Serialize_s32(&Msg->Data[MsgIndex + 4], (int32_t)magnetic_field.y, 4);
      Serialize_s32(&Msg->Data[MsgIndex + 8], (int32_t)magnetic_field.z, 4);
      MsgIndex = MsgIndex + 12;
    }
    else if (TerminalActive != 0U)
    {
      data[0] = magnetic_field.x;
      data[1] = magnetic_field.y;
      data[2] = magnetic_field.z;

      (void)snprintf(DataOut, MAX_BUF_SIZE, "MAG_X: %d, MAG_Y: %d, MAG_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
      (void)VCOM_write(DataOut, (uint16_t)strlen(DataOut));
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the sensors interrupts
 * @param  Msg the INTERRUPT part of the stream
 * @retval None
 */
static void Sensors_Interrupt_Handler(TMsg *Msg)
{
  uint8_t int_status = 0;
  static uint8_t mem_int_status = 0;

  /* Interrupt pins SensorTile.Box */
  // Bit 0: LSM6DSOX INT1 (PA2)
  // Bit 1: LSM6DSOX INT2 (PE3)
  // Bit 2: LPS22HH INT (PD15)
  // Bit 3: LIS2DW12 INT1 (PC5) / LIS3DHH INT1 (PC13)
  // Bit 4: LIS2DW12 INT2 (PD14) / LIS3DHH INT2 (PE6)
  // Bit 5: LIS2MDL INT (PD12)
  // Bit 6: HTS221 (PD13)
  // Bit 7: STTS751 (PG5)
  
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) int_status |= (1 << 0); else int_status &= ~(1 << 0);
  if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET) int_status |= (1 << 1); else int_status &= ~(1 << 1);
  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET) int_status |= (1 << 2); else int_status &= ~(1 << 2);
  
  if (AccInstance == LIS2DW12_0)
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) int_status |= (1 << 3); else int_status &= ~(1 << 3);
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET) int_status |= (1 << 4); else int_status &= ~(1 << 4);
  }
  else if (AccInstance == LIS3DHH_0)
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) int_status |= (1 << 3); else int_status &= ~(1 << 3);
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET) int_status |= (1 << 4); else int_status &= ~(1 << 4);
  }
  else
  {
    int_status &= ~(1 << 3);
    int_status &= ~(1 << 4);
  }
  
  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) int_status |= (1 << 5); else int_status &= ~(1 << 5);
  if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) int_status |= (1 << 6); else int_status &= ~(1 << 6);
  if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_5) == GPIO_PIN_SET) int_status |= (1 << 7); else int_status &= ~(1 << 7);

  if ((mem_int_status != int_status) || (UpdateInterrupt == 1) || ((HAL_GetTick() - LastUpdate) > 100))
  {
    UpdateInterrupt = 0;
    NewData++;
    NewDataFlags |= 64U;
    Msg->Data[MsgIndex] = int_status;
    MsgIndex = MsgIndex + 1;
    
    mem_int_status = int_status;
    LastUpdate = HAL_GetTick();
  }
}

/**
 * @brief  Handles the MLC status data
 * @param  Msg the MLC part of the stream
 * @retval None
 */
static void MLC_Handler(TMsg *Msg)
{
  int32_t i = 0;
  uint8_t new_status = 0;
  uint8_t mlc_status[8];
  static uint8_t mem_mlc_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  if ((AccInstance == LSM6DSOX_0) && (GyrInstance == LSM6DSOX_0))
  {
    (void)BSP_MOTION_SENSOR_Write_Register(LSM6DSOX_0, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_EMBEDDED_FUNC_BANK << 6);

    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC0_SRC, &mlc_status[0]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC1_SRC, &mlc_status[1]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC2_SRC, &mlc_status[2]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC3_SRC, &mlc_status[3]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC4_SRC, &mlc_status[4]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC5_SRC, &mlc_status[5]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC6_SRC, &mlc_status[6]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_MLC7_SRC, &mlc_status[7]);

    (void)BSP_MOTION_SENSOR_Write_Register(LSM6DSOX_0, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_USER_BANK << 6);
  }
    else
  {
    mlc_status[0] = 0;
    mlc_status[1] = 0;
    mlc_status[2] = 0;
    mlc_status[3] = 0;
    mlc_status[4] = 0;
    mlc_status[5] = 0;
    mlc_status[6] = 0;
    mlc_status[7] = 0;
  }
  
  for (i = 0; i < 8; i++)
  {
    if (mlc_status[i] != mem_mlc_status[i])
    {
      new_status = 1;
      break;
    }
  }

  if (new_status == 1)
  {
    memcpy((void *)&Msg->Data[MsgIndex], (void *)&mlc_status, 8);
    memcpy((void *)&mem_mlc_status, (void *)mlc_status, 8);
    MsgIndex = MsgIndex + 8;

    NewData++;
    NewDataFlags |= 128U;
  }
}

/**
 * @brief  Handles the FSM status data
 * @param  Msg the FSM part of the stream
 * @retval None
 */
static void FSM_Handler(TMsg *Msg)
{
  int32_t i = 0;
  uint8_t new_status = 0;
  uint8_t fsm_status[16];
  static uint8_t mem_fsm_status[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  if ((AccInstance == LSM6DSOX_0) && (GyrInstance == LSM6DSOX_0))
  {
    (void)BSP_MOTION_SENSOR_Write_Register(LSM6DSOX_0, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_EMBEDDED_FUNC_BANK << 6);

    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS1, &fsm_status[0]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS2, &fsm_status[1]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS3, &fsm_status[2]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS4, &fsm_status[3]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS5, &fsm_status[4]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS6, &fsm_status[5]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS7, &fsm_status[6]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS8, &fsm_status[7]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS9, &fsm_status[8]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS10, &fsm_status[9]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS11, &fsm_status[10]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS12, &fsm_status[11]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS13, &fsm_status[12]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS14, &fsm_status[13]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS15, &fsm_status[14]);
    (void)BSP_MOTION_SENSOR_Read_Register(LSM6DSOX_0, LSM6DSOX_FSM_OUTS16, &fsm_status[15]);

    (void)BSP_MOTION_SENSOR_Write_Register(LSM6DSOX_0, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_USER_BANK << 6);
  }
  else
  {
    fsm_status[0] = 0;
    fsm_status[1] = 0;
    fsm_status[2] = 0;
    fsm_status[3] = 0;
    fsm_status[4] = 0;
    fsm_status[5] = 0;
    fsm_status[6] = 0;
    fsm_status[7] = 0;
    fsm_status[8] = 0;
    fsm_status[9] = 0;
    fsm_status[10] = 0;
    fsm_status[11] = 0;
    fsm_status[12] = 0;
    fsm_status[13] = 0;
    fsm_status[14] = 0;
    fsm_status[15] = 0;
  }

  for (i = 0; i < 8; i++)
  {
    if (fsm_status[i] != mem_fsm_status[i])
    {
      new_status = 1;
      break;
    }
  }

  if (new_status == 1)
  {
    memcpy((void *)&Msg->Data[MsgIndex], (void *)&fsm_status, 16);
    memcpy((void *)&mem_fsm_status, (void *)fsm_status, 16);
    MsgIndex = MsgIndex + 16;

    NewData++;
    NewDataFlags |= 256U;
  }
}

/**
  * @brief  Configures the RTC
  * @param  None
  * @retval None
  */
static void RTC_Config(void)
{
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
    - Hour Format    = Format 12
    - Asynch Prediv  = Value according to source clock
    - Synch Prediv   = Value according to source clock
    - OutPut         = Output Disable
    - OutPutPolarity = High Polarity
    - OutPutType     = Open Drain
   */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV_LSE;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV_LSE;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  Configures the current time and date
  * @param  None
  * @retval None
  */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /* Configure the Date using BCD format */
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Time using BCD format */
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
  * @brief  Configures the current time and date
  * @param  hh the hour value to be set
  * @param  mm the minute value to be set
  * @param  ss the second value to be set
  * @retval None
  */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{

  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIOPin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIOPin)
{
  int do_operation = 0;
  uint32_t int_current_time2_local;

  if (GPIOPin == KEY_BUTTON_PIN)
  {
    int_current_time2_local = IntCurrentTime2;

    /* Manage software debouncing*/
    if (IntCurrentTime1 == 0U && int_current_time2_local == 0U)
    {
      IntCurrentTime1 = HAL_GetTick();
      do_operation = 1;
    }
    else
    {
      IntCurrentTime2 = HAL_GetTick();
      int_current_time2_local = IntCurrentTime2;

      /* If I receive a button interrupt after more than 300 ms from the first one I get it, otherwise I discard it */
      if ((int_current_time2_local - IntCurrentTime1) > 300U)
      {
        IntCurrentTime1 = IntCurrentTime2;
        do_operation = 1;
      }
    }

    if (do_operation != 0)
    {
      if (DataLoggerActive != 0U)
      {
        TerminalActive = 0; /* always off */
      }
      else
      {
        TerminalActive = (TerminalActive != 0U) ? 0U : 1U; /* toggle on each button pressed */
      }
    }
  }
}

/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    CDC_TIM_PeriodElapsedCallback(htim);
}

/**
  * @brief  This function is executed in case of error occurrence
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while (1)
  {}
}

/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_GetTickUS function
 * @retval Error DWT counter (1: clock cycle counter not started, 0: clock cycle counter works)
 */
static uint32_t DWT_Delay_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;

  /* Enable TRC */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

  /* Enable clock cycle counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

  /* Delay 1ms */
  HAL_Delay(1);

  /* Check if clock cycle counter has started */
  if (DWT->CYCCNT)
  {
     return 0; /* Clock cycle counter started */
  }
  else
  {
    return 1; /* Clock cycle counter not started */
  }
}

/**
 * @brief  Get relative time in micro seconds
 * @note   Call at least every 2^32 cycles. Do not call from interrupt context!
 * @retval Relative time in micro seconds
 */
uint32_t DWT_GetTickUS(void)
{
  static uint64_t last_cycle_count_64 = 0;
  uint32_t clock_MHz = HAL_RCC_GetHCLKFreq() / 1000000;

  last_cycle_count_64 += DWT->CYCCNT - (uint32_t)(last_cycle_count_64);

  return (uint32_t)(last_cycle_count_64 / clock_MHz);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}

#endif

/**
  * @}
  */

/**
  * @}
  */

