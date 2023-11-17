/**
  ******************************************************************************
  * @file    SDDataLogRToS\Src\main.c
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
 * @page SDDataLogRToS SD Data Log
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
 * - FatFs generic FAT file system module provides access the storage devices 
 *  such as memory card and hard disk.
 * - FreeRTOS Real Time Kernel/Scheduler that allows applications to be organized as a collection of independent threads of execution
 *  (under MIT open source license)
 *
 * <b>Example Application</b>
 * 
 * The Example application initializes all the Components and pressing the User button is possible to 
 * start/stop the recording off all board's sensors to SD-card.
 * The program save 3 different files on SD-card for each log:
 *   - <b>Sens000.csv</b> where it stores the values of Acc/Gyro/Mag/Pressure/Temperature/Humidity
 *   - <b>Mic000.wav</b> where it stores the wave file for Analog Microphone at 16Khz
 *   - <b>Rep000.txt</b> where it stores the summary of used FreeRTOS queues and Max time for writing the Audio Buffer to the .wav file:
 *
 * <b>Pool Queue</b>:\n
 *   - <b>Max Size</b>  = XXX\n
 *   - <b>Released</b>  = XXX\n
 *   - <b>Allocated</b> = XXX
 *
 * <b>Message Queue</b>:\n
 *   - <b>Max Size</b>  = XXX\n
 *   - <b>Released</b>  = XXX\n
 *   - <b>Allocated</b> = XXX
 * Max time for writing XXXXbytes for Audio =XXX mSec\n
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "TargetFeatures.h"
#include "datalog_application.h"

#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio_SensorTile.box.h" /* defines SD_Driver as external */

/* Exported Variables --------------------------------------------------------*/
osMessageQId dataQueue_id;
uint32_t MaxWriteTimeAudio=0;

/* For understanding the Data Pool Queue Size */
int32_t PoolAllocated=0;
int32_t PoolReleased=0;
int32_t PoolMaxSize=0;

/* For understanding the Message Queue Size */
int32_t MessagePushed=0;
int32_t MessageRemoved=0;
int32_t MessageMaxSize=0;

/* Imported variables ---------------------------------------------------------*/
extern uint16_t PCM_Buffer[];

/* Private define ------------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  THREAD_1 = 0,
  THREAD_2
} Thread_TypeDef;

/* Private variables ---------------------------------------------------------*/

osThreadId GetDataThreadId, WriteDataThreadId;

osMessageQDef(dataqueue, DATAQUEUE_SIZE, int);

osPoolId sensorPool_id;
osPoolDef(sensorPool, DATAQUEUE_SIZE, T_SensorsData);

osSemaphoreId readDataSem_id;
osSemaphoreDef(readDataSem);

static volatile uint8_t UserButtonPressed = 0;
volatile uint8_t SD_Log_Enabled = 0;

/* Private function prototypes -----------------------------------------------*/
static void GetData_Thread(void const *argument);
static void WriteData_Thread(void const *argument);
static void dataTimer_Callback(void const *arg);
static void dataTimerStart(void);
static void dataTimerStop(void);

osTimerId sensorTimId;
osTimerDef(SensorTimer, dataTimer_Callback);

void SystemClock_Config(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();
  
  /* Initialize the SensorTile.box */
  InitTargetPlatform();

  /* Everything is Ready */
  BSP_LED_On(LED_GREEN);

  /* Thread 1 definition */
  osThreadDef(THREAD_1, GetData_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*8);

  /* Thread 2 definition */
  osThreadDef(THREAD_2, WriteData_Thread,osPriorityNormal, 0, configMINIMAL_STACK_SIZE*8);

  /* Start thread 1 */
  GetDataThreadId = osThreadCreate(osThread(THREAD_1), NULL);

  /* Start thread 2 */
  WriteDataThreadId = osThreadCreate(osThread(THREAD_2), NULL);

  /* Start scheduler */
  osKernelStart();

  while(1);
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pin connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == KEY_BUTTON_PIN) {
    UserButtonPressed=1;
    osSemaphoreRelease(readDataSem_id);
  }
}

/**
  * @brief  Get data raw from sensors and push them into the queue
  * @param  thread not used
  * @retval None
  */
static void GetData_Thread(void const *argument)
{
  (void) argument;

  sensorPool_id = osPoolCreate(osPool(sensorPool));
  dataQueue_id = osMessageCreate(osMessageQ(dataqueue), NULL);

  readDataSem_id = osSemaphoreCreate(osSemaphore(readDataSem), 1);
  osSemaphoreWait(readDataSem_id, osWaitForever);

  for (;;) {
    osSemaphoreWait(readDataSem_id, osWaitForever);
    if(UserButtonPressed) {
      UserButtonPressed = 0;
      if(SD_Log_Enabled) {
        dataTimerStop();
        /* Stop Audio Recording */
        if(BSP_AUDIO_IN_Stop(BSP_AUDIO_IN_INSTANCE) != BSP_ERROR_NONE) {
          ErrorHanlder(STBOX1_AUDIO_ERROR);
        }
      }
      MessagePushed++;
      if(MessageMaxSize< (MessagePushed-MessageRemoved)) {
        MessageMaxSize = MessagePushed-MessageRemoved;
      }
      osMessagePut(dataQueue_id, MSG_ENABLE_DISABLE, osWaitForever);
    } else {
      /* Try to allocate a memory block and check if is not NULL */      
      T_SensorsData *mptr = osPoolAlloc(sensorPool_id);
      if(mptr != NULL) {
        PoolAllocated++;
        if(PoolMaxSize< (PoolAllocated-PoolReleased)) {
          PoolMaxSize = PoolAllocated-PoolReleased;
        }
        /* Get Data from Sensors */
        if(getSensorsData(mptr) == BSP_ERROR_NONE) {
          /* Push the new memory Block in the Data Queue */
          MessagePushed++;
          if(MessageMaxSize< (MessagePushed-MessageRemoved)) {
            MessageMaxSize = MessagePushed-MessageRemoved;
          }
          if(osMessagePut(dataQueue_id, (uint32_t)mptr, osWaitForever) != osOK) {
             ErrorHanlder(STBOX1_OS);
          }
        } else {
          ErrorHanlder(STBOX1_MEMS_ERROR);
        }
      } else {
        ErrorHanlder(STBOX1_OS);
      }
    }
  }
}


/**
  * @brief  get data from the queue and write then on file
  * @param  argument not used
  * @retval None
  */
static void WriteData_Thread(void const *argument)
{
  (void) argument;
  osEvent evt;
  T_SensorsData *rptr;
  int size;
  char data_s[256];

  for (;;) {
    evt = osMessageGet(dataQueue_id, osWaitForever);  // wait for message
    /* Check... */
    MessageRemoved++;
    if (evt.status == osEventMessage) {
      if(evt.value.v == MSG_ENABLE_DISABLE) {
        if (SD_Log_Enabled) {
          DATALOG_SD_Log_Disable();
          SD_Log_Enabled=0;

          /* Close the Wav file */
          closeFileAudio();

          /* Print out the Summary */
          PrintSummary();

          /* Reset the Counters */
          PoolAllocated=0;
          PoolReleased=0;
          PoolMaxSize=0;
          MessagePushed=0;
          MessageRemoved=0;
          MessageMaxSize=0;
          MaxWriteTimeAudio=0;
        } else {
          DATALOG_SD_Log_Enable();
          SD_Log_Enabled=1;

          /* Open Wav file */
          if(SD_LogAudio_Enabled==0) {
            DATALOG_SD_LogAudio_Enable();
          }

          dataTimerStart();

          /* Start Audio Recording */
          if(BSP_AUDIO_IN_Record(BSP_AUDIO_IN_INSTANCE, (uint8_t *) PCM_Buffer, PCM_AUDIO_IN_SAMPLES*2) != BSP_ERROR_NONE) {
            ErrorHanlder(STBOX1_AUDIO_ERROR);
          }
        }
      } else {
        if(evt.value.v==MSG_AUDIO_SAVE) {
          SaveAudioData();
        } else {
          rptr = evt.value.p;
          size = sprintf(data_s, "%ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld, %5.2f, %5.2f, %4.1f\r\n",
                         rptr->ms_counter,
                         rptr->acc.x , rptr->acc.y , rptr->acc.z ,
                         rptr->gyro.x, rptr->gyro.y, rptr->gyro.z,
                         rptr->mag.x , rptr->mag.y , rptr->mag.z ,
                         rptr->pressure, rptr->temperature, rptr->humidity);
          DATALOG_SD_writeBuf(data_s, size);
          osPoolFree(sensorPool_id, rptr);     // free memory allocated for message
          PoolReleased++;
        }
      }
    }
  }
}

/**
  * @brief  callback function called by osTimer
  * @param  void *arg pointer to arguments list (not used)
  * @retval None
  */
static void dataTimer_Callback(void const *arg)
{ 
  osSemaphoreRelease(readDataSem_id);
}

/**
  * @brief  Function for starting the ostimer
  * @param  None
  * @retval None
  */
static void dataTimerStart(void)
{
  osStatus  status;
  uint32_t  exec;
  
  // Create periodic timer
  exec = 1;
  sensorTimId = osTimerCreate(osTimer(SensorTimer), osTimerPeriodic, &exec);
  if (sensorTimId)  {
    // start timer
    status = osTimerStart (sensorTimId, DATA_PERIOD_MS);
    if (status != osOK)  {
      ErrorHanlder(STBOX1_OS);
    }
  }
}

/**
  * @brief  Function for stopping the ostimer
  * @param  None
  * @retval None
  */
static void dataTimerStop(void)
{
  osTimerStop(sensorTimId);
  osTimerDelete(sensorTimId);
}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  uint32_t Instance Not used
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  if(SD_LogAudio_Enabled) {
    AudioProcess_SD_Recording(PCM_Buffer, PCM_AUDIO_IN_SAMPLES);  
  }
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  uint32_t Instance Not used
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  if(SD_LogAudio_Enabled) {
    AudioProcess_SD_Recording(PCM_Buffer, PCM_AUDIO_IN_SAMPLES);
  }
}

/**
  * @brief  System Clock tree configuration
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
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

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

