/**
  ******************************************************************************
  * @file    SDDataLogRToS\Src\datalog_application.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "string.h"
#include "TargetFeatures.h"
#include "datalog_application.h"

/* Exported Variables --------------------------------------------------------*/
uint32_t SD_LogAudio_Enabled = 0;

/* Private Defines -----------------------------------------------------------*/
#define STBOX1_AUDIO_DATA_NOT_READY 0xFFFFFF

/* Private variables ---------------------------------------------------------*/
static FATFS SDFatFs;  /* File system object for SD card logical drive */
static FIL MyFileMems;     /* File object for MEMS */
static FIL MyFileAudio;    /* File object for Audio File */
static char SDPath[4]; /* SD card logical drive path */

static uint16_t Audio_OUT_Buff[AUDIO_BUFF_SIZE];
static volatile uint32_t  ReadIndexBuffer = STBOX1_AUDIO_DATA_NOT_READY;

static uint8_t pAudioHeader[44];

/* Private function prototypes -----------------------------------------------*/
static uint32_t WavProcess_HeaderInit(void);
static void DATALOG_SD_LogAudio_Disable(void);
static uint32_t WavProcess_HeaderUpdate(uint32_t len);

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
void DATALOG_SD_Init(void)
{
  if (FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
    if (f_mount(&SDFatFs, (TCHAR const *)SDPath, 0) != FR_OK)
    {
      ErrorHanlder(STBOX1_FATFS);
    }
  }

  /* Explicit Init to trap SD Missing */
  if (SD_Driver.disk_initialize(0) != RES_OK)
  {
    ErrorHanlder(STBOX1_FATFS);
  }
}

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;
  char header[] = "T [ms],AccX [mg],AccY [mg],AccZ [mg],GyroX [mdps],GyroY [mdps],GyroZ [mdps],MagX [mgauss],MagY [mgauss],MagZ [mgauss],P [mB],T ['C],H [%]\r\n";
  uint32_t byteswritten; /* written byte count */
  char file_name[30];
  sprintf(file_name, "Sens%03d.csv", sdcard_file_counter);
  sdcard_file_counter++;

  if (f_open(&MyFileMems, (char const *)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    sdcard_file_counter--;
    BSP_LED_On(LED_RED);
    return 0;
  }

  if (f_write(&MyFileMems, (const void *)&header, sizeof(header) - 1, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return 0;
  }

  BSP_LED_Off(LED_GREEN);
  BSP_LED_On(LED_BLUE);
  BSP_LED_Off(LED_RED);
  return 1;
}

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_LogAudio_Enable(void)
{
  static uint16_t sdcard_file_counter = 0;
  uint32_t byteswritten; /* written byte count */
  char file_name[30];
  sprintf(file_name, "Mic%03d.wav", sdcard_file_counter);
  sdcard_file_counter++;

  if (f_open(&MyFileAudio, (char const *)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    sdcard_file_counter--;
    BSP_LED_On(LED_RED);
    return 0;
  }

  WavProcess_HeaderInit();

  if (f_write(&MyFileAudio, (uint8_t *) pAudioHeader, sizeof(pAudioHeader), (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return 0;
  }

  /* Flush the cached information (pAudioHeader), writing data to the volume */
  if (f_sync(&MyFileAudio) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return 0;
  }

  SD_LogAudio_Enabled = 1;
  BSP_LED_Off(LED_RED);
  return 1;
}

/**
  * @brief  Management of the audio file closing
  * @param  None
  * @retval None
  */
void closeFileAudio(void)
{
  if (SD_LogAudio_Enabled)
  {
    DATALOG_SD_LogAudio_Disable();
    SD_LogAudio_Enabled = 0;
  }
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
static void DATALOG_SD_LogAudio_Disable(void)
{
  uint32_t len;
  uint32_t byteswritten;

  if (SD_LogAudio_Enabled)
  {
    len = f_size(&MyFileAudio);
    WavProcess_HeaderUpdate(len);

    /* Update the data length in the header of the recorded Wave */
    f_lseek(&MyFileAudio, 0);
    if (f_write(&MyFileAudio, (uint8_t *)pAudioHeader,  sizeof(pAudioHeader), (void *)&byteswritten) != FR_OK)
    {
      ErrorHanlder(STBOX1_FATFS);
    }

    /* Close file */
    f_close(&MyFileAudio);
    SD_LogAudio_Enabled = 0;
  }
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderInit(void)
{
  uint16_t   BitPerSample = 16;
  uint32_t   ByteRate = AUDIO_SAMPLING_FREQUENCY * (BitPerSample / 8);

  uint32_t   SampleRate = AUDIO_SAMPLING_FREQUENCY;
  uint16_t   BlockAlign = BitPerSample / 8;

  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pAudioHeader[0] = 'R';
  pAudioHeader[1] = 'I';
  pAudioHeader[2] = 'F';
  pAudioHeader[3] = 'F';

  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pAudioHeader[4] = 0x00;
  pAudioHeader[5] = 0x4C;
  pAudioHeader[6] = 0x1D;
  pAudioHeader[7] = 0x00;

  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pAudioHeader[8]  = 'W';
  pAudioHeader[9]  = 'A';
  pAudioHeader[10] = 'V';
  pAudioHeader[11] = 'E';

  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pAudioHeader[12]  = 'f';
  pAudioHeader[13]  = 'm';
  pAudioHeader[14]  = 't';
  pAudioHeader[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pAudioHeader[16]  = 0x10;
  pAudioHeader[17]  = 0x00;
  pAudioHeader[18]  = 0x00;
  pAudioHeader[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pAudioHeader[20]  = 0x01;
  pAudioHeader[21]  = 0x00;

  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pAudioHeader[22]  = 1;
  pAudioHeader[23]  = 0x00;

  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
  pAudioHeader[24]  = (uint8_t)((SampleRate & 0xFF));
  pAudioHeader[25]  = (uint8_t)((SampleRate >> 8) & 0xFF);
  pAudioHeader[26]  = (uint8_t)((SampleRate >> 16) & 0xFF);
  pAudioHeader[27]  = (uint8_t)((SampleRate >> 24) & 0xFF);

  /* Write the Byte Rate -----------------------------------------------------*/
  pAudioHeader[28]  = (uint8_t)((ByteRate & 0xFF));
  pAudioHeader[29]  = (uint8_t)((ByteRate >> 8) & 0xFF);
  pAudioHeader[30]  = (uint8_t)((ByteRate >> 16) & 0xFF);
  pAudioHeader[31]  = (uint8_t)((ByteRate >> 24) & 0xFF);

  /* Write the block alignment -----------------------------------------------*/
  pAudioHeader[32]  = BlockAlign;
  pAudioHeader[33]  = 0x00;

  /* Write the number of bits per sample -------------------------------------*/
  pAudioHeader[34]  = BitPerSample;
  pAudioHeader[35]  = 0x00;

  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pAudioHeader[36]  = 'd';
  pAudioHeader[37]  = 'a';
  pAudioHeader[38]  = 't';
  pAudioHeader[39]  = 'a';

  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pAudioHeader[40]  = 0x00;
  pAudioHeader[41]  = 0x4C;
  pAudioHeader[42]  = 0x1D;
  pAudioHeader[43]  = 0x00;

  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderUpdate(uint32_t len)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pAudioHeader[4] = (uint8_t)(len);
  pAudioHeader[5] = (uint8_t)(len >> 8);
  pAudioHeader[6] = (uint8_t)(len >> 16);
  pAudioHeader[7] = (uint8_t)(len >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  len -= 44;
  pAudioHeader[40] = (uint8_t)(len);
  pAudioHeader[41] = (uint8_t)(len >> 8);
  pAudioHeader[42] = (uint8_t)(len >> 16);
  pAudioHeader[43] = (uint8_t)(len >> 24);
  /* Return 0 if all operations are OK */
  return 0;
}

uint8_t DATALOG_SD_writeBuf(char *s, uint32_t size)
{
  uint32_t byteswritten;
  if (f_write(&MyFileMems, s, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return 0;
  }
  return 1;
}

/**
  * @brief  Management of the audio data logging
  * @param  pInBuff     points to the audio buffer.
  * @param  len         number of samples to process.
  * @retval None
  */
void AudioProcess_SD_Recording(uint16_t *pInBuff, uint32_t len)
{
  static uint32_t WriteIndexBuffer = 0;

  /* Accumulate audio buffer into local ping-pong buffer before writing */
  memcpy(Audio_OUT_Buff + WriteIndexBuffer, pInBuff, len * sizeof(uint16_t));
  WriteIndexBuffer += len;

  WriteIndexBuffer &= AUDIO_BUFF_SIZE_MASK;

  if (WriteIndexBuffer == (AUDIO_BUFF_SIZE / 2))
  {
    /* first half */
    ReadIndexBuffer = 0;
    /* Send the Message for writing out the 1/2 buffer */
    MessagePushed++;
    if (MessageMaxSize < (MessagePushed - MessageRemoved))
    {
      MessageMaxSize = MessagePushed - MessageRemoved;
    }
    if (osMessagePut(dataQueue_id, MSG_AUDIO_SAVE, osWaitForever) != osOK)
    {
      ErrorHanlder(STBOX1_OS);
    }
  }
  else if (WriteIndexBuffer == 0)
  {
    /* second half */
    ReadIndexBuffer = AUDIO_BUFF_SIZE / 2;
    /* Send the Message for writing out the 1/2 buffer */
    MessagePushed++;
    if (MessageMaxSize < (MessagePushed - MessageRemoved))
    {
      MessageMaxSize = MessagePushed - MessageRemoved;
    }
    if (osMessagePut(dataQueue_id, MSG_AUDIO_SAVE, osWaitForever) != osOK)
    {
      ErrorHanlder(STBOX1_OS);
    }
  }

  /* Control section */
  if (ReadIndexBuffer == 0)
  {
    /* We need to read the First half */
    if (WriteIndexBuffer < (AUDIO_BUFF_SIZE / 2))
    {
      BSP_LED_Toggle(LED_RED);
    }
  }
  else if (ReadIndexBuffer == (AUDIO_BUFF_SIZE / 2))
  {
    /* We need to Read the Second half */
    if (WriteIndexBuffer > (AUDIO_BUFF_SIZE / 2))
    {
      BSP_LED_Toggle(LED_RED);
    }
  }
}

void SaveAudioData(void)
{
  uint32_t BytesWritten;
  uint32_t Tick;
  uint32_t DeltaTick;
  Tick = HAL_GetTick();
  if (f_write(&MyFileAudio, ((uint8_t *)(Audio_OUT_Buff + ReadIndexBuffer)),
              AUDIO_BUFF_SIZE /* Because we need to write 16bit for sample */,
              (void *)&BytesWritten) != FR_OK)
  {
    ErrorHanlder(STBOX1_FATFS);
  }
  DeltaTick = HAL_GetTick() - Tick;
  if (DeltaTick > MaxWriteTimeAudio)
  {
    MaxWriteTimeAudio = DeltaTick;
  }
}


/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  BSP_LED_On(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  f_close(&MyFileMems);
}

/**
  * @brief  DeInit SD card
  * @param  None
  * @retval None
  */
void DATALOG_SD_DeInit(void)
{
  FATFS_UnLinkDriver(SDPath);
  HAL_SD_DeInit(&hsd1);
}

/**
  * @brief  Disable SDCard Log
  * @param  T_SensorsData *mptr structure where to save Env/Inertial values
  * @retval int32_t ErrorCode
  */
int32_t getSensorsData(T_SensorsData *mptr)
{
  int32_t ret = BSP_ERROR_NONE;
  mptr->ms_counter = HAL_GetTick();

  if (BSP_MOTION_SENSOR_GetAxes(LSM6DSOX_0, MOTION_ACCELERO, &mptr->acc) == BSP_ERROR_COMPONENT_FAILURE)
  {
    mptr->acc.x = 0;
    mptr->acc.y = 0;
    mptr->acc.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (BSP_MOTION_SENSOR_GetAxes(LSM6DSOX_0, MOTION_GYRO, &mptr->gyro) == BSP_ERROR_COMPONENT_FAILURE)
  {
    mptr->gyro.x = 0;
    mptr->gyro.y = 0;
    mptr->gyro.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (BSP_MOTION_SENSOR_GetAxes(LIS2MDL_0, MOTION_MAGNETO, &mptr->mag) == BSP_ERROR_COMPONENT_FAILURE)
  {
    mptr->mag.x = 0;
    mptr->mag.y = 0;
    mptr->mag.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (BSP_ENV_SENSOR_GetValue(LPS22HH_0, ENV_PRESSURE, &mptr->pressure) == BSP_ERROR_COMPONENT_FAILURE)
  {
    mptr->pressure = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (BSP_ENV_SENSOR_GetValue(STTS751_0, ENV_TEMPERATURE, &mptr->temperature) == BSP_ERROR_COMPONENT_FAILURE)
  {
    mptr->temperature = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if (BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, &mptr->humidity) == BSP_ERROR_COMPONENT_FAILURE)
  {
    mptr->humidity = 0.0f;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  return ret;
}

/**
  * @brief  Print Out Summary on Screen or SD card.
  * @param  None
  * @retval None
  */
void PrintSummary(void)
{
  /* Writing on report File on SD Card */
  static uint16_t sdcard_file_counter = 0;
  uint32_t byteswritten; /* written byte count */
  char file_name[30];
  int32_t size;
  char data[256];

  sprintf(file_name, "Rep%03d.txt", sdcard_file_counter);
  sdcard_file_counter++;

  if (f_open(&MyFileMems, (char const *)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
  {
    sdcard_file_counter--;
    BSP_LED_On(LED_RED);
    return;
  }


  size = sprintf(data, "\r\n------------------------------------------------------------\r\n");
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "STMicroelectronics %s\r\n"
                 "\tVersion %c.%c.%c\r\n"
                 "\tSTM32L4R9ZI-SensorTile.box board\r\n",
                 STBOX1_PACKAGENAME,
                 STBOX1_VERSION_MAJOR, STBOX1_VERSION_MINOR, STBOX1_VERSION_PATCH);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\t(HAL %ld.%ld.%ld_%ld)\r\n\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                 " (IAR)\r\n",
#elif defined (__ARMCC_VERSION)
                 " (KEIL)\r\n",
#elif defined (__GNUC__)
                 " (STM32CubeIDE)\r\n",
#endif /* IDE */
                 HAL_GetHalVersion() >> 24,
                 (HAL_GetHalVersion() >> 16) & 0xFF,
                 (HAL_GetHalVersion() >> 8) & 0xFF,
                 HAL_GetHalVersion()      & 0xFF,
                 __DATE__, __TIME__);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "------------------------------------------------------------\r\n");
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }

  size = sprintf(data, "Pool Queue:\r\n");
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\tMax Size  =%ld\r\n", PoolMaxSize);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\tReleased  =%ld\r\n", PoolReleased);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\tAllocated =%ld\r\n", PoolAllocated);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }

  size = sprintf(data, "Message Queue:\r\n");
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\tMax Size  =%ld\r\n", MessageMaxSize);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\tReleased  =%ld\r\n", MessageRemoved);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "\tAllocated =%ld\r\n", MessagePushed);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }
  size = sprintf(data, "Max time for writing %dbytes for Audio =%ld mSec\r\n", AUDIO_BUFF_SIZE, MaxWriteTimeAudio);
  if (f_write(&MyFileMems, data, size, (void *)&byteswritten) != FR_OK)
  {
    BSP_LED_On(LED_RED);
    return;
  }

  BSP_LED_Off(LED_RED);
  f_close(&MyFileMems);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

