/**
  ******************************************************************************
  * @file    BLELowPowerRToS\Src\main.c
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
 * @page BLELowPowerRToS FreeRTOS low Power BLE trasmission of Sensor's data
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
 * - FreeRTOS Real Time Kernel/Scheduler that allows applications to be organized as a collection of independent threads of execution
 *   (under MIT open source license)
 *
 *
 * <b>Example Application</b>
 *
 * The Example application initializes all the Components and Library creating some Custom Bluetooth services:
 * - The first service exposes all the HW characteristics:
 *    - Temperature, Humidity, Pressure, Magnetometer, Gyroscope and Accelerometer
 *    - Microphones Signal Noise dB level.
 *    - LSM6DSOX hardware features (FreeFall/Single&Double Tap/Tilt/6D Orientation)
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
#include "cmsis_os.h"
#include "main.h"

#include "PowerControl.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define LED_TIME_OFF 2000UL /* shall not be same than ON time */
#define LED_TIME_ON   100UL

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern uint16_t PCM_Buffer[];

/* Exported Variables --------------------------------------------------------*/
osSemaphoreId semRun;

volatile float RMS_Ch;
float dB_Value_Old;

uint8_t bdaddr[6];
char BoardName[8]={NAME_BLUEMS,0};
uint16_t PedometerStepCount= 0;

/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle;

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
static volatile int           ButtonPressed    = 0;
static volatile int           MEMSInterrupt    = 0;

volatile int PowerButtonPressed                = 0;
volatile int NeedToRestartDFU                  = 0;

static volatile uint32_t      SendEnv          = 0;
static volatile uint32_t      SendAudioLevel   = 0;
static volatile uint32_t      SendAccGyroMag   = 0;
static volatile uint32_t      ledTimer         = 0;

static volatile uint32_t SendBatteryInfo       = 0;

static int audioInProgress = 0 ;

/* Private function prototypes -----------------------------------------------*/

//static void Init_BlueNRG_Custom_Services(void);
//static void Init_BlueNRG_Stack(void);

static void SendEnvironmentalData(void);
static void MEMSCallback(void);

static void ButtonCallback(void);
static void SendMotionData(void);
static void SendAudioLevelData(void);

static void LedBlinkCb(void const *arg);

static int  HardwareInit(void);

static void ProcessThread(void const *argument);
static void HostThread   (void const *argument);

static void SendBatteryInfoData(void);

void AudioProcess_FromMics(void);
void AudioProcess_DB_Noise(void);

void APP_UserEvtRx(void *pData);

static void OsTimerCallback (void const *arg);

#ifdef STBOX1_ENABLE_PRINTF
static void DisplayFirmwareInfo(void);
#endif /* STBOX1_ENABLE_PRINTF */

/* CMSIS-OS  definitions */
/* threads */
osThreadDef(THREAD_1, ProcessThread, osPriorityNormal     , 0, configMINIMAL_STACK_SIZE*8);
osThreadDef(THREAD_2, HostThread   , osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*8);

/* Semaphores */
osSemaphoreDef(SEM_Sm1);

/* Mail Queue */
osMailQId  mail;
osMailQDef(mail, 30, msgData_t);

/* Timers */
osTimerId timLedId,timEnvId,timMotionId;
osTimerId timAudioLevId;
osTimerId timBatId;

osTimerDef (TimerLedHandle , LedBlinkCb);
osTimerDef (TimerEnvHandle , OsTimerCallback);
osTimerDef (TimerBatHandle , OsTimerCallback);
osTimerDef (TimerMotionHandle, OsTimerCallback);
osTimerDef (TimerAudioLevHandle, OsTimerCallback);

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

  HardwareInit();

  /* Create threads */
  osThreadCreate(osThread(THREAD_1), NULL);
  osThreadCreate(osThread(THREAD_2), NULL);

  /* Create the semaphores */
  semRun = osSemaphoreCreate(osSemaphore(SEM_Sm1), 1);

  /* create mail queue */
  mail = osMailCreate(osMailQ(mail), NULL);

  /* set lowest reachable power mode  */
  SetMinPowerMode(IDLE_SLEEP_STOP);

  /* Start scheduler  */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler  */
  for (;;);
}

/**
  * @brief  Process Thread Function
  * @param  void *argument pointer to argument list
  * @retval None
  */
static void ProcessThread(void const *argument)
{
  while (1){
    if (semRun != NULL){
      if(osSemaphoreWait(semRun, osWaitForever) == osOK) {
        if(set_connectable){
          msgData_t msg;
          msg.type  = SET_CONNECTABLE ;
          SendMsgToHost(&msg);
          set_connectable =0;
        }

        /* Handle Interrupt from MEMS */
        if(MEMSInterrupt) {
          MEMSCallback();
          MEMSInterrupt=0;
        }

        /* Handle user button */
        if(ButtonPressed) {
          ButtonCallback();
          ButtonPressed=0;
        }

        /* Power Off the SensorTile.box */
        if(PowerButtonPressed){
          BSP_BC_CmdSend(SHIPPING_MODE_ON);
          PowerButtonPressed =0;
        }

        /* handle the DFU reboot request */
        if(NeedToRestartDFU==1) {
          NeedToRestartDFU=0;
          osDelay(5000);
          DFU_Var = DFU_MAGIC_NUM;
          HAL_NVIC_SystemReset();
        }

        /* Environmental Data */
        if(SendEnv) {
          SendEnv=0;
          SendEnvironmentalData();
        }

        /* Mic Data */
        if (SendAudioLevel) {
          SendAudioLevel = 0;
          SendAudioLevelData();
        }

        /* Motion Data */
        if(SendAccGyroMag) {
          SendAccGyroMag=0;
          SendMotionData();
        }

        /* Battery Info Data */
        if(SendBatteryInfo) {
          SendBatteryInfo=0;
          SendBatteryInfoData();
        }
      }
    }
  }
}

/**
  * @brief  Function for sending Messages from Thread to Host Process
  * @param  msgData_t *msgPtr pointer to message structure
  * @retval None
  */
int SendMsgToHost(msgData_t *msgPtr)
{
  msgData_t *ptr;
  if (mail) {
    /* Allocate memory */
    ptr = osMailAlloc(mail, osWaitForever);
    if (ptr) {
      BLUENRG_memcpy(ptr,msgPtr, sizeof(msgData_t));
      osMailPut(mail, ptr);
    } else {
      STBOX1_PRINTF("SendMsgToHost: mem allocation failed %d\r\n",msgPtr->type);
      return 0;
    }
  }
  return 1;
}

/**
  * @brief  Host Thread Function
  * @param  void *argument pointer to argument list
  * @retval None
  */
static void HostThread(void const *argument)
{
  msgData_t *msgPtr;
  osEvent  evt;

  for (;;) {
    /* wait for mail */
    evt = osMailGet(mail, osWaitForever);
    if (evt.status == osEventMail) {
      msgPtr = evt.value.p;
      switch(msgPtr->type) {
        case SET_CONNECTABLE:
          setConnectable();
          LedBlinkStart();
          break;
        case  PROCESS_EVENT :
          hci_user_evt_proc();
          break;
        case  CONF_NOTIFY :
          Config_Update(msgPtr->conf.feature,msgPtr->conf.command,msgPtr->conf.data);
          break;
        case  ACC :
          BLE_AccEnvUpdate(msgPtr->acc, 2);
          break;
        case  ACC_STEP :
          if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
            BLE_AccEnvUpdate(msgPtr->stepCnt, 2);
          }
          break;
        case  AUDIO_LEV :
          BLE_AudioLevelUpdate(msgPtr->dB_Value,1);
          break;
        case  ENV :
          BLE_EnvironmentalUpdate(msgPtr->env.press,
                                  msgPtr->env.hum,
                                  msgPtr->env.temp,
                                  0);
          break;
        case MOTION :
        {
          BLE_MANAGER_INERTIAL_Axes_t ACC_SensorValue;
          BLE_MANAGER_INERTIAL_Axes_t GYR_SensorValue;
          BLE_MANAGER_INERTIAL_Axes_t MAG_SensorValue;
          
          ACC_SensorValue.Axis_x= msgPtr->motion.acc.x;
          ACC_SensorValue.Axis_y= msgPtr->motion.acc.y;
          ACC_SensorValue.Axis_z= msgPtr->motion.acc.z;

          GYR_SensorValue.Axis_x= msgPtr->motion.gyr.x;
          GYR_SensorValue.Axis_y= msgPtr->motion.gyr.y;
          GYR_SensorValue.Axis_z= msgPtr->motion.gyr.z;

          MAG_SensorValue.Axis_x= msgPtr->motion.mag.x;
          MAG_SensorValue.Axis_y= msgPtr->motion.mag.y;
          MAG_SensorValue.Axis_z= msgPtr->motion.mag.z;
          
          BLE_AccGyroMagUpdate(&ACC_SensorValue,
                               &GYR_SensorValue,
                               &MAG_SensorValue);
          }
          break;
        case BATTERY_INFO:
          BLE_BatteryUpdate(msgPtr->batteryInfo.level,
                            msgPtr->batteryInfo.voltage,
                            0x8000,
                            msgPtr->batteryInfo.soc_status);
          break;
        case TERM_STDOUT:
          Term_Update(msgPtr->term.data,msgPtr->term.length);
          break;
        case TERM_STDERR:
          Stderr_Update(msgPtr->term.data,msgPtr->term.length);
          break;
        default :
          STBOX1_PRINTF("HostThread unexpected message:%d\r\n",msgPtr->type );
      }

      /* free memory allocated for mail */
      osMailFree(mail, msgPtr);

      /* check subsequent processing    */
      if(semRun) {
        osSemaphoreRelease(semRun);
      }
    }
  }
}

/**
  * @brief  System Initialization function
  * @param  None
  * @retval None
  */
static int HardwareInit(void)
{
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();

  initPowerController();

  InitTargetPlatform();

#ifdef STBOX1_ENABLE_PRINTF
  DisplayFirmwareInfo();
#endif /* STBOX1_ENABLE_PRINTF */

//  /* Initialize the BlueNRG */
//  Init_BlueNRG_Stack();
//
//  /* Initialize the BlueNRG Custom services */
//  Init_BlueNRG_Custom_Services();
  
  /* Initialize the BlueNRG stack and services */
  BluetoothInit();

  PowerButtonPressed  =0;
  return 0;
}

/**
  * @brief  Callback functions called by Ostimer
  * @param  void *arg pointer to the ostimer
  * @retval None
  */
static void OsTimerCallback  (void const *arg)
{
  if(arg == timEnvId){
    if(EnvironmentalTimerEnabled) {
      SendEnv=1;
    }
  }
  else if     (arg == timBatId) {
    if(BatteryTimerEnabled) {
      SendBatteryInfo= 1;
    }
  }
  else if (arg == timMotionId){
    SendAccGyroMag=1;
  }
  else if (arg == timAudioLevId){
    SendAudioLevel=1;
  }
  else{
    STBOX1_PRINTF("wrong timer : %ld\n",(uint32_t)arg);
  }
  if(semRun) {
    osSemaphoreRelease(semRun);
  }
}

/**
  * @brief  This function starts one ostimer
  * @param  msgType_t type defines ostimer we need to start
  * @param  uint32_t period period in mSec used for the ostimer
  * @retval None
  */
int startProc(msgType_t type,uint32_t period)
{
  osTimerId id     = NULL;
  switch (type)
  {
    case BATTERY_INFO:
      if (!timBatId) {
        timBatId = osTimerCreate (osTimer(TimerBatHandle), osTimerPeriodic, NULL);
      }
      id = timBatId;
      break;
    case ENV:
      if (!timEnvId) {
        timEnvId = osTimerCreate (osTimer(TimerEnvHandle), osTimerPeriodic, NULL);
      }
      EnableEnvSensors();
      id = timEnvId;
       break;
    case MOTION:
      if (!timMotionId) {
        timMotionId = osTimerCreate (osTimer(TimerMotionHandle),osTimerPeriodic, NULL);
      }
      EnableMotionSensors ();
      id = timMotionId;
      break;
    case AUDIO_LEV:
      RMS_Ch=0;
      dB_Value_Old =0;
      if (!timAudioLevId) {
        timAudioLevId = osTimerCreate (osTimer(TimerAudioLevHandle),osTimerPeriodic, NULL);
      }
      id = timAudioLevId;
      PowerCtrlLock();
      audioInProgress = 0x1 ;
      break;
    default :
      STBOX1_PRINTF("wrong type : %d\n",type);
      break;
  }
  
  if (id){
    if  (osTimerStart (id, period) != osOK){
      STBOX1_PRINTF("failed starting timer\n");
    }

  }
  return 0;
}

/**
  * @brief  This function stops one ostimer
  * @param  msgType_t type defines ostimer we need to stop
  * @retval None
  */
int stopProc(msgType_t type)
{
  msgData_t msg;
  osTimerId id = NULL;
  msg.type = type;
  switch (type) {
    case BATTERY_INFO:
      id           = timBatId;
      timBatId     = NULL;
      break;
    case ENV:
      id           = timEnvId;
      timEnvId     = NULL;
      DisableEnvSensors();
      break;
    case MOTION:
      id            = timMotionId;
      timMotionId   = NULL;
      DisableMotionSensors ();
      break;
    case AUDIO_LEV:
      id            = timAudioLevId;
      if (audioInProgress) {
        PowerCtrlUnLock();
        audioInProgress =0;
      }
      timAudioLevId = NULL;
      break;
  default :
      break;
  }
  if (id){
    if  (osTimerStop (id) != osOK){
      STBOX1_PRINTF("could not stop timer\n");
    }
    if (osTimerDelete (id) != osOK)  {
      STBOX1_PRINTF("could not delete timer\n");
    }
  }
  SendMsgToHost(&msg);
  return 0;
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
  STBOX1_PRINTF("\r\n");
}
#endif /* STBOX1_ENABLE_PRINTF */

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
 STBOX1_PRINTF("User button pressed\r\n");
}

/**
  * @brief  function for sending one Acceleration hw event
  * @param  AccEventType Event type of Acceleration event
  * @retval None
  */
void  AccEvent_Msg(AccEventType Event)
{
  msgData_t msg;
  msg.type  = ACC;
  msg.acc   = Event;
  SendMsgToHost(&msg);
}

/**
  * @brief  function for sending the Steps counted
  * @param  uint16_t stepCnt number of steps counted
  * @retval None
  */
void  AccStepEvent_Msg(uint16_t stepCnt)
{
  msgData_t msg;
  msg.type    = ACC_STEP;
  msg.stepCnt = stepCnt;
  SendMsgToHost(&msg);
}

/**
  * @brief  Send Notification where there is a interrupt from LSM6DSOX
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  BSP_MOTION_SENSOR_Event_Status_t status;

  BSP_MOTION_SENSOR_Get_Event_Status(TargetBoardFeatures.HandleAccSensor,&status);

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    /* Check if the interrupt is due to Pedometer */
    if(status.StepStatus != 0) {
      PedometerStepCount = GetStepHWPedometer();
      if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
        AccStepEvent_Msg(PedometerStepCount);
      }
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    /* Check if the interrupt is due to Free Fall */
    if(status.FreeFallStatus != 0) {
      AccEvent_Msg(ACC_FREE_FALL);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    /* Check if the interrupt is due to Single Tap */
    if(status.TapStatus != 0) {
      AccEvent_Msg(ACC_SINGLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    /* Check if the interrupt is due to Double Tap */
    if(status.DoubleTapStatus != 0) {
      AccEvent_Msg(ACC_DOUBLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    /* Check if the interrupt is due to Tilt */
    if(status.TiltStatus != 0) {
      AccEvent_Msg(ACC_TILT);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    /* Check if the interrupt is due to 6D Orientation */
    if(status.D6DOrientationStatus != 0) {
      AccEventType Orientation = GetHWOrientation6D();
      AccEvent_Msg(Orientation);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    if(status.WakeUpStatus != 0) {
      AccEvent_Msg(ACC_WAKE_UP);
    }
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  BSP_MOTION_SENSOR_Axes_t ACC_Value;
  BSP_MOTION_SENSOR_Axes_t GYR_Value;
  BSP_MOTION_SENSOR_Axes_t MAG_Value;
  msgData_t msg;

  /* Read the Acc values */
  if(TargetBoardFeatures.HandleAccSensor != STBOX1_SNS_NOT_VALID ) {
    BSP_MOTION_SENSOR_GetAxes(TargetBoardFeatures.HandleAccSensor,MOTION_ACCELERO,&ACC_Value);
  } else {
    ACC_Value.x = ACC_Value.y = ACC_Value.z =0;
  }

  /* Read the Magneto values */
  if(TargetBoardFeatures.HandleMagSensor != STBOX1_SNS_NOT_VALID ) {
    BSP_MOTION_SENSOR_GetAxes(TargetBoardFeatures.HandleMagSensor,MOTION_MAGNETO,&MAG_Value);
  } else {
    MAG_Value.x = MAG_Value.y = MAG_Value.z =0;
  }

  /* Read the Gyro values */
  if(TargetBoardFeatures.HandleGyroSensor != STBOX1_SNS_NOT_VALID ) {
    BSP_MOTION_SENSOR_GetAxes(TargetBoardFeatures.HandleGyroSensor,MOTION_GYRO,&GYR_Value);
  } else {
    GYR_Value.x = GYR_Value.y = GYR_Value.z =0;
  }

  msg.type        = MOTION;
  msg.motion.acc  = ACC_Value ;
  msg.motion.gyr  = GYR_Value;
  msg.motion.mag  = MAG_Value;
  SendMsgToHost(&msg);
}

/**
* @brief  Callback function when 1ms PCM Audio is received from Microphones
* @param  none
* @retval None
*/
void AudioProcess_FromMics(void)
{
  if(AudioLevelTimerEnabled) {
    AudioProcess_DB_Noise();
  }
}

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess_DB_Noise(void)
{
  int32_t i;
  for(i = 0; i < 16; i++){
    int16_t IntPCM =(int16_t) PCM_Buffer[i];
    RMS_Ch += (float)(IntPCM*IntPCM);
  }
}

/**
  * @brief  Send Audio Level Data to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  uint16_t dB_Value;
  msgData_t msg;

  dB_Value = 0;

  RMS_Ch /= (16.0f*MICS_DB_UPDATE_MS);

  dB_Value = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (TargetBoardFeatures.AudioVolume - 4))) + 10.0f * log10f(RMS_Ch)) * 0.3f + dB_Value_Old * 0.7f);
  dB_Value_Old = dB_Value;
  RMS_Ch = 0.0f;

  msg.type  = AUDIO_LEV;
  msg.dB_Value[0] = dB_Value;
  SendMsgToHost(&msg);
}

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  uint32_t Instance Not used
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(uint32_t Instance)
{
  AudioProcess_FromMics();
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  uint32_t Instance Not used
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(uint32_t Instance)
{
  AudioProcess_FromMics();
}

/**
  * @brief  Manages the BSP audio in error event.
  * @param  Instance Audio in instance.
  * @retval None.
  */
void BSP_AUDIO_IN_Error_CallBack(uint32_t Instance)
{
  Error_Handler();
}


/**
  * @brief  Read The Environmental Data (Temperature/Pressure/Humidity)
  * @param  int32_t *PressToSend pointer to Press Value
  * @param  uint16_t *HumToSend  pointer to Humidity Value
  * @param  int16_t *TempToSend pointer to Temperature1 Value
  * @retval None
  */
void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *TempToSend)
{
  float SensorValue;
  int32_t decPart, intPart;
  
  *PressToSend=0;
  *HumToSend=0;
  *TempToSend=0;

  if(TargetBoardFeatures.HandleHumSensor != STBOX1_SNS_NOT_VALID) {
    /*Read Humidity and Temperature */
    BSP_ENV_SENSOR_GetValue(TargetBoardFeatures.HandleHumSensor,ENV_HUMIDITY,&SensorValue);
    MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
    *HumToSend = intPart*10+decPart;

#ifdef ONE_SHOT
    BSP_ENV_SENSOR_Set_One_Shot(TargetBoardFeatures.HandleHumSensor);
#endif
  }
  
  if(TargetBoardFeatures.HandleTempSensor != STBOX1_SNS_NOT_VALID) {
      BSP_ENV_SENSOR_GetValue(TargetBoardFeatures.HandleTempSensor,ENV_TEMPERATURE,&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      *TempToSend = intPart*10+decPart;

#ifdef ONE_SHOT
    BSP_ENV_SENSOR_Set_One_Shot(TargetBoardFeatures.HandleTempSensor);
#endif
  }
  
  if(TargetBoardFeatures.HandlePressSensor != STBOX1_SNS_NOT_VALID) {
    /* Read Pressure and Temperature */
    BSP_ENV_SENSOR_GetValue(TargetBoardFeatures.HandlePressSensor,ENV_PRESSURE,&SensorValue);
    MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
    *PressToSend=intPart*100+decPart;

#ifdef ONE_SHOT
    BSP_ENV_SENSOR_Set_One_Shot(TargetBoardFeatures.HandlePressSensor);
#endif
  }
}

/**
  * @brief  Send Environmental Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  msgData_t msg;

  /* Pressure,Humidity, and Temperatures*/
  if(EnvironmentalTimerEnabled) {
    int32_t PressToSend;
    uint16_t HumToSend;
    int16_t TempToSend;

    /* Read all the Environmental Sensors */
    ReadEnvironmentalData(&PressToSend,&HumToSend, &TempToSend);

    msg.type      = ENV;
    msg.env.press = PressToSend;
    msg.env.hum   = HumToSend;
    msg.env.temp  = TempToSend;
    SendMsgToHost(&msg);
  }
}

/**
  * @brief  Send Battery Info Data (Voltage/Current/Soc) to BLE
  * @param  None
  * @retval None
  */
static void SendBatteryInfoData(void)
{
  uint32_t soc_status;

  msgData_t msg;

  uint32_t voltage;
  uint32_t BatteryLevel;
  
  /* Read the Battery Charger voltage value */
  BSP_BC_GetVoltageAndLevel(&voltage,&BatteryLevel);
  
  /* The Charge Status is unknow because we could not use the the Interrupt 
     * for understanding the status change.
     * So it's possible to see the status only measuring the Voltage change */
   soc_status     = 0x04; /* Unknown */

  /* Battery Informations */
  if(BatteryTimerEnabled) {

    msg.type                   = BATTERY_INFO;
    msg.batteryInfo.soc_status = soc_status;
    msg.batteryInfo.voltage    = voltage;
    msg.batteryInfo.level      = BatteryLevel;

    SendMsgToHost(&msg);
  }
}


///** @brief Initialize the BlueNRG Stack
// * @param None
// * @retval None
// */
//static void Init_BlueNRG_Stack(void)
//{  
//  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
//  tBleStatus ret;
//  uint8_t data_len_out;
//
//  /* Initialize the BlueNRG HCI */
//  hci_init(APP_UserEvtRx, NULL);
//
//  /* we will let the BLE chip to use its Random MAC address */
//    
//#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
//  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, &data_len_out, bdaddr);
//
//  if(ret != BLE_STATUS_SUCCESS){
//    STBOX1_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
//    goto fail;
//  }
//  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
//                                  CONFIG_DATA_PUBADDR_LEN,
//                                  bdaddr);
//
//  if(ret != BLE_STATUS_SUCCESS){
//     STBOX1_PRINTF("\r\nSetting Public BD_ADDR failed\r\n");
//     goto fail;
//  }
//
//  ret = aci_gatt_init();
//  if(ret != BLE_STATUS_SUCCESS){
//     STBOX1_PRINTF("\r\nGATT_Init failed\r\n");
//     goto fail;
//  }
//
//  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, strlen(BoardName), &service_handle, &dev_name_char_handle, &appearance_char_handle);
//  
//
//  if(ret != BLE_STATUS_SUCCESS){
//     STBOX1_PRINTF("\r\nGAP_Init failed\r\n");
//     goto fail;
//  }
//
//#ifdef STBOX1_ENABLE_PRINTF
//  #ifdef STBOX1_RESTART_DFU
//    /* Ask if we want to boot in DFU mode */
//    {
//      int32_t Answer =0;
//      char Buffer[16];
//      uint32_t InitTick;
//
//      STBOX1_PRINTF("Do you want to reboot in DFU mode (y/n) [5sec]?");
//      InitTick = HAL_GetTick();
//      while(Answer==0) {
//        if(VCOM_read(Buffer, 16)) {
//          if(Buffer[0]=='y') {
//            DFU_Var = DFU_MAGIC_NUM;
//            HAL_NVIC_SystemReset();
//          } else {
//            STBOX1_PRINTF("\r\n\n");
//            Answer=1;
//          }
//        } else {
//          /* For avoding too much delay on board name introduction dialog */
//          CDC_PeriodElapsedCallback();
//          HAL_Delay(10);
//          if((HAL_GetTick()-InitTick)>5000) {
//            STBOX1_PRINTF("\r\n\tTimeOut\r\n");
//            Answer=1;
//          }
//        }
//      }
//    }
//  #endif /* STBOX1_RESTART_DFU */
//
//  /* Ask if we want to change the board Name (timeout of 5 seconds )*/
//  {
//    int32_t Answer =0;
//    char Buffer[16];
//    uint32_t InitTick;
//
//    STBOX1_PRINTF("Default BLE board Name [%s]\r\n",BoardName);
//    STBOX1_PRINTF("\tDo you want change it (y/n) [5sec]?");
//    InitTick = HAL_GetTick();
//    while(Answer==0) {
//      if(VCOM_read(Buffer, 16)) {
//        if(Buffer[0]=='y') {
//          int32_t NumBytes=0;
//          STBOX1_PRINTF("\r\n\tWrite the Name (7 Chars): _______\b\b\b\b\b\b\b");
//          while(NumBytes!=7) {
//            if(VCOM_read(Buffer, 16)) {
//              if(Buffer[0]!='\b') {
//                if(Buffer[0]=='\r') {
//                  for(;NumBytes<7;NumBytes++) {
//                    BoardName[NumBytes]=' ';
//                  }
//                } else {
//                  BoardName[NumBytes]=Buffer[0];
//                  NumBytes++;
//                }
//              } else {
//                if(NumBytes>0) {
//                  NumBytes--;
//                }
//              }
//            }
//          }
//          Answer=1;
//        } else {
//          STBOX1_PRINTF("\r\n\n");
//          Answer=1;
//        }
//      } else {
//        /* For avoding too much delay on board name introduction dialog */
//        CDC_PeriodElapsedCallback();
//        HAL_Delay(10);
//        if((HAL_GetTick()-InitTick)>5000) {
//          STBOX1_PRINTF("\r\n\tTimeOut\r\n");
//          Answer=1;
//        }
//      }
//    }
//  }
//#endif /* STBOX1_ENABLE_PRINTF */
//
//  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
//                                   strlen(BoardName), (uint8_t *)BoardName);
//
//  if(ret != BLE_STATUS_SUCCESS){
//     STBOX1_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
//    while(1);
//  }
//
//  ret = aci_gap_set_authentication_requirement(BONDING,
//                                               MITM_PROTECTION_REQUIRED,
//                                               SC_IS_SUPPORTED,
//                                               KEYPRESS_IS_NOT_SUPPORTED,
//                                               7, 
//                                               16,
//                                               USE_FIXED_PIN_FOR_PAIRING,
//                                               123456,
//                                               0x00);
//  if (ret != BLE_STATUS_SUCCESS) {
//     STBOX1_PRINTF("\r\nGAP setting Authentication failed\r\n");
//     goto fail;
//  }
//
//  STBOX1_PRINTF("\r\nSERVER: BLE Stack Initialized \r\n"
//         "\t\tBoardName= %s\r\n"
//         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n",
//         BoardName,
//         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
//
//  /* Set output power level */
//  aci_hal_set_tx_power_level(1,4); /* -2,1 dBm */
//
//  return;
//
//fail:
//  return;
//}

/** @brief HCI Transport layer user function
  * @param void *pData pointer to HCI event data
  * @retval None
  */
void APP_UserEvtRx(void *pData)
{
  uint32_t i;

  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;

  if(hci_pckt->type == HCI_EVENT_PKT) {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

    if(event_pckt->evt == EVT_LE_META_EVENT) {
      evt_le_meta_event *evt = (void *)event_pckt->data;

      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++) {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code) {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    } else if(event_pckt->evt == EVT_VENDOR) {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;        

      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++) {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code) {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    } else {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++) {
        if (event_pckt->evt == hci_events_table[i].evt_code) {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}

///** @brief Initialize all the Custom BlueNRG services
// * @param None
// * @retval None
// */
//static void Init_BlueNRG_Custom_Services(void)
//{
//  int ret;
//
//  ret = Add_HW_SW_ServW2ST_Service();
//  if(ret == BLE_STATUS_SUCCESS) {
//     STBOX1_PRINTF("HW & SW Service W2ST added successfully\r\n");
//  } else {
//     STBOX1_PRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
//  }
//
//  ret = Add_ConsoleW2ST_Service();
//  if(ret == BLE_STATUS_SUCCESS) {
//     STBOX1_PRINTF("Console Service W2ST added successfully\r\n");
//  } else {
//     STBOX1_PRINTF("\r\nError while adding Console Service W2ST\r\n");
//  }
//
//  ret = Add_ConfigW2ST_Service();
//  if(ret == BLE_STATUS_SUCCESS) {
//     STBOX1_PRINTF("Config  Service W2ST added successfully\r\n");
//  } else {
//     STBOX1_PRINTF("\r\nError while adding Config Service W2ST\r\n");
//  }
//}

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
    msgData_t msg;

  switch(GPIO_Pin){
    case HCI_TL_SPI_EXTI_PIN:
      hci_tl_lowlevel_isr();
      msg.type  = PROCESS_EVENT;
      SendMsgToHost(&msg);
    break;
    case USER_BUTTON_PIN:
    ButtonPressed = 1;
    if(semRun) {
      osSemaphoreRelease(semRun);
    }
    break;

    case POWER_BUTTON_PIN:
      /* Power off the board */
      PowerButtonPressed = 1;
      if(semRun) {
        osSemaphoreRelease(semRun);
      }
    break;

   /* HW events from LSM6DSOX */
   case GPIO_PIN_3:
    MEMSInterrupt=1;
    if(semRun) {
      osSemaphoreRelease(semRun);
    }
    break;
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

/**
  * @brief  Function for starting the ostimer for Led Blinking
  * @param  None
  * @retval None
  */
void LedBlinkStart(void)
{
  ledTimer = LED_TIME_ON;
  LED_ON_TARGET_PLATFORM();
  if (!timLedId) {
    timLedId     = osTimerCreate (osTimer(TimerLedHandle),osTimerOnce, NULL);
  }
  if (timLedId){
    if  (osTimerStart (timLedId, ledTimer) != osOK) {
      STBOX1_PRINTF("failed starting timer\n\r");
    }
  }
}

/**
  * @brief  Function for stopping the ostimer for Led Blinking
  * @param  None
  * @retval None
  */
void LedBlinkStop(void)
{
  LED_OFF_TARGET_PLATFORM();
  if (timLedId) {
    if  (osTimerStop (timLedId) != osOK){
      STBOX1_PRINTF("could not stop led timer\n\r");
    }
    if (osTimerDelete (timLedId) != osOK)  {
      STBOX1_PRINTF("could not delete led timer\n\r");
    }
  timLedId = NULL;
  ledTimer = (uint32_t)NULL;
  }
}

/**
  * @brief  Callback function called by Ostimer for LedBlinking
  * @param  void *arg pointer to the ostimer
  * @retval None
  */
static void LedBlinkCb  (void const *arg)
{
  if (ledTimer == LED_TIME_ON){
    ledTimer = LED_TIME_OFF;
    LED_OFF_TARGET_PLATFORM();
  } else {
    ledTimer = LED_TIME_ON;
    LED_ON_TARGET_PLATFORM();
  }
  if (timLedId){
    if  (osTimerStart (timLedId, ledTimer) != osOK){
      STBOX1_PRINTF("failed starting timer\n\r");
    }
  }
}

/**
  * @brief  Hook function for stack overflow
  * @param  None
  * @retval None
  */
void vApplicationStackOverflowHook (void)
{
  while (1)
  {

  }
}
