/**
  ******************************************************************************
  * @file    DataLogExtended\Src\sensor_commands.c
  * @brief   Handle commands for sensor
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
#include "sensor_commands.h"
#include "main.h"
#include "demo_serial.h"
#include "vcom.h"

/** @addtogroup SENSORTILE_Examples
  * @{
  */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define SENSOR_NAME_MAX_LENGTH  10

/* Private macro -------------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
/* Currently selected sensor instances (defaults) */
uint32_t AccInstance = LSM6DSOX_0;
uint32_t GyrInstance = LSM6DSOX_0;
uint32_t MagInstance = LIS2MDL_0;
uint32_t HumInstance = HTS221_0;
uint32_t TmpInstance = HTS221_0;
uint32_t PrsInstance = LPS22HH_0;

/* Private variables ---------------------------------------------------------*/
/* Supported sensor names. Please verify that second index of array is HIGHER than longest string in array!!! */
static uint8_t AccNameList[][SENSOR_NAME_MAX_LENGTH] = {"LSM6DSOX", "LIS2DW12", "LIS3DHH"};
static uint8_t GyrNameList[][SENSOR_NAME_MAX_LENGTH] = {"LSM6DSOX"};
static uint8_t MagNameList[][SENSOR_NAME_MAX_LENGTH] = {"LIS2MDL"};
static uint8_t HumNameList[][SENSOR_NAME_MAX_LENGTH] = {"HTS221"};
static uint8_t TmpNameList[][SENSOR_NAME_MAX_LENGTH] = {"HTS221", "STTS751", "LPS22HH"};
static uint8_t PrsNameList[][SENSOR_NAME_MAX_LENGTH] = {"LPS22HH"};

/* Supported sensor instances (have to correspond with supported sensor names above) */
static uint32_t AccInstanceList[] = {LSM6DSOX_0, LIS2DW12_0, LIS3DHH_0};
static uint32_t GyrInstanceList[] = {LSM6DSOX_0};
static uint32_t MagInstanceList[] = {LIS2MDL_0};
static uint32_t HumInstanceList[] = {HTS221_0};
static uint32_t TmpInstanceList[] = {HTS221_0, STTS751_0, LPS22HH_0};
static uint32_t PrsInstanceList[] = {LPS22HH_0};

/* Sensor fullscale lists (have to correspond with supported sensor names above)
 * Please verify that second index of array is equal to or higher than count of longest sub-array items */
static uint32_t AccFsList[][5] =   /* g */
{
  {4, 2, 4, 8, 16}, /* LSM6DSOX */
  {4, 2, 4, 8, 16}, /* LIS2DW12 */
  {1, 3}, /* LIS3DHH */
};
static uint32_t GyrFsList[][5] =   /* dps */
{
  {4, 245, 500, 1000, 2000}, /* LSM6DSOX */
};
static uint32_t MagFsList[][2] =   /* Ga */
{
  {1, 50}, /* LIS2MDL */
};
static uint32_t HumFsList[][1] =   /* % */
{
  {0}, /* HTS221 */
};
static uint32_t TmpFsList[][1] =   /* C */
{
  {0}, /* HTS221 */
  {0}, /* STTS751 */
  {0}, /* LPS22HH */
};
static uint32_t PrsFsList[][1] =   /* Pa */
{
  {0}, /* LPS22HH */
};

/* Sensor output data rate lists (have to correspond with supported sensor names above)
 * Please verify that second index of array is equal to or higher than count of longest sub-array items */
static float AccOdrList[][11] =   /* Hz */
{
  {10, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667}, /* LSM6DSOX */
  {9, 1.6, 12.5, 25, 50, 100, 200, 400, 800, 1600},         /* LIS2DW12 */
  {1, 1100},                                                /* LIS3DHH */
};
static float GyrOdrList[][11] =   /* Hz */
{
  {10, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667}, /* LSM6DSOX */
};
static float MagOdrList[][5] =   /* Hz */
{
  {4, 10, 20, 50, 100}, /* LIS2MDL */
};
static float HumOdrList[][4] =   /* Hz */
{
  {3, 1, 7, 12.5}, /* HTS221 */
};
static float TmpOdrList[][11] =   /* Hz */
{
  {3, 1, 7, 12.5},                                    /* HTS221 */
  {10, 0.0625, 0.125, 0.25, 0.5, 1, 2, 4, 8, 16, 32}, /* STTS751 */
  {7, 1, 10, 25, 50, 75, 100, 200},                   /* LPS22HH */
};
static float PrsOdrList[][8] =   /* Hz */
{
  {7, 1, 10, 25, 50, 75, 100, 200}, /* LPS22HH */
};

/* Supported sensor names of same kind in one string (comma separated) */
static uint8_t AccNamesString[sizeof(AccNameList) + sizeof(AccNameList) / SENSOR_NAME_MAX_LENGTH];
static uint8_t GyrNamesString[sizeof(GyrNameList) + sizeof(GyrNameList) / SENSOR_NAME_MAX_LENGTH];
static uint8_t MagNamesString[sizeof(MagNameList) + sizeof(MagNameList) / SENSOR_NAME_MAX_LENGTH];
static uint8_t TmpNamesString[sizeof(TmpNameList) + sizeof(TmpNameList) / SENSOR_NAME_MAX_LENGTH];
static uint8_t HumNamesString[sizeof(HumNameList) + sizeof(HumNameList) / SENSOR_NAME_MAX_LENGTH];
static uint8_t PrsNamesString[sizeof(PrsNameList) + sizeof(PrsNameList) / SENSOR_NAME_MAX_LENGTH];

/* Currently selected sensor indexes */
static uint8_t AccIndex;
static uint8_t GyrIndex;
static uint8_t MagIndex;
static uint8_t HumIndex;
static uint8_t TmpIndex;
static uint8_t PrsIndex;

/* Private function prototypes -----------------------------------------------*/
static int32_t SC_Get_Sensor_Name(TMsg *Msg);
static int32_t SC_Read_Register(TMsg *Msg);
static int32_t SC_Write_Register(TMsg *Msg);
static int32_t SC_Get_Full_Scale_List(TMsg *Msg);
static int32_t SC_Set_Full_Scale(TMsg *Msg);
static int32_t SC_Get_Full_Scale(TMsg *Msg);
static int32_t SC_Get_ODR_List(TMsg *Msg);
static int32_t SC_Set_ODR(TMsg *Msg);
static int32_t SC_Get_ODR(TMsg *Msg);
static int32_t SC_Get_Sensor_List(TMsg *Msg);
static int32_t SC_Set_Sensor_Index(TMsg *Msg);

static void Send_Sensor_Name(TMsg *Msg, uint8_t *SensorName);
static void Send_Sensor_FS_List(TMsg *Msg, uint32_t *FsList);
static void Send_Sensor_ODR_List(TMsg *Msg, float *OdrList);

static void FloatToArray(uint8_t *Dest, float data);
static void ArrayToFloat(uint8_t *Source, float *data);

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Handle Sensors command
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int32_t Handle_Sensor_command(TMsg *Msg)
{
  int32_t ret;

  /* Commands */
  switch (Msg->Data[3])
  {
    case SC_GET_SENSOR_NAME:
      ret = SC_Get_Sensor_Name(Msg);
      break;

    case SC_READ_REGISTER:
      ret = SC_Read_Register(Msg);
      break;

    case SC_WRITE_REGISTER:
      ret = SC_Write_Register(Msg);
      break;

    case SC_GET_FULL_SCALE_LIST:
      ret = SC_Get_Full_Scale_List(Msg);
      break;

    case SC_SET_FULL_SCALE:
      ret = SC_Set_Full_Scale(Msg);
      break;

    case SC_GET_FULL_SCALE:
      ret = SC_Get_Full_Scale(Msg);
      break;

    case SC_GET_ODR_LIST:
      ret = SC_Get_ODR_List(Msg);
      break;

    case SC_SET_ODR:
      ret = SC_Set_ODR(Msg);
      break;

    case SC_GET_ODR:
      ret = SC_Get_ODR(Msg);
      break;

    case SC_GET_SENSOR_LIST:
      ret = SC_Get_Sensor_List(Msg);
      break;

    case SC_SET_SENSOR_INDEX:
      ret = SC_Set_Sensor_Index(Msg);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Get sensor name
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Get_Sensor_Name(TMsg *Msg)
{
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      Send_Sensor_Name(Msg, AccNameList[AccIndex]);
      break;

    case SC_GYROSCOPE:
      Send_Sensor_Name(Msg, GyrNameList[GyrIndex]);
      break;

    case SC_MAGNETOMETER:
      Send_Sensor_Name(Msg, MagNameList[MagIndex]);
      break;

    case SC_TEMPERATURE:
      Send_Sensor_Name(Msg, TmpNameList[TmpIndex]);
      break;

    case SC_HUMIDITY:
      Send_Sensor_Name(Msg, HumNameList[HumIndex]);
      break;

    case SC_PRESSURE:
      Send_Sensor_Name(Msg, PrsNameList[PrsIndex]);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
  * @brief  Read sensor register
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Read_Register(TMsg *Msg)
{
  uint8_t reg_value = 0;
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      if (BSP_MOTION_SENSOR_Read_Register(AccInstance, Msg->Data[5], &reg_value) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_GYROSCOPE:
      if (BSP_MOTION_SENSOR_Read_Register(GyrInstance, Msg->Data[5], &reg_value) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_MAGNETOMETER:
      if (BSP_MOTION_SENSOR_Read_Register(MagInstance, Msg->Data[5], &reg_value) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_TEMPERATURE:
      if (BSP_ENV_SENSOR_Read_Register(TmpInstance, Msg->Data[5], &reg_value) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_HUMIDITY:
      if (BSP_ENV_SENSOR_Read_Register(HumInstance, Msg->Data[5], &reg_value) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_PRESSURE:
      if (BSP_ENV_SENSOR_Read_Register(PrsInstance, Msg->Data[5], &reg_value) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    default:
      ret = 0;
      break;
  }

  if (ret == 0)
  {
    return ret;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Data[6] = reg_value;
  Msg->Len = 7;
  VCOM_send_MSG(Msg);
  return 1;
}

/**
  * @brief  Write to sensor register
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int32_t SC_Write_Register(TMsg *Msg)
{
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      if (BSP_MOTION_SENSOR_Write_Register(AccInstance, Msg->Data[5], Msg->Data[6]) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_GYROSCOPE:
      if (BSP_MOTION_SENSOR_Write_Register(GyrInstance, Msg->Data[5], Msg->Data[6]) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_MAGNETOMETER:
      if (BSP_MOTION_SENSOR_Write_Register(MagInstance, Msg->Data[5], Msg->Data[6]) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_TEMPERATURE:
      if (BSP_ENV_SENSOR_Write_Register(TmpInstance, Msg->Data[5], Msg->Data[6]) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_HUMIDITY:
      if (BSP_ENV_SENSOR_Write_Register(HumInstance, Msg->Data[5], Msg->Data[6]) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_PRESSURE:
      if (BSP_ENV_SENSOR_Write_Register(PrsInstance, Msg->Data[5], Msg->Data[6]) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    default:
      ret = 0;
      break;
  }

  if (ret == 0)
  {
    return ret;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 7;
  VCOM_send_MSG(Msg);
  return 1;
}

/**
  * @brief  Get sensor full scale list
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int32_t SC_Get_Full_Scale_List(TMsg *Msg)
{
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      Send_Sensor_FS_List(Msg, AccFsList[AccIndex]);
      break;

    case SC_GYROSCOPE:
      Send_Sensor_FS_List(Msg, GyrFsList[GyrIndex]);
      break;

    case SC_MAGNETOMETER:
      Send_Sensor_FS_List(Msg, MagFsList[MagIndex]);
      break;

    case SC_TEMPERATURE:
      Send_Sensor_FS_List(Msg, TmpFsList[TmpIndex]);
      break;

    case SC_HUMIDITY:
      Send_Sensor_FS_List(Msg, HumFsList[HumIndex]);
      break;

    case SC_PRESSURE:
      Send_Sensor_FS_List(Msg, PrsFsList[PrsIndex]);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
  * @brief  Set sensor full scale
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Set_Full_Scale(TMsg *Msg)
{
  uint32_t full_scale = Deserialize(&Msg->Data[5], 4);
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      if (BSP_MOTION_SENSOR_SetFullScale(AccInstance, MOTION_ACCELERO, (int32_t)full_scale) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_GYROSCOPE:
      if (BSP_MOTION_SENSOR_SetFullScale(GyrInstance, MOTION_GYRO, (int32_t)full_scale) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_MAGNETOMETER:
      if (BSP_MOTION_SENSOR_SetFullScale(MagInstance, MOTION_MAGNETO, (int32_t)full_scale) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_TEMPERATURE:
    case SC_HUMIDITY:
    case SC_PRESSURE:
      break;

    default:
      ret = 0;
      break;
  }

  if (ret == 0)
  {
    return ret;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 9;
  VCOM_send_MSG(Msg);
  return 1;
}

/**
  * @brief  Get sensor full scale
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Get_Full_Scale(TMsg *Msg)
{
  int32_t full_scale = 0;
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      if (BSP_MOTION_SENSOR_GetFullScale(AccInstance, MOTION_ACCELERO, &full_scale) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_GYROSCOPE:
      if (BSP_MOTION_SENSOR_GetFullScale(GyrInstance, MOTION_GYRO, &full_scale) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_MAGNETOMETER:
      if (BSP_MOTION_SENSOR_GetFullScale(MagInstance, MOTION_MAGNETO, &full_scale) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_TEMPERATURE:
    case SC_HUMIDITY:
    case SC_PRESSURE:
      break;

    default:
      ret = 0;
      break;
  }

  if (ret == 0)
  {
    return ret;
  }

  BUILD_REPLY_HEADER(Msg);
  Serialize(&Msg->Data[5], full_scale, 4);
  Msg->Len = 9;
  VCOM_send_MSG(Msg);
  return ret;
}

/**
  * @brief  Get sensor output data rate list
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Get_ODR_List(TMsg *Msg)
{
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      Send_Sensor_ODR_List(Msg, AccOdrList[AccIndex]);
      break;

    case SC_GYROSCOPE:
      Send_Sensor_ODR_List(Msg, GyrOdrList[GyrIndex]);
      break;

    case SC_MAGNETOMETER:
      Send_Sensor_ODR_List(Msg, MagOdrList[MagIndex]);
      break;

    case SC_TEMPERATURE:
      Send_Sensor_ODR_List(Msg, TmpOdrList[TmpIndex]);
      break;

    case SC_HUMIDITY:
      Send_Sensor_ODR_List(Msg, HumOdrList[HumIndex]);
      break;

    case SC_PRESSURE:
      Send_Sensor_ODR_List(Msg, PrsOdrList[PrsIndex]);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
  * @brief  Set sensor output data rate
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Set_ODR(TMsg *Msg)
{
  float odr = 0.0f;
  int32_t ret = 1;

  ArrayToFloat(&Msg->Data[5], &odr);

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      if (BSP_MOTION_SENSOR_SetOutputDataRate(AccInstance, MOTION_ACCELERO, odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_GYROSCOPE:
      if (BSP_MOTION_SENSOR_SetOutputDataRate(GyrInstance, MOTION_GYRO, odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_MAGNETOMETER:
      if (BSP_MOTION_SENSOR_SetOutputDataRate(MagInstance, MOTION_MAGNETO, odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_TEMPERATURE:
      if (BSP_ENV_SENSOR_SetOutputDataRate(TmpInstance, ENV_TEMPERATURE, odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_HUMIDITY:
      if (BSP_ENV_SENSOR_SetOutputDataRate(HumInstance, ENV_HUMIDITY, odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_PRESSURE:
      if (BSP_ENV_SENSOR_SetOutputDataRate(PrsInstance, ENV_PRESSURE, odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    default:
      ret = 0;
      break;
  }

  if (ret == 0)
  {
    return ret;
  }

  BUILD_REPLY_HEADER(Msg);
  Msg->Len = 9;
  VCOM_send_MSG(Msg);
  return 1;
}

/**
  * @brief  Get sensor output data rate
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Get_ODR(TMsg *Msg)
{
  float odr = 0.0f;
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      if (BSP_MOTION_SENSOR_GetOutputDataRate(AccInstance, MOTION_ACCELERO, &odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_GYROSCOPE:
      if (BSP_MOTION_SENSOR_GetOutputDataRate(GyrInstance, MOTION_GYRO, &odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_MAGNETOMETER:
      if (BSP_MOTION_SENSOR_GetOutputDataRate(MagInstance, MOTION_MAGNETO, &odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_TEMPERATURE:
      if (BSP_ENV_SENSOR_GetOutputDataRate(TmpInstance, ENV_TEMPERATURE, &odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_HUMIDITY:
      if (BSP_ENV_SENSOR_GetOutputDataRate(HumInstance, ENV_HUMIDITY, &odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    case SC_PRESSURE:
      if (BSP_ENV_SENSOR_GetOutputDataRate(PrsInstance, ENV_PRESSURE, &odr) != BSP_ERROR_NONE)
      {
        return 0;
      }
      break;

    default:
      ret = 0;
      break;
  }

  if (ret == 0)
  {
    return ret;
  }

  BUILD_REPLY_HEADER(Msg);
  FloatToArray(&Msg->Data[5], odr);
  Msg->Len = 9;
  VCOM_send_MSG(Msg);
  return ret;
}

/**
  * @brief  Get sensor list
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Get_Sensor_List(TMsg *Msg)
{
  int32_t ret = 1;
  int32_t i;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      /* Concatenate all sensor names of same kind to one string (comma separated) */
      for (i = 0; i < sizeof(AccNamesString); i++)
      {
        AccNamesString[i] = '\0';
      }
      (void)strcat((char *)AccNamesString, (char *)AccNameList[0]);
      for (i = 1; i < sizeof(AccNameList) / SENSOR_NAME_MAX_LENGTH; i++)
      {
        (void)strcat((char *)AccNamesString, ",");
        (void)strcat((char *)AccNamesString, (char *)AccNameList[i]);
      }
      Send_Sensor_Name(Msg, AccNamesString);
      break;

    case SC_GYROSCOPE:
      /* Concatenate all sensor names of same kind to one string (comma separated) */
      for (i = 0; i < sizeof(GyrNamesString); i++)
      {
        GyrNamesString[i] = '\0';
      }
      (void)strcat((char *)GyrNamesString, (char *)GyrNameList[0]);
      for (i = 1; i < sizeof(GyrNameList) / SENSOR_NAME_MAX_LENGTH; i++)
      {
        (void)strcat((char *)GyrNamesString, ",");
        (void)strcat((char *)GyrNamesString, (char *)GyrNameList[i]);
      }
      Send_Sensor_Name(Msg, GyrNamesString);
      break;

    case SC_MAGNETOMETER:
      /* Concatenate all sensor names of same kind to one string (comma separated) */
      for (i = 0; i < sizeof(MagNamesString); i++)
      {
        MagNamesString[i] = '\0';
      }
      (void)strcat((char *)MagNamesString, (char *)MagNameList[0]);
      for (i = 1; i < sizeof(MagNameList) / SENSOR_NAME_MAX_LENGTH; i++)
      {
        (void)strcat((char *)MagNamesString, ",");
        (void)strcat((char *)MagNamesString, (char *)MagNameList[i]);
      }
      Send_Sensor_Name(Msg, MagNamesString);
      break;

    case SC_TEMPERATURE:
      /* Concatenate all sensor names of same kind to one string (comma separated) */
      for (i = 0; i < sizeof(TmpNamesString); i++)
      {
        TmpNamesString[i] = '\0';
      }
      (void)strcat((char *)TmpNamesString, (char *)TmpNameList[0]);
      for (i = 1; i < sizeof(TmpNameList) / SENSOR_NAME_MAX_LENGTH; i++)
      {
        (void)strcat((char *)TmpNamesString, ",");
        (void)strcat((char *)TmpNamesString, (char *)TmpNameList[i]);
      }
      Send_Sensor_Name(Msg, TmpNamesString);
      break;

    case SC_HUMIDITY:
      /* Concatenate all sensor names of same kind to one string (comma separated) */
      for (i = 0; i < sizeof(HumNamesString); i++)
      {
        HumNamesString[i] = '\0';
      }
      (void)strcat((char *)HumNamesString, (char *)HumNameList[0]);
      for (i = 1; i < sizeof(HumNameList) / SENSOR_NAME_MAX_LENGTH; i++)
      {
        (void)strcat((char *)HumNamesString, ",");
        (void)strcat((char *)HumNamesString, (char *)HumNameList[i]);
      }
      Send_Sensor_Name(Msg, HumNamesString);
      break;

    case SC_PRESSURE:
      /* Concatenate all sensor names of same kind to one string (comma separated) */
      for (i = 0; i < sizeof(PrsNamesString); i++)
      {
        PrsNamesString[i] = '\0';
      }
      (void)strcat((char *)PrsNamesString, (char *)PrsNameList[0]);
      for (i = 1; i < sizeof(PrsNameList) / SENSOR_NAME_MAX_LENGTH; i++)
      {
        (void)strcat((char *)PrsNamesString, ",");
        (void)strcat((char *)PrsNamesString, (char *)PrsNameList[i]);
      }
      Send_Sensor_Name(Msg, PrsNamesString);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
  * @brief  Set sensor index
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
static int32_t SC_Set_Sensor_Index(TMsg *Msg)
{
  int32_t ret = 1;

  /* Sensor Type */
  switch (Msg->Data[4])
  {
    case SC_ACCELEROMETER:
      AccIndex = Msg->Data[5];
      if (AccInstance != AccInstanceList[AccIndex])
      {
        if (BSP_MOTION_SENSOR_Disable(AccInstance, MOTION_ACCELERO) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        if (BSP_MOTION_SENSOR_Init(AccInstanceList[AccIndex], MOTION_ACCELERO) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        AccInstance = AccInstanceList[AccIndex];
      }
      break;

    case SC_GYROSCOPE:
      GyrIndex = Msg->Data[5];
      if (GyrInstance != GyrInstanceList[GyrIndex])
      {
        if (BSP_MOTION_SENSOR_Disable(GyrInstance, MOTION_GYRO) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        if (BSP_MOTION_SENSOR_Init(GyrInstanceList[GyrIndex], MOTION_GYRO) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        GyrInstance = GyrInstanceList[GyrIndex];
      }
      break;

    case SC_MAGNETOMETER:
      MagIndex = Msg->Data[5];
      if (MagInstance != MagInstanceList[MagIndex])
      {
        if (BSP_MOTION_SENSOR_Disable(MagInstance, MOTION_MAGNETO) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        if (BSP_MOTION_SENSOR_Init(MagInstanceList[MagIndex], MOTION_MAGNETO) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        MagInstance = MagInstanceList[MagIndex];
      }
      break;

    case SC_TEMPERATURE:
      TmpIndex = Msg->Data[5];
      if (TmpInstance != TmpInstanceList[TmpIndex])
      {
        if (BSP_ENV_SENSOR_Disable(TmpInstance, ENV_TEMPERATURE) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        if (BSP_ENV_SENSOR_Init(TmpInstanceList[TmpIndex], ENV_TEMPERATURE) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        TmpInstance = TmpInstanceList[TmpIndex];
      }
      break;

    case SC_HUMIDITY:
      HumIndex = Msg->Data[5];
      if (HumInstance != HumInstanceList[HumIndex])
      {
        if (BSP_ENV_SENSOR_Disable(HumInstance, ENV_HUMIDITY) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        if (BSP_ENV_SENSOR_Init(HumInstanceList[HumIndex], ENV_HUMIDITY) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        HumInstance = HumInstanceList[HumIndex];
      }
      break;

    case SC_PRESSURE:
      PrsIndex = Msg->Data[5];
      if (PrsInstance != PrsInstanceList[PrsIndex])
      {
        if (BSP_ENV_SENSOR_Disable(PrsInstance, ENV_PRESSURE) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        if (BSP_ENV_SENSOR_Init(PrsInstanceList[PrsIndex], ENV_PRESSURE) != BSP_ERROR_NONE)
        {
          ret = 0;
        }
        PrsInstance = PrsInstanceList[PrsIndex];
      }
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
  * @brief  Send sensor name
  * @param  Msg the pointer to the message to be handled
  * @param  SensorName the sensor name
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
void Send_Sensor_Name(TMsg *Msg, uint8_t *sensor_name)
{
  int32_t i = 0;
  BUILD_REPLY_HEADER(Msg);

  while (i < strlen((char const *)sensor_name))
  {
    Msg->Data[5 + i] = sensor_name[i];
    i++;
  }

  Msg->Len = 5 + i;
  VCOM_send_MSG(Msg);
}

/**
  * @brief  Send sensor full scale list
  * @param  Msg the pointer to the message to be handled
  * @param  FsList the sensor full scale list
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
void Send_Sensor_FS_List(TMsg *Msg, uint32_t *fs_list)
{
  int32_t i = 0;
  BUILD_REPLY_HEADER(Msg);

  Serialize(&Msg->Data[5], fs_list[0], 4);

  for (i = 0; i < fs_list[0]; i++)
  {
    Serialize(&Msg->Data[9 + i * 4], fs_list[i + 1], 4);
  }

  Msg->Len = 9 + i * 4;
  VCOM_send_MSG(Msg);
}

/**
  * @brief  Send sensor output data rate list
  * @param  Msg the pointer to the message to be handled
  * @param  OdrList the sensor output data rate list
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
void Send_Sensor_ODR_List(TMsg *Msg, float *odr_list)
{
  int32_t i = 0;
  BUILD_REPLY_HEADER(Msg);

  Serialize(&Msg->Data[5], (int) odr_list[0], 4);

  for (i = 0; i < odr_list[0]; i++)
  {
    FloatToArray(&Msg->Data[9 + i * 4], odr_list[i + 1]);
  }

  Msg->Len = 9 + i * 4;
  VCOM_send_MSG(Msg);
}

/**
  * @brief  Copy float data to array
  * @param  Dest the pointer to the destination
  * @param  data the source data
  * @retval None
  */
void FloatToArray(uint8_t *Dest, float data)
{
  memcpy(Dest, (void *) &data, 4);
}

/**
  * @brief  Copy float data from array
  * @param  Source the pointer to the source data
  * @param  data the pointer to the destination
  * @retval None
  */
void ArrayToFloat(uint8_t *Source, float *data)
{
  memcpy((void *) data, Source, 4);
}

/**
  * @}
  */

/**
  * @}
  */

