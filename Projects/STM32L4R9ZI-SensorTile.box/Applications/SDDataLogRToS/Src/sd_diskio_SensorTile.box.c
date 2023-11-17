/**
  ******************************************************************************
  * @file    SDDataLogRToS\Src\sd_diskio_SensorTile.box.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V1.6.0
  * @date    20-October-2023
  * @brief   SD Disk I/O driver for SensorTile.box
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
#include "ff_gen_drv.h"
#include "sd_diskio_SensorTile.box.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*
* the following Timeout is useful to give the control back to the applications
* in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
* the value by default is as defined in the BSP platform driver otherwise 30 secs
*/
#define SD_TIMEOUT 30 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512

#define _USE_WRITE  1 /* 1: Enable disk_write function */
#define _USE_IOCTL  1 /* 1: Enable disk_ioctl function */


/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static volatile  UINT  WriteStatus = 0, ReadStatus = 0;
/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize(BYTE);
DSTATUS SD_status(BYTE);
DRESULT SD_read(BYTE, BYTE *, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write(BYTE, const BYTE *, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE, BYTE, void *);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if (BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
  Stat = STA_NOINIT;
#if !defined(DISABLE_SD_INIT)

  if (BSP_SD_Init() == MSD_OK)
  {
    Stat = SD_CheckStatus(lun);
  }

#else /* !defined(DISABLE_SD_INIT) */
  Stat = SD_CheckStatus(lun);
#endif /* !defined(DISABLE_SD_INIT) */
  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  ReadStatus = 0;
  uint32_t timeout;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
#endif /* (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1) */

  if (BSP_SD_ReadBlocks_DMA((uint32_t *)buff,
                            (uint32_t)(sector),
                            count) == MSD_OK)
  {
    /* Wait that the reading process is completed or a timeout occurs */
    timeout = HAL_GetTick();
    while ((ReadStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
    {
      ;
    }
    /* In case of a timeout return error */
    if (ReadStatus == 0)
    {
      res = RES_ERROR;
    }
    else
    {
      ReadStatus = 0;
      timeout = HAL_GetTick();

      while ((HAL_GetTick() - timeout) < SD_TIMEOUT)
      {
        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
        {
          res = RES_OK;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
          /*
             the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
             adjust the address and the D-Cache size to invalidate accordingly.
           */
          alignedAddr = (uint32_t)buff & ~0x1F;
          SCB_InvalidateDCache_by_Addr((uint32_t *)alignedAddr, count * BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif /* (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1) */
          break;
        }
      }
    }
  }

  return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  WriteStatus = 0;
  uint32_t timeout;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
  /*
   the SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
   adjust the address and the D-Cache size to clean accordingly.
   */
  alignedAddr = (uint32_t)buff &  ~0x1F;
  SCB_CleanDCache_by_Addr((uint32_t *)alignedAddr, count * BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif /* (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1) */

  if (BSP_SD_WriteBlocks_DMA((uint32_t *)buff,
                             (uint32_t)(sector),
                             count) == MSD_OK)
  {
    /* Wait that writing process is completed or a timeout occurs */

    timeout = HAL_GetTick();
    while ((WriteStatus == 0) && ((HAL_GetTick() - timeout) < SD_TIMEOUT))
    {
      ;
    }
    /* In case of a timeout return error */
    if (WriteStatus == 0)
    {
      res = RES_ERROR;
    }
    else
    {
      WriteStatus = 0;
      timeout = HAL_GetTick();

      while ((HAL_GetTick() - timeout) < SD_TIMEOUT)
      {
        if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
        {
          res = RES_OK;
          break;
        }
      }
    }
  }

  return res;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) { return RES_NOTRDY; }

  switch (cmd)
  {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
      res = RES_OK;
      break;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
      BSP_SD_GetCardInfo(&CardInfo);
      *(DWORD *)buff = CardInfo.LogBlockNbr;
      res = RES_OK;
      break;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
      BSP_SD_GetCardInfo(&CardInfo);
      *(WORD *)buff = CardInfo.LogBlockSize;
      res = RES_OK;
      break;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
      BSP_SD_GetCardInfo(&CardInfo);
      *(DWORD *)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void)
{
  WriteStatus = 1;
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void)
{
  ReadStatus = 1;
}

