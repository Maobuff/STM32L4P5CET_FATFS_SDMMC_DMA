/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    user_diskio.c
 * @brief   This file includes a diskio driver skeleton to be completed by the user.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "user_diskio.h"

#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2

#define SD_TIMEOUT 30 * 1000

#define SD_DEFAULT_BLOCK_SIZE 512
/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
osMessageQueueId_t SDQueueID = NULL;
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	Stat = STA_NOINIT;
#if (osCMSIS <= 0x20000U)
  if(osKernelRunning())
#else
	if (osKernelGetState() == osKernelRunning)
#endif
	    {
#if !defined(DISABLE_SD_INIT)

		if (BSP_SD_Init() == MSD_OK) {
			Stat = SD_CheckStatus(pdrv);
		}

#else
    Stat = SD_CheckStatus(pdrv);
#endif

		/*
		 * if the SD is correctly initialized, create the operation queue
		 * if not already created
		 */

		if (Stat != STA_NOINIT) {
			if (SDQueueID == NULL) {
#if (osCMSIS <= 0x20000U)
      osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
      SDQueueID = osMessageCreate (osMessageQ(SD_Queue), NULL);
#else
				SDQueueID = osMessageQueueNew(QUEUE_SIZE, 2, NULL);
#endif
			}

			if (SDQueueID == NULL) {
				Stat |= STA_NOINIT;
			}
		}
	}
	return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	return SD_CheckStatus(pdrv);
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	DRESULT res = RES_ERROR;
	uint32_t timer;
#if (osCMSIS < 0x20000U)
	  osEvent event;
	#else
	uint16_t event;
	osStatus_t status;
#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
	  uint32_t alignedAddr;
	#endif
	/*
	 * ensure the SDCard is ready for a new operation
	 */

	if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0) {
		return res;
	}

#if defined(ENABLE_SCRATCH_BUFFER)
	  if (!((uint32_t)buff & 0x3))
	  {
	#endif
	/* Fast path cause destination buffer is correctly aligned */
	uint8_t ret = BSP_SD_ReadBlocks_DMA((uint32_t*) buff, (uint32_t) (sector), count);

	if (ret == MSD_OK) {
#if (osCMSIS < 0x20000U)
	    /* wait for a message from the queue or a timeout */
	    event = osMessageGet(SDQueueID, SD_TIMEOUT);

	    if (event.status == osEventMessage)
	    {
	      if (event.value.v == READ_CPLT_MSG)
	      {
	        timer = osKernelSysTick();
	        /* block until SDIO IP is ready or a timeout occur */
	        while(osKernelSysTick() - timer <SD_TIMEOUT)
	#else
		status = osMessageQueueGet(SDQueueID, (void*) &event, NULL, SD_TIMEOUT);
		if ((status == osOK) && (event == READ_CPLT_MSG )) {
			timer = osKernelGetTickCount();
			/* block until SDIO IP is ready or a timeout occur */
			while (osKernelGetTickCount() - timer < SD_TIMEOUT)
#endif
			{
				if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
					res = RES_OK;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
	                /*
	                the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
	                adjust the address and the D-Cache size to invalidate accordingly.
	                */
	                alignedAddr = (uint32_t)buff & ~0x1F;
	                SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
	#endif
					break;
				}
			}
#if (osCMSIS < 0x20000U)
	          }
	        }
	#else
		}
#endif
	}

#if defined(ENABLE_SCRATCH_BUFFER)
	    }
	    else
	    {
	      /* Slow path, fetch each sector a part and memcpy to destination buffer */
	      int i;

	      for (i = 0; i < count; i++)
	      {
	        ret = BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
	        if (ret == MSD_OK )
	        {
	          /* wait until the read is successful or a timeout occurs */
	#if (osCMSIS < 0x20000U)
	          /* wait for a message from the queue or a timeout */
	          event = osMessageGet(SDQueueID, SD_TIMEOUT);

	          if (event.status == osEventMessage)
	          {
	            if (event.value.v == READ_CPLT_MSG)
	            {
	              timer = osKernelSysTick();
	              /* block until SDIO IP is ready or a timeout occur */
	              while(osKernelSysTick() - timer <SD_TIMEOUT)
	#else
	                status = osMessageQueueGet(SDQueueID, (void *)&event, NULL, SD_TIMEOUT);
	              if ((status == osOK) && (event == READ_CPLT_MSG))
	              {
	                timer = osKernelGetTickCount();
	                /* block until SDIO IP is ready or a timeout occur */
	                ret = MSD_ERROR;
	                while(osKernelGetTickCount() - timer < SD_TIMEOUT)
	#endif
	                {
	                  ret = BSP_SD_GetCardState();

	                  if (ret == MSD_OK)
	                  {
	                    break;
	                  }
	                }

	                if (ret != MSD_OK)
	                {
	                  break;
	                }
	#if (osCMSIS < 0x20000U)
	              }
	            }
	#else
	          }
	#endif
	#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
	          /*
	          *
	          * invalidate the scratch buffer before the next read to get the actual data instead of the cached one
	          */
	          SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
	#endif
	          memcpy(buff, scratch, BLOCKSIZE);
	          buff += BLOCKSIZE;
	        }
	        else
	        {
	          break;
	        }
	      }

	      if ((i == count) && (ret == MSD_OK ))
	        res = RES_OK;
	    }
	#endif
	return res;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
	/* USER CODE HERE */
	DRESULT res = RES_ERROR;
	uint32_t timer;

#if (osCMSIS < 0x20000U)
	  osEvent event;
	#else
	uint16_t event;
	osStatus_t status;
#endif

#if defined(ENABLE_SCRATCH_BUFFER)
	  int32_t ret;
	#endif

	/*
	 * ensure the SDCard is ready for a new operation
	 */

	if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0) {
		return res;
	}

#if defined(ENABLE_SCRATCH_BUFFER)
	  if (!((uint32_t)buff & 0x3))
	  {
	#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
	  uint32_t alignedAddr;
	  /*
	    the SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
	    adjust the address and the D-Cache size to clean accordingly.
	  */
	  alignedAddr = (uint32_t)buff & ~0x1F;
	  SCB_CleanDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
	#endif

	if (BSP_SD_WriteBlocks_DMA((uint32_t*) buff, (uint32_t) (sector), count) == MSD_OK) {
#if (osCMSIS < 0x20000U)
	    /* Get the message from the queue */
	    event = osMessageGet(SDQueueID, SD_TIMEOUT);

	    if (event.status == osEventMessage)
	    {
	      if (event.value.v == WRITE_CPLT_MSG)
	      {
	#else
		status = osMessageQueueGet(SDQueueID, (void*) &event, NULL, SD_TIMEOUT);
		if ((status == osOK) && (event == WRITE_CPLT_MSG )) {
#endif
#if (osCMSIS < 0x20000U)
	        timer = osKernelSysTick();
	        /* block until SDIO IP is ready or a timeout occur */
	        while(osKernelSysTick() - timer  < SD_TIMEOUT)
	#else
			timer = osKernelGetTickCount();
			/* block until SDIO IP is ready or a timeout occur */
			while (osKernelGetTickCount() - timer < SD_TIMEOUT)
#endif
			{
				if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
					res = RES_OK;
					break;
				}
			}
#if (osCMSIS < 0x20000U)
	      }
	    }
	#else
		}
#endif
	}
#if defined(ENABLE_SCRATCH_BUFFER)
	  else {
	    /* Slow path, fetch each sector a part and memcpy to destination buffer */
	    int i;

	#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
	    /*
	     * invalidate the scratch buffer before the next write to get the actual data instead of the cached one
	     */
	     SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
	#endif
	      for (i = 0; i < count; i++)
	      {
	        memcpy((void *)scratch, buff, BLOCKSIZE);
	        buff += BLOCKSIZE;

	        ret = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
	        if (ret == MSD_OK )
	        {
	          /* wait until the read is successful or a timeout occurs */
	#if (osCMSIS < 0x20000U)
	          /* wait for a message from the queue or a timeout */
	          event = osMessageGet(SDQueueID, SD_TIMEOUT);

	          if (event.status == osEventMessage)
	          {
	            if (event.value.v == READ_CPLT_MSG)
	            {
	              timer = osKernelSysTick();
	              /* block until SDIO IP is ready or a timeout occur */
	              while(osKernelSysTick() - timer <SD_TIMEOUT)
	#else
	                status = osMessageQueueGet(SDQueueID, (void *)&event, NULL, SD_TIMEOUT);
	              if ((status == osOK) && (event == READ_CPLT_MSG))
	              {
	                timer = osKernelGetTickCount();
	                /* block until SDIO IP is ready or a timeout occur */
	                ret = MSD_ERROR;
	                while(osKernelGetTickCount() - timer < SD_TIMEOUT)
	#endif
	                {
	                  ret = BSP_SD_GetCardState();

	                  if (ret == MSD_OK)
	                  {
	                    break;
	                  }
	                }

	                if (ret != MSD_OK)
	                {
	                  break;
	                }
	#if (osCMSIS < 0x20000U)
	              }
	            }
	#else
	          }
	#endif
	        }
	        else
	        {
	          break;
	        }
	      }

	      if ((i == count) && (ret == MSD_OK ))
	        res = RES_OK;
	    }

	  }
	#endif

	return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	DRESULT res = RES_ERROR;
	BSP_SD_CardInfo CardInfo;

	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	switch (cmd) {
		/* Make sure that no pending write process */
		case CTRL_SYNC:
			res = RES_OK;
		break;

			/* Get number of sectors on the disk (DWORD) */
		case GET_SECTOR_COUNT:
			BSP_SD_GetCardInfo(&CardInfo);
			*(DWORD*) buff = CardInfo.LogBlockNbr;
			res = RES_OK;
		break;

			/* Get R/W sector size (WORD) */
		case GET_SECTOR_SIZE:
			BSP_SD_GetCardInfo(&CardInfo);
			*(WORD*) buff = CardInfo.LogBlockSize;
			res = RES_OK;
		break;

			/* Get erase block size in unit of sector (DWORD) */
		case GET_BLOCK_SIZE:
			BSP_SD_GetCardInfo(&CardInfo);
			*(DWORD*) buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
			res = RES_OK;
		break;

		default:
			res = RES_PARERR;
	}

	return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
