/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various existing       */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NUC505Series.h"
#include "diskio.h"     /* FatFs lower layer API */

#define SUPPORT_SD

/* Definitions of physical drive number for each media */
#define DRV_SD0     0

#define Sector_Size 128
uint32_t Tmp_Buffer[Sector_Size];

extern DISK_DATA_T SD_DiskInfo0;
extern int  SD_Open_(uint32_t cardSel);
extern void SD_Close_(uint32_t cardSel);


/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv       /* Physical drive number (0..) */
)
{
    switch (pdrv) {
#ifdef SUPPORT_SD
    case DRV_SD0 :
        SD_Open_(SD_PORT0 | CardDetect_From_GPIO);
        return 0;
#endif
    }
    return RES_PARERR;
}


/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv       /* Physical drive number (0..) */
)
{
    switch (pdrv) {
#ifdef SUPPORT_SD
    case DRV_SD0 :
        return RES_OK;
#endif
    }
    return STA_NOINIT;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Sector address (LBA) */
    //BYTE count      /* Number of sectors to read (1..128) */    // for FATFS v0.09
    UINT count      /* Number of sectors to read (1..128) */    // for FATFS v0.11
)
{
    uint32_t status = Successful, status1 = Successful;
    uint32_t shift_buf_flag = 0;
    uint32_t tmp_StartBufAddr;

    if(count == 0)
    {
        return RES_PARERR;
    }

    if((uint32_t)buff%4)
    {
        shift_buf_flag = 1;
    }

    switch (pdrv) {
#ifdef SUPPORT_SD
    case DRV_SD0 :

        SD->GCTL = SDH_GCTL_SDEN_Msk;

        if(shift_buf_flag == 1)
        {
            if(count == 1)
            {
                status = SD_Read(SD_PORT0, (uint8_t*)(&Tmp_Buffer), sector, count);
                memcpy(buff, (&Tmp_Buffer), SD_DiskInfo0.sectorSize*count);
            }
            else
            {
                tmp_StartBufAddr = (((uint32_t)buff/4 + 1) * 4);
                status = SD_Read(SD_PORT0, ((uint8_t*)tmp_StartBufAddr), sector, (count -1));
                memcpy(buff, (void*)tmp_StartBufAddr, (SD_DiskInfo0.sectorSize*(count-1)) );
                status1 = SD_Read(SD_PORT0, (uint8_t*)(&Tmp_Buffer), (sector+count-1), 1);
                memcpy( (buff+(SD_DiskInfo0.sectorSize*(count-1))), (void*)Tmp_Buffer, SD_DiskInfo0.sectorSize);
            }
        }
        else
            status = SD_Read(SD_PORT0, (uint8_t*)buff, sector, count);

        if ((status != Successful) || (status1 != Successful))
            return RES_ERROR;

        return RES_OK;
#endif
    }
    return RES_PARERR;
}


/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
    BYTE pdrv,          /* Physical drive number (0..) */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Sector address (LBA) */
    //BYTE count          /* Number of sectors to write (1..128) */   // for FATFS v0.09
    UINT count          /* Number of sectors to read (1..128) */    // for FATFS v0.11
)
{
    int status = RES_OK, status1 = RES_OK;
    uint32_t shift_buf_flag = 0;
    uint32_t tmp_StartBufAddr;
    uint32_t i;

    if((uint32_t)buff%4)
    {
        shift_buf_flag = 1;
    }

    switch (pdrv) {
#ifdef SUPPORT_SD
    case DRV_SD0 :

        SD->GCTL = SDH_GCTL_SDEN_Msk;

        if(shift_buf_flag == 1)
        {
            if(count == 1)
            {
                memcpy((&Tmp_Buffer), buff, count);
                status = SD_Write(SD_PORT0, (uint8_t*)(&Tmp_Buffer), sector, SD_DiskInfo0.sectorSize*count);
            }
            else
            {
                tmp_StartBufAddr = (((uint32_t)buff/4 + 1) * 4);

                memcpy((void*)Tmp_Buffer, (buff+(SD_DiskInfo0.sectorSize*(count-1))), SD_DiskInfo0.sectorSize);

                for(i = (SD_DiskInfo0.sectorSize*(count-1)); i > 0; i--)
                {
                    memcpy((void *)(tmp_StartBufAddr + i - 1), (buff + i -1), 1);
                }

                status = SD_Write(SD_PORT0, ((uint8_t*)tmp_StartBufAddr), sector, (count -1));
                status1 = SD_Write(SD_PORT0, (uint8_t*)(&Tmp_Buffer), (sector+count-1), 1);
            }
        }
        else
            status = SD_Write(SD_PORT0, (uint8_t*)buff, sector, count);

        if ((status != Successful) || (status1 != Successful))
            return RES_ERROR;

        return RES_OK;
#endif
    }
    return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    DRESULT res = RES_OK;

    switch (pdrv) {
#ifdef SUPPORT_SD
    case DRV_SD0 :
        switch(cmd) {
        case CTRL_SYNC:
            break;
        case GET_SECTOR_COUNT:
            *(DWORD*)buff = SD_DiskInfo0.totalSectorN;
            break;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = SD_DiskInfo0.sectorSize;
            break;
        default:
            res = RES_PARERR;
            break;
        }
        break;
#endif

    default:
        res = RES_PARERR;
        break;
    }
    return res;
}
#endif
