/******************************************************************************
 * @file     config.h
 * @brief    NUC505 I2S Driver Sample header file
 * @version  V1.0
 * $Revision: 4 $
 * $Date: 14/11/26 1:36p $
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CONFIG_H__
#define __CONFIG_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* Size of one of two audio DMA buffers for ping-pong buffering. */
#define FSTLVL_BUFF_LEN     (160 * 2)
/* Size of 2nd level buffer. */
#define SECLVL_BUFF_LEN     (1024 * 16)
/* Maximum write stride. Too large of this setting will lock 2nd level buffer on write to SD card. */
#define MAX_WRITE_STRIDE    (1024 * 8)

#endif   //__CONFIG_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
