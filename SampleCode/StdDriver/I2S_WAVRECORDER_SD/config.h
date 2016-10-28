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
#define FSTLVL_BUFF_LEN     (1024 * 1)
/* Size of 2nd level buffer. */
#define SECLVL_BUFF_LEN     (1024 * 70)
/* Maximum write stride. Too large of this setting will lock 2nd level buffer on write to SD card. */
#define MAX_WRITE_STRIDE    (1024 * 8)
/* Support 24-bit sample size or not. */
#define SUPPORT_24BIT       0
#if SUPPORT_24BIT
/* Number of extra buffers for 24-bit sample size. Each such buffer will hold one DMA buffer data from 32-bit to 24-bit. */
#define NUM_24BIT_BUFF      (MAX_WRITE_STRIDE / FSTLVL_BUFF_LEN)
#endif  // #if SUPPORT_24BIT

#endif   //__CONFIG_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
