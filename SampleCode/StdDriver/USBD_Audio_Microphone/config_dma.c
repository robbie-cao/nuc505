/******************************************************************************
 * @file     config_dma.c
 * @brief    NuMicro series USBD driver Sample file
 * @version  V2.0  
 * @date     2015/09/23 07:15 p.m.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC505Series.h"
#include "usbd_audio.h"

extern int16_t PcmRecBuff[RING_BUF_SMPL_CNT];

/* FIXME Configure DMA */
void DMA_Init(void)
{
	I2S_SET_RXDMA_STADDR(I2S, (uint32_t) &PcmRecBuff[0]);										// Rx Start Address
	I2S_SET_RXDMA_EADDR(I2S, (uint32_t) &PcmRecBuff[RING_BUF_SMPL_CNT-2]);	// Rx End Address
}
