/******************************************************************************
 * @file     nuc505_isr.c
 * @version  V0.4
 * $Revision: 5 $
 * $Date: 14/11/27 3:18p $
 * @brief    NUC505 series ISR source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"
//#include "ringbuff.h"

extern volatile uint8_t PcmRxBuffFull[2];
//extern uint8_t PcmRxBuff[2][];

void I2S_IRQHandler(void)
{
	uint32_t u32I2SIntFlag;

	u32I2SIntFlag = I2S_GET_INT_FLAG(I2S, (I2S_STATUS_RDMATIF_Msk | I2S_STATUS_RDMAEIF_Msk));

	/* Copy RX data to TX buffer */
	if (u32I2SIntFlag & I2S_STATUS_RDMATIF_Msk)
	{
		I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMATIF_Msk);
		PcmRxBuffFull[0] = TRUE;
	}
	else if (u32I2SIntFlag & I2S_STATUS_RDMAEIF_Msk)
	{
		I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMAEIF_Msk);
		PcmRxBuffFull[1] = TRUE;
	}
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
