/******************************************************************************
 * @file     nuc505_isr.c
 * @version  V0.10
 * $Revision: 4 $
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 ISR source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"

extern volatile uint8_t aPCMBuffer_Full[2];

void I2S_IRQHandler(void)
{
		uint32_t u32I2SIntFlag;

    u32I2SIntFlag = I2S_GET_INT_FLAG(I2S, I2S_STATUS_TDMATIF_Msk | I2S_STATUS_TDMAEIF_Msk);
		
		if (u32I2SIntFlag & I2S_STATUS_TDMATIF_Msk)
    {
				I2S_CLR_INT_FLAG(I2S, I2S_STATUS_TDMATIF_Msk);
        aPCMBuffer_Full[0] = 0;
    }
    else if (u32I2SIntFlag & I2S_STATUS_TDMAEIF_Msk)
    {
				I2S_CLR_INT_FLAG(I2S, I2S_STATUS_TDMAEIF_Msk);
        aPCMBuffer_Full[1] = 0;
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
