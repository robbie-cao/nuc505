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

extern int16_t PcmRxBuff[2][BUFF_LEN*2];
extern int16_t PcmTxBuff[2][BUFF_LEN*2];

extern volatile uint8_t aPCMBuffer_Full[2];

volatile static uint32_t s_flag1;

void I2S_IRQHandler(void)
{
    uint32_t u32I2SIntFlag;
    uint16_t i;

    u32I2SIntFlag = I2S_GET_INT_FLAG(I2S, I2S_STATUS_TDMATIF_Msk | I2S_STATUS_TDMAEIF_Msk | I2S_STATUS_RDMATIF_Msk | I2S_STATUS_RDMAEIF_Msk);

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

    /* Copy RX data to TX buffer */
    if (u32I2SIntFlag & I2S_STATUS_RDMATIF_Msk)
    {
        I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMATIF_Msk);
        /* Copy RX data to TX buffer */
        for (i = 0; i < BUFF_LEN; i++)
        {
            PcmTxBuff[0][i*2] = PcmRxBuff[0][i*2];
            //			PcmTxBuff[0][i*2] = 0;
            PcmTxBuff[0][i*2+1] = PcmRxBuff[0][i*2+1];
        }
    }
    else if (u32I2SIntFlag & I2S_STATUS_RDMAEIF_Msk)
    {
        I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMAEIF_Msk);
        if ( s_flag1 == 0 )
        {
            s_flag1 = 1;
            I2S_ENABLE_TXDMA(I2S);
            I2S_ENABLE_TX(I2S);
        }
        /* Copy RX data to TX buffer */
        for (i = 0; i < BUFF_LEN; i++)
        {
            PcmTxBuff[1][i*2] = PcmRxBuff[1][i*2];
            //			PcmTxBuff[1][i*2] = 0;
            PcmTxBuff[1][i*2+1] = PcmRxBuff[1][i*2+1];
        }
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
