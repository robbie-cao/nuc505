/******************************************************************************
 * @file     nuc505_isr.c
 * @version  V0.30
 * $Revision: 4 $
 * $Date: 14/11/19 3:39a $
 * @brief    NUC505 series ISR source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"
#include "ringbuff.h"

extern uint8_t PcmRxBuff[2][FSTLVL_BUFF_LEN];
extern int volatile nBuffOverRun;
extern struct RingBuff audio_rb;

void I2S_IRQHandler(void)
{
	uint32_t u32I2SIntFlag;

	u32I2SIntFlag = I2S_GET_INT_FLAG(I2S, (I2S_STATUS_RDMATIF_Msk | I2S_STATUS_RDMAEIF_Msk));

	/* Copy RX data to TX buffer */
	if (u32I2SIntFlag & I2S_STATUS_RDMATIF_Msk)
	{
		I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMATIF_Msk);

        {
            void *next_wrt_p;
            unsigned next_wrt_cap;
            rb_next_write(&audio_rb, &next_wrt_p, &next_wrt_cap);
            if (next_wrt_cap < sizeof (PcmRxBuff[0])) {
                nBuffOverRun ++;
            }
            else {
                memcpy(next_wrt_p, &PcmRxBuff[0][0], sizeof (PcmRxBuff[0]));
                rb_write_done(&audio_rb, sizeof (PcmRxBuff[0]));
            }
        }
	}
	else if (u32I2SIntFlag & I2S_STATUS_RDMAEIF_Msk)
	{
		I2S_CLR_INT_FLAG(I2S, I2S_STATUS_RDMAEIF_Msk);
		
		{
            void *next_wrt_p;
            unsigned next_wrt_cap;
            rb_next_write(&audio_rb, &next_wrt_p, &next_wrt_cap);
            if (next_wrt_cap < sizeof (PcmRxBuff[1])) {
                nBuffOverRun ++;
            }
            else {
                memcpy(next_wrt_p, &PcmRxBuff[1][0], sizeof (PcmRxBuff[1]));
                rb_write_done(&audio_rb, sizeof (PcmRxBuff[1]));
            }
        }
	}
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
