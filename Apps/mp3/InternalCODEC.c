/**************************************************************************//**
 * @file     InternalCODEC.c
 * @version  V1.1
 * $Revision: 3 $
 * $Date: 16/01/09 3:39p $
 * @brief    NUC505 I2S Driver Sample Code
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

void SysTick_Handler(void)
{

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Internal CODEC Settings                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void InternalCODEC_Setup(void)
{
    uint32_t i;

    printf("\nConfigure Internal CODEC ...");

    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
    I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	// Set CODEC slave
    I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
    for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
        CLK_SysTickDelay(100000);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
    I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
    CLK_SysTickDelay(100);	//Delay 100uS
    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x16);	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x16);	//Un-mute Headphone and set volume

    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */

    printf("[OK]\n");
}
