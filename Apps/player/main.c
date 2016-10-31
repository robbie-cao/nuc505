/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 11 $
 * $Date: 15/09/01 02:41p $
 * @brief    A WAV file player demo using internal audio codec used to playback WAV file stored in SD card.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"

FATFS FatFs[_VOLUMES];      /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;       /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */
#endif

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}

void SysTick_Handler(void)
{

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Internal CODEC Setting with I2C interface                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void InternalCODEC_Setup()
{
    uint32_t i;

    printf("\nConfigure Internal CODEC ...");

    /* IIC Configure Step without PLL: */
    /* Add MCLK(256*Fs) in. */

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
    I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x06);	//Un-mute Headphone and set volume
    I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x06);	//Un-mute Headphone and set volume

    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */

    printf("[OK]\n");
}

//---------------------------------------------------------
//--- Initial SD0 multi-function GPIO pin
//
// NUC505 support 3 groups of GPIO pins and SD sockets for same one SD port.
// Please select ONLY ONE configuration from them.
// 1. SD-A socket on daughter board + default SD0_Init(). (Default)
// 2. SD-B socket on main board + short JP3 and JP4
//    + define compile flag "SDH_GPIO_GB" in SD0_Init().
//    (Note: this configuration conflict with UART1)
// 3. SD-C socket on main board + short JP3 and JP2
//    + define compile flag "SDH_GPIO_GA" in SD0_Init()
//    (Note: this configuration conflict with UART0)
//---------------------------------------------------------
void SD0_Init(void)
{
#ifdef SDH_GPIO_GA
    // The group A are GPA10~11, GPA13~15, GPB0~1
    // Conflict with UART0
    // printf("SD_Open(): Configure GPIO group A as SDH pins.\n");
    SYS->GPA_MFPH &= (~0x77707700);
    SYS->GPA_MFPH |=   0x44404400;
    SYS->GPA_MFPH &= (~0x00000077);
    SYS->GPB_MFPL |=   0x00000044;

#elif defined SDH_GPIO_GB
    // The group B are GPB2~3, GPB5~9
    // Conflict with UART1
    // printf("SD_Open(): Configure GPIO group B as SDH pins.\n");
    SYS->GPB_MFPL &= (~0x77707700);
    SYS->GPB_MFPL |=   0x44404400;
    SYS->GPB_MFPH &= (~0x00000077);
    SYS->GPB_MFPH |=   0x00000044;

#elif defined SDH_GPIO_G_48PIN
    // The group 48PIN are GPB0~3, GPB5~7 for NUC505 48PIN chip
    // Conflict with both UART0 and UART1
    // printf("SD_Open(): Configure special GPIO as SDH pins for 48 pins NUC505 chip.\n");
    SYS->GPB_MFPL &= (~0x77707777);
    SYS->GPB_MFPL |=   0x44404444;

#else   // default for defined SDH_GPIO_GC
    // The group C are GPC0~2, GPC4~7
    // printf("SD_Open(): Configure GPIO group C as SDH pins.\n");
    SYS->GPC_MFPL &= (~0x77770777);
    SYS->GPC_MFPL |=   0x11110111;
#endif
}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    //SYS_UnlockReg();

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(100000000);

    /* PCLK divider */
    CLK_SetModuleClock(PCLK_MODULE, NULL, 1);

    /* Lock protected registers */
    //SYS_LockReg();

    //--- Initial SD0 multi-function pin
    SD0_Init();
}

void UART0_Init(void)
{
    /* Enable UART0 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* UART0 module clock from EXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baud-rate */
    UART_Open(UART0, 115200);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}

void I2S_Init(void)
{
    /* Enable I2S Module clock */
    CLK_EnableModuleClock(I2S_MODULE);
    /* I2S module clock from APLL */
    // APLL = 49152031Hz
    CLK_SET_APLL(CLK_APLL_49152031);
    // I2S = 49152031Hz / (0+1) = 49152031Hz for 8k, 12k, 16k, 24k, 32k, 48k, and 96k sampling rate
    CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);
    /* Reset IP */
    SYS_ResetModule(I2S_RST);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for I2S */
    // GPC[8]  = MCLK
    // GPC[9]  = DIN
    // GPC[10] = DOUT
    // GPC[11] = LRCLK
    // GPC[12] = BCLK
    SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk) ) | SYS_GPC_MFPH_PC8MFP_I2S_MCLK;
    SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk) ) | SYS_GPC_MFPH_PC9MFP_I2S_DIN;
    SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk) ) | SYS_GPC_MFPH_PC10MFP_I2S_DOUT;
    SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk) ) | SYS_GPC_MFPH_PC11MFP_I2S_LRCLK;
    SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk) ) | SYS_GPC_MFPH_PC12MFP_I2S_BCLK;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   I2S Driver Sample Code with internal CODEC           |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with internal CODEC.\n");

    /* Configure FATFS */
    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
    // Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

    /* Init I2S, IP clock and multi-function I/O */
    I2S_Init();

    WAVPlayer();

    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
