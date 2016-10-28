/**************************************************************************//**
 * @file     main.c
 * @version  V2.0 
 * $Revision: 9 $
 * $Date: 15/09/01 2:38p $
 * @brief    A WAV file player demo using NAU8822 audio codec used to playback WAV file stored in SD card.
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

#define NAU8822_ADDR        0x1A                /* NAU8822 Device ID */

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

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0)) {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    } else {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

uint8_t g_u8DeviceAddr;
uint8_t g_au8TxData[2];
uint8_t g_u8RxData;
uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted and prepare SLA+W */
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 1) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_SI);
        }
    } else if (u32Status == 0x10) {             /* Repeat START has been transmitted and prepare SLA+R */
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x40) {             /* SLA+R has been transmitted and ACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);
    } else if (u32Status == 0x50) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x58) {             /* DATA has been received and NACK has been returned */
        g_u8RxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
				g_u8EndFlag = 1;
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08) {                    /* START has been transmitted */
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x18) {             /* SLA+W has been transmitted and ACK has been received */
        I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI);
    } else if (u32Status == 0x20) {             /* SLA+W has been transmitted and NACK has been received */
        I2C_SET_CONTROL_REG(I2C0, I2C_STA | I2C_STO | I2C_SI);
    } else if (u32Status == 0x28) {             /* DATA has been transmitted and ACK has been received */
        if (g_u8DataLen != 2) {
            I2C_SET_DATA(I2C0, g_au8TxData[g_u8DataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_SI);
        } else {
            I2C_SET_CONTROL_REG(I2C0, I2C_STO | I2C_SI);
            g_u8EndFlag = 1;
        }
    } else {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void NAU8822_WriteData(char addr, unsigned short data)
{
		g_au8TxData[0] = ((addr << 1)  | (data >> 8));		//addr(7bit) + data(first bit)
		g_au8TxData[1] = (char)(data & 0x00FF);			//data(8bit)

		g_u8DataLen = 0;
		g_u8EndFlag = 0;

		/* I2C as master sends START signal */
		I2C_SET_CONTROL_REG(I2C0, I2C_STA);

		/* Wait I2C Tx Finish */
		while (g_u8EndFlag == 0);
		g_u8EndFlag = 0;
}

void NAU8822_ReadData(char addr)
{
		g_au8TxData[0] = (addr << 1); 					//addr(7bit) + 0 (lsb)

		g_u8DataLen = 0;
		g_u8EndFlag = 0;

		/* I2C as master sends START signal */
		I2C_SET_CONTROL_REG(I2C0, I2C_STA);

		/* Wait I2C Tx Finish */
		while (g_u8EndFlag == 0);
		g_u8EndFlag = 0;
}

void WAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    if((u32SampleRate % 8) == 0) {
        NAU8822_WriteData(36, 0x008);    //12.288Mhz
        NAU8822_WriteData(37, 0x00C);
        NAU8822_WriteData(38, 0x093);
        NAU8822_WriteData(39, 0x0E9);
    } else {
        NAU8822_WriteData(36, 0x007);    //11.2896Mhz
        NAU8822_WriteData(37, 0x021);
        NAU8822_WriteData(38, 0x161);
        NAU8822_WriteData(39, 0x026);
    }

    switch (u32SampleRate) {
		case 8000:
        NAU8822_WriteData(6, 0x1E9);
        break;
		case 11025:
		case 12000:
        NAU8822_WriteData(6, 0x1C9);
        break;
		case 22050:
		case 24000:
        NAU8822_WriteData(6, 0x189);
        break;
    case 16000:
        NAU8822_WriteData(6, 0x1A9);   /* Divide by 6, 16K */
//        NAU8822_WriteData(7, 0x006);   /* 16K for internal filter coefficients */
        break;

    case 32000:
        NAU8822_WriteData(6, 0x169);    /* Divide by 3, 32K */
//        NAU8822_WriteData(7, 0x002);    /* 32K for internal filter coefficients */
        break;

    case 44100:
    case 48000:
        NAU8822_WriteData(6, 0x149);    /* Divide by 1, 48K */
//        NAU8822_WriteData(7, 0x000);    /* 48K for internal filter coefficients */
        break;
		case 96000:
				NAU8822_WriteData(4, 0x050);
        NAU8822_WriteData(6, 0x109);
//        NAU8822_WriteData(7, 0x000);    /* 48K for internal filter coefficients */
				NAU8822_WriteData(10,0x000);
				NAU8822_WriteData(72,0x020);
				break;
		case 192000:
				NAU8822_WriteData(4, 0x050);
				NAU8822_WriteData(6, 0x109);
//        NAU8822_WriteData(7, 0x000);    /* 48K for internal filter coefficients */
				NAU8822_WriteData(72,0x027);
				break;
    }
}

void Delay(int count)
{
    volatile uint32_t i;
    for (i = 0; i < count ; i++);
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

void I2C0_Init(void)
{
		/* Enable I2C0 Module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
		/* Reset IP */
    SYS_ResetModule(I2C0_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPA14,GPA15 multi-function pins for I2C0 */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA14MFP_Msk) ) | SYS_GPA_MFPH_PA14MFP_I2C0_SCL;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA15MFP_Msk) ) | SYS_GPA_MFPH_PA15MFP_I2C0_SDA;
		
		/* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);
		
		/* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
		
		I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2S_Init(void)
{
		/* Enable I2S Module clock */
    CLK_EnableModuleClock(I2S_MODULE);
		/* I2S module clock from APLL */
		// APLL = 49152031Hz
		//CLK_SET_APLL(CLK_APLL_49152031);
		// I2S = 49152031Hz / (0+1) = 49152031Hz for 8k, 12k, 16k, 24k, 32k, 48k, and 96k sampling rate
		//CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);
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
/*  WAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void WAU8822_Setup(void)
{
    printf("\nConfigure WAU8822 ...");

		s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
		g_u8DeviceAddr = NAU8822_ADDR;

    NAU8822_WriteData(0,  0x000);   /* Reset all registers */
    Delay(0x200);

    NAU8822_WriteData(1,  0x02F);
    NAU8822_WriteData(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    NAU8822_WriteData(3,  0x07F);   /* Enable L/R main mixer, DAC */

    // offset: 0x4 => default, 24bit, I2S format, Stereo
    NAU8822_WriteData(4,  0x010);   /* 16-bit word length, I2S format, Stereo */

    NAU8822_WriteData(5,  0x000);   /* Companding control and loop back mode (all disable) */
    NAU8822_WriteData(6,  0x1AD);   /* Divide by 6, 16K */
//    NAU8822_WriteData(7,  0x006);   /* 16K for internal filter coefficients */
    NAU8822_WriteData(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    NAU8822_WriteData(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    NAU8822_WriteData(15, 0x1EF);   /* ADC left digital volume control */
    NAU8822_WriteData(16, 0x1EF);   /* ADC right digital volume control */

    NAU8822_WriteData(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    NAU8822_WriteData(47, 0x050);   /* LLIN connected, and its Gain value */
    NAU8822_WriteData(48, 0x050);   /* RLIN connected, and its Gain value */
    NAU8822_WriteData(50, 0x001);   /* Left DAC connected to LMIX */
    NAU8822_WriteData(51, 0x001);   /* Right DAC connected to RMIX */

		NAU8822_WriteData(52, 0x11F);
    NAU8822_WriteData(53, 0x11F);
		
    printf("[OK]\n");
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
    printf("|                   I2S Driver Sample Code with NAU8822 CODEC            |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with NAU8822 CODEC.\n");

    /* Configure FATFS */
    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
		// Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

    /* Init I2C0 to access WAU8822 */
    I2C0_Init();

    /* Init I2S, IP clock and multi-function I/O */
		I2S_Init();

    WAVPlayer();

    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
