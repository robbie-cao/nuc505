/**************************************************************************//**
 * @file     main.c
 * @version  V1.2
 * $Revision: 9 $
 * $Date: 14/10/02 3:46p $
 * @brief    An I2S demo using internal audio codec used to playback the input from line-in or MIC interface.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"

int16_t PcmRxBuff[2][BUFF_LEN*2] = {0};
int16_t PcmTxBuff[2][BUFF_LEN*2] = {0};

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void I2S_Init(void);
void demo_LineIn(void);
void demo_MIC0(void);
void demo_MIC1(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

		/* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

		/* Init I2S, IP clock and multi-function I/O */
		I2S_Init();
		
    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (internal CODEC)        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 48 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC11)\n      I2S_BCLK(PC12)\n      I2S_MCLK(PC8)\n");
    printf("      I2S_DI (PC9)\n      I2S_DO (PC10)\n\n");
    printf("  NOTE: Need head-phone and line-in for internal CODEC.\n");

		/* Master mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
		/* Other sampling rate please change APLL clock setting in I2S_Init() */
		I2S_Open(I2S, I2S_MODE_MASTER, 48000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);

		// Open MCLK
		I2S_EnableMCLK(I2S, 48000*256);
		
		I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
		I2S_SET_RX_TH_LEVEL(I2S, I2S_FIFO_RX_LEVEL_WORD_16);
		
		I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &PcmTxBuff[0]);								// Tx Start Address
		I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &PcmTxBuff[0][BUFF_LEN*2-2]);	// Tx Threshold Address
		I2S_SET_TXDMA_EADDR( I2S, (uint32_t) &PcmTxBuff[1][BUFF_LEN*2-2]);	// Tx End Address

		I2S_SET_RXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);								// Rx Start Address
		I2S_SET_RXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][BUFF_LEN*2-2]);	// Rx Threshold Address
		I2S_SET_RXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][BUFF_LEN*2-2]);	// Rx End Address

		// Open Rx Dma Enable
		I2S_ENABLE_RXDMA(I2S);
		
		demo_LineIn();
//		demo_MIC0();
//		demo_MIC1();

		// Clear Interrupt Status
		I2S_CLR_INT_FLAG(I2S, I2S_STATUS_LZCIF_Msk|I2S_STATUS_RZCIF_Msk|I2S_STATUS_TXOVIF_Msk|I2S_STATUS_TXUDIF_Msk|I2S_STATUS_RXOVIF_Msk|I2S_STATUS_RXUDIF_Msk|I2S_STATUS_TDMATIF_Msk|I2S_STATUS_TDMAEIF_Msk|I2S_STATUS_RDMATIF_Msk|I2S_STATUS_RDMAEIF_Msk);
		// Rx Enable
		I2S_ENABLE_RX(I2S);
		
		NVIC_EnableIRQ(I2S_IRQn);
		I2S_EnableInt(I2S, (I2S_IEN_RDMATIEN_Msk|I2S_IEN_RDMAEIEN_Msk));

		while(1);
}

void SysTick_Handler(void)
{
		
}

void demo_LineIn(void)
{
	uint32_t i;

		// Setting Right Line In Channel
		SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD4MFP_Msk) ) | SYS_GPD_MFPL_PD4MFP_RLINEIN;
		SYS_SetSharedPinType(SYS_PORT_D, 4, 0, 0);
	
		/* IIC Configure Step without PLL: */
		/* Add MCLK(256*Fs) in. */

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	//Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	//Mute the ADC Right channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x0F);	//Mute the ADC Side tone volume

		I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	//Set CODEC slave

		I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xF0);	//Enable Analog Part
		I2S_SET_INTERNAL_CODEC(I2S, 0x0E, 0x00);	//ADC input select Line in

		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
		for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
			CLK_SysTickDelay(100000);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
		I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
		CLK_SysTickDelay(100000);	//Delay 100mS
		
		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x06);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x06);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x08);	//Un-Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x08);	//Un-Mute the ADC Right channel volume
//		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

    /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}

void demo_MIC0(void)
{
	uint32_t i;

		/* IIC Configure Step without PLL: */
		/* Add MCLK(256*Fs) in. */

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	//Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	//Mute the ADC Right channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x0F);	//Mute the ADC Side tone volume

		I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	//Set CODEC slave

		I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xC0);	//Enable Analog Part
		I2S_SET_INTERNAL_CODEC(I2S, 0x0E, 0x02);	//ADC input select MIC0

		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
		for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
			CLK_SysTickDelay(100000);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
		I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
		CLK_SysTickDelay(100000);	//Delay 100mS
		
		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x06);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x06);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x18);	//Un-Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x08);	//Un-Mute the ADC Right channel volume
//		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

   /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
}

void demo_MIC1(void)
{
	uint32_t i;

		SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD3MFP_Msk) ) | SYS_GPD_MFPL_PD3MFP_MIC1_N;
		SYS->GPD_MFPL  = (SYS->GPD_MFPL & (~SYS_GPD_MFPL_PD2MFP_Msk) ) | SYS_GPD_MFPL_PD2MFP_MIC1_P;
		SYS_SetSharedPinType(SYS_PORT_D, 2, 0, 0);
		SYS_SetSharedPinType(SYS_PORT_D, 3, 0, 0);

		/* IIC Configure Step without PLL: */
		/* Add MCLK(256*Fs) in. */

		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);	// Mute headphone of Left channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);	// Mute headphone of Right channel
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	//Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	//Mute the ADC Right channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x0F);	//Mute the ADC Side tone volume

		I2S_SET_INTERNAL_CODEC(I2S, 0x02, 0xC0);	//Set CODEC slave

		I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x80);	//Digital Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0F, 0xC0);	//Enable Analog Part
		I2S_SET_INTERNAL_CODEC(I2S, 0x0E, 0x06);	//ADC input select MIC1

		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF3);	//Analog Part Enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0D, 0x31);	//Biasing enable
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xE3);
		for (i=0; i < 15; i++)	//Delay 1.5s~2.5s
			CLK_SysTickDelay(100000);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0A, 0x09);
		I2S_SET_INTERNAL_CODEC(I2S, 0x0B, 0xF0);
		I2S_SET_INTERNAL_CODEC(I2S, 0x00, 0xD0);	//ADC digital enabled
		CLK_SysTickDelay(100000);	//Delay 100mS
		
		I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x06);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x06);	//Un-mute Headphone and set volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x18);	//Un-Mute the ADC Left channel volume
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x08);	//Un-Mute the ADC Right channel volume
//		I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

  /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
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

    CLK_SetCoreClock(FREQ_96MHZ);
    
		/* PCLK divider */
		CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
		
    /* Lock protected registers */
    //SYS_LockReg();
        
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
		// I2S = 49152031Hz / (0+1) for 8k, 12k, 16k, 24k, 32k, 48k, and 96k sampling rate

		// APLL = 45158425Hz
		// CLK_SET_APLL(CLK_APLL_45158425);
		// I2S = 45158425Hz / (0+1) for 11025, 22050, and 44100 sampling rate

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

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
