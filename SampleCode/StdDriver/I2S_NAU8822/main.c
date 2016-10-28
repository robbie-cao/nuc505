/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * $Revision: 7 $
 * $Date: 14/07/31 2:50p $
 * @brief    An I2S demo using NAU8822 audio codec to playback the input from line-in or MIC interface.
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
void I2C0_Init(void);
void I2S_Init(void);
void demo_stereo_MIC(void);
void demo_stereo_LineIn(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

		/* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

		/* Init I2C0, IP clock and multi-function I/O */
		I2C0_Init();
		
		/* Init I2S, IP clock and multi-function I/O */
		I2S_Init();
		
    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (external CODEC)        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 48 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC11)\n      I2S_BCLK(PC12)\n      I2S_MCLK(PC8)\n");
    printf("      I2S_DI (PC9)\n      I2S_DO (PC10)\n\n");

		/* Master mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
		/* Other sampling rate please change APLL clock setting in I2S_Init() */
		I2S_Open(I2S, I2S_MODE_MASTER, 48000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_DISABLE_INTERNAL_CODEC);

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
		
		demo_stereo_MIC();
//		demo_stereo_LineIn();

		// Clear Interrupt Status
		I2S_CLR_INT_FLAG(I2S, I2S_STATUS_LZCIF_Msk|I2S_STATUS_RZCIF_Msk|I2S_STATUS_TXOVIF_Msk|I2S_STATUS_TXUDIF_Msk|I2S_STATUS_RXOVIF_Msk|I2S_STATUS_RXUDIF_Msk|I2S_STATUS_TDMATIF_Msk|I2S_STATUS_TDMAEIF_Msk|I2S_STATUS_RDMATIF_Msk|I2S_STATUS_RDMAEIF_Msk);
		// Rx Enable
		I2S_ENABLE_RX(I2S);

		NVIC_EnableIRQ(I2S_IRQn);
		I2S_EnableInt(I2S, (I2S_IEN_RDMATIEN_Msk|I2S_IEN_RDMAEIEN_Msk));

		while(1);
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

	/* To avoid the synchronization issue between system and APB clock domain */
    u32Status = I2C_GET_STATUS(I2C0);
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

void I2C_Delay(int count)
{
	volatile unsigned int i;
	for (i = 0; i < count ; i++);
}

void demo_stereo_MIC(void)
{
		printf("  NOTE: Need head-phone for external CODEC.\n");
		printf("  NOTE: Need 2 mic for external CODEC.\n");

		/* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
		g_u8DeviceAddr = 0x1A;

		NAU8822_WriteData(0, 0x000);   /* Reset all registers */
		I2C_Delay(0x1000);
		
		//input source is MIC
		NAU8822_WriteData(1, 0x01F);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_WriteData(4, 0x070);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_WriteData(5, 0x000);//R5 companding ctrl
		NAU8822_WriteData(6, 0x000);//R6 clock ctrl at slave mode
		NAU8822_WriteData(35, 0x000);//R35 disable noise gate
		NAU8822_WriteData(45, 0x13F);//R45 Left input PGA gain
		NAU8822_WriteData(46, 0x13F);//R46 Right input PGA gain
		NAU8822_WriteData(44, 0x033);//R44 MIC
		NAU8822_WriteData(47, 0x000);//R47 Left ADC boost
		NAU8822_WriteData(48, 0x000);//R48 Right ADC boost
		NAU8822_WriteData(2, 0x1BF);//R2 Power Management 2
		NAU8822_WriteData(3, 0x00F);//R3 Power Management 3
		NAU8822_WriteData(10, 0x000);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_WriteData(45, 0x13F);//R45 Left input PGA gain
		NAU8822_WriteData(46, 0x13F);//R46 Right input PGA gain
		NAU8822_WriteData(50, 0x001);//R50 Left mixer
		NAU8822_WriteData(51, 0x001);//R51 Right mixer
		NAU8822_WriteData(49, 0x002);//R49 Output control

		printf("I2C write NAU8822 OK\n");
		
		s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
		
		NAU8822_ReadData(1);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_ReadData(4);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_ReadData(5);//R5 companding ctrl
		NAU8822_ReadData(6);//R6 clock ctrl at slave mode
		NAU8822_ReadData(35);//R35 disable noise gate
		NAU8822_ReadData(45);//R45 Left input PGA gain
		NAU8822_ReadData(46);//R46 Right input PGA gain
		NAU8822_ReadData(44);//R44 MIC
		NAU8822_ReadData(47);//R47 Left ADC boost
		NAU8822_ReadData(48);//R48 Right ADC boost
		NAU8822_ReadData(2);//R2 Power Management 2
		NAU8822_ReadData(3);//R3 Power Management 3
		NAU8822_ReadData(10);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_ReadData(45);//R45 Left input PGA gain
		NAU8822_ReadData(46);//R46 Right input PGA gain
		NAU8822_ReadData(50);//R50 Left mixer
		NAU8822_ReadData(51);//R51 Right mixer
		NAU8822_ReadData(49);//R49 Output control
		
		printf("I2C read NAU8822 OK at g_u8RxData\n");
}

void demo_stereo_LineIn(void)
{
		printf("  NOTE: Need head-phone for external CODEC.\n");
		printf("  NOTE: Need Line-In for external CODEC.\n");

		/* I2C function to write data to slave */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;
		g_u8DeviceAddr = 0x1A;
		
		NAU8822_WriteData(0, 0x000);   /* Reset all registers */
		I2C_Delay(0x1000);
		
		//input source is Line-In
		NAU8822_WriteData(1, 0x01F);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_WriteData(4, 0x070);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_WriteData(5, 0x000);//R5 companding ctrl
		NAU8822_WriteData(6, 0x000);//R6 clock ctrl at slave mode
		NAU8822_WriteData(35, 0x000);//R35 disable noise gate
		NAU8822_WriteData(44, 0x044);//R44 Line-In
		NAU8822_WriteData(47, 0x050);//R47 Left ADC boost
		NAU8822_WriteData(48, 0x050);//R48 Right ADC boost
		NAU8822_WriteData(2, 0x1BF);//R2 Power Management 2
		NAU8822_WriteData(3, 0x00F);//R3 Power Management 3
		NAU8822_WriteData(10, 0x000);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_WriteData(50, 0x001);//R50 Left mixer
		NAU8822_WriteData(51, 0x001);//R51 Right mixer
		NAU8822_WriteData(49, 0x002);//R49 Output control

		printf("I2C write NAU8822 OK\n");
		
		s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;
		
		NAU8822_ReadData(1);//R1 MICBIASEN ABIASEN IOBUFEN REFIMP
		NAU8822_ReadData(4);//R4 select audio format(I2S format) and word length (32bits)
		NAU8822_ReadData(5);//R5 companding ctrl
		NAU8822_ReadData(6);//R6 clock ctrl at slave mode
		NAU8822_ReadData(35);//R35 disable noise gate
		NAU8822_ReadData(44);//R44 Line-In
		NAU8822_ReadData(47);//R47 Left ADC boost
		NAU8822_ReadData(48);//R48 Right ADC boost
		NAU8822_ReadData(2);//R2 Power Management 2
		NAU8822_ReadData(3);//R3 Power Management 3
		NAU8822_ReadData(10);//R10 DAC control (softmute disable, oversample select 64x (lowest power) )
		NAU8822_ReadData(50);//R50 Left mixer
		NAU8822_ReadData(51);//R51 Right mixer
		NAU8822_ReadData(49);//R49 Output control
		
		printf("I2C read NAU8822 OK at g_u8RxData\n");
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
