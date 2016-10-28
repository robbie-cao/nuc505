/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 15 $
 * $Date: 15/09/01 2:44p $
 * @brief    A recorder demo, uses internal audio codec, encode in AMR and store to SD card.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "amrenc.h"

FATFS FatFs[_VOLUMES];      /* File system object for logical drive */
FIL fs;

__align(32) BYTE Buff[1024] ;       /* Working buffer */

volatile uint8_t PcmRxBuffFull[2] = {0};
__align(32) uint8_t PcmRxBuff[2][FSTLVL_BUFF_LEN] = {0};

uint32_t EncBuffHead;
uint32_t EncBuffTail;
__align(4) uint8_t EncBuff[SECLVL_BUFF_LEN];

extern void VectorRemap(void);


/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void I2S_Init(void);
void demo_LineIn(void);
void demo_MIC0(void);
void demo_MIC1(void);
void SD0_Init(void);
void amr_recorder_th(char *Filename);
void amr_recorder_bh(int Seconds);

#define AMR_FILE    "0:\\test.amr"

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

        /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

#ifdef __ICCARM__
	#pragma section = "VECTOR2"
	extern uint32_t __Vectors[];
	extern uint32_t __Vectors_Size[];
   
	printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", __section_begin("VECTOR2"));
	memcpy((void *) __section_begin("VECTOR2"), (void *) __Vectors, (unsigned int) __Vectors_Size);
	SCB->VTOR = (uint32_t) __section_begin("VECTOR2");
#endif

#ifdef __ARMCC_VERSION
	VectorRemap();
#endif

    /* Init I2S, IP clock and multi-function I/O */
    I2S_Init();
        
    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (internal CODEC)        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate %d Hz\n", 8000);
    printf("      Word width %d bits\n", 16);
    printf("      %s mode\n", "Mono");
    printf("      I2S format\n");
    printf("  The I2S I/O connection for external codec:\n");
    printf("      I2S_LRCLK (PC11)\n      I2S_BCLK(PC12)\n      I2S_MCLK(PC8)\n");
    printf("      I2S_DI (PC9)\n      I2S_DO (PC10)\n\n");
    printf("  Or need head-phone and line-in for internal CODEC.\n");
    
    I2S_Open(I2S, I2S_MODE_MASTER, 8000, I2S_DATABIT_16, I2S_MONO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);

	// Open MCLK
	I2S_EnableMCLK(I2S, 8000 * 256);
        
    I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
    I2S_SET_RX_TH_LEVEL(I2S, I2S_FIFO_RX_LEVEL_WORD_16);
	
	I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);                     // Tx Start Address
	I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][FSTLVL_BUFF_LEN-4]);  // Tx Threshold Address
	I2S_SET_TXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][FSTLVL_BUFF_LEN-4]);  // Tx End Address

	I2S_SET_RXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);                     // Rx Start Address
	I2S_SET_RXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][FSTLVL_BUFF_LEN-4]);  // Rx Threshold Address
	I2S_SET_RXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][FSTLVL_BUFF_LEN-4]);  // Rx End Address

	/* Init AMR codec */
	amrInitEncode();
	/* Create AMR file */
	amr_recorder_th(AMR_FILE);
	
	/* Init one of three input source */
	demo_LineIn();
//	demo_MIC0();
//	demo_MIC1();

	/* record 20 seconds then close file */
    amr_recorder_bh(20);
	/* Clean AMR codec */
	amrFinishEncode();
    while(1);
}

void amr_recorder_th(char *Filename)
{
    FRESULT res;
    UINT s2;
   
    /* Configure FATFS */
    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
	// Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

    res = f_open(&fs, Filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Open file error \r\n");
        return;
    }
    
	/* write AMR header to file */
    f_write(&fs, &AMR_ENC_AMR_FILE_ID, sizeof(AMR_ENC_AMR_FILE_ID) - 1, &s2);
    f_sync(&fs);
}

void amr_recorder_bh(int Seconds)
{
    int play_started = 0;
	uint32_t WaitingPcmBuf = 0;
	int encsize;
	int bufsize;
	uint32_t realwrite;
	uint32_t record_len;

	/* Clear EncBuff pointers */
	EncBuffHead = 0;
	EncBuffTail = 0;

    /* Start to record. */
    I2S_ENABLE_RXDMA(I2S);
    I2S_ENABLE_RX(I2S);
    
	/* Enable related interrupts */
    NVIC_EnableIRQ(I2S_IRQn);
    I2S_EnableInt(I2S, (I2S_IEN_RDMATIEN_Msk|I2S_IEN_RDMAEIEN_Msk));

	record_len = 0;

	while(1)
	{
		if (PcmRxBuffFull[WaitingPcmBuf] == TRUE)
		{
			if (!play_started)
			{
				/* Start playback after the first buffer has been ready. */
				I2S_ENABLE_TXDMA(I2S);
				I2S_ENABLE_TX(I2S);
				play_started = 1;
			}

			/* Encode a AMR frame */
			amrEncode(7, 0, (short *)(PcmRxBuff[WaitingPcmBuf]), (short *)(EncBuff+EncBuffHead), &encsize);
			PcmRxBuffFull[WaitingPcmBuf] = FALSE;
			
			record_len += encsize;
			
			if ((EncBuffHead += encsize) >= sizeof(EncBuff))
				EncBuffHead = 0;
				
			WaitingPcmBuf = 1-WaitingPcmBuf;			
		}
		
		/* Calculate encoded size */
		bufsize = EncBuffHead - EncBuffTail;
		if (bufsize < 0)
			bufsize += sizeof(EncBuff);

		/* Write encoded data to SD card if reach write threshold */
		if (bufsize >= MAX_WRITE_STRIDE)
		{
			f_write(&fs, EncBuff+EncBuffTail, MAX_WRITE_STRIDE, &realwrite);
			printf("w=%d ", realwrite);
			
			if ((EncBuffTail += MAX_WRITE_STRIDE) >= sizeof(EncBuff))
				EncBuffTail = 0;
		}
		
		/* check record time */
		if (record_len >= Seconds * 50 * AMR_ENC_MAX_PACKET_SIZE)
			/* break out if time is up */
			break;
	}

	/* flush remain data to SD card */
	bufsize = EncBuffHead - EncBuffTail;
	if (bufsize < 0)
	{
		f_write(&fs, EncBuff+EncBuffTail, sizeof(EncBuff)-EncBuffTail, &realwrite);
		EncBuffTail = 0;
		bufsize = EncBuffHead /* - EncBuffTail */;
	}
	if (bufsize)
	{
		f_write(&fs, EncBuff+EncBuffTail, bufsize, &realwrite);
	}
		
    f_close(&fs);
        
    printf( "\n### AMR recorded ###\n" );
}

void SysTick_Handler(void)
{
		
}

void demo_LineIn()
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
//	I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

	/* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */

    printf("[OK]\n");
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
//	I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

   /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
	 
	 printf("[OK]\n");
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
//	I2S_SET_INTERNAL_CODEC(I2S, 0x12, 0x00);	//Un-Mute the ADC Side tone volume

  /* If Fs is changed, please Mute Headphone First and soft reset digital part after MCLK is stable. */
	
	printf("[OK]\n");
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
	//CLK_SysTickDelay(500);    //delay 5 seconds
	// I2S = 49152031Hz / (0+1) = 49152031Hz for 8k, 12k, 16k, 24k, 32k, 48k, and 96k sampling rate
	CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 1);
    
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

//void SysTick_Handler(void){
        
//}

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

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
