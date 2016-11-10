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
#include "numNotation.h"
#include "key.h"
#include "stdlib.h"

FATFS FatFs[_VOLUMES];      /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;       /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */
#endif


#define RXBUFSIZE   1024
#define KEY_OFFSET_PER_DEGREE       12
#define KEY_PIANO_BEGIN   21
#define AUDIO_KEYNOTE_PIANO_BEGIN 21
#define KEYNOTE_BEGIN           AUDIO_KEYNOTE_PIANO_BEGIN
#define KEYNOTE_BY_KEY(k)       (KEYNOTE_BEGIN + ((k) - KEY_PIANO_BEGIN))
#define FAKE_PLAY_OFFSET 0

extern FIL wavFileObject;
extern  uint8_t bAudioPlaying;
extern FRESULT res;
extern uint8_t u8PCMBufferTargetIdx;
extern uint32_t u32WavSamplingRate, u32WavChannel, u32WavBit;
extern FIL    wavFileObject;
extern size_t ReturnSize;

extern uint32_t aPCMBuffer[2][PCM_BUFFER_SIZE];
extern uint8_t aPCMBuffer2[12288];
extern volatile uint8_t aPCMBuffer_Full[2];

uint8_t musicNameFlag = 0;
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};
uint8_t time;

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

// Music_0 Little Star
static const NoteSimplified_t sNoteSimplified_0[] =
{
  {1,0,1,0}, {1,0,1,0}, {5,0,1,0}, {5,0,1,0},
  {6,0,1,0}, {6,0,1,0}, {5,0,0,0}, {4,0,1,0},
  {4,0,1,0}, {3,0,1,0},
  {3,0,1,0}, {2,0,1,0}, {2,0,1,0}, {1,0,0,0},
  {5,0,1,0}, {5,0,1,0}, {4,0,1,0}, {4,0,1,0},
  {3,0,1,0}, {3,0,1,0}, {2,0,0,0},
  {5,0,1,0}, {5,0,1,0}, {4,0,1,0}, {4,0,1,0},
  {3,0,1,0}, {3,0,1,0}, {2,0,0,0}, {1,0,1,0},
  {1,0,1,0}, {5,0,1,0}, {5,0,1,0},
  {6,0,1,0}, {6,0,1,0}, {5,0,0,0}, {4,0,1,0},
  {4,0,1,0}, {3,0,1,0}, {3,0,1,0}, {2,0,1,0},
  {2,0,1,0}, {1,0,0,0},
};

// Music_1 Two Tigers
static const NoteSimplified_t sNoteSimplified_1[] =
{
  {1,0,1,0}, {2,0,1,0}, {3,0,1,0}, {1,0,1,0},
  {1,0,1,0}, {2,0,1,0}, {3,0,1,0}, {1,0,1,0},
  {3,0,1,0}, {4,0,1,0}, {5,0,0,0},
  {3,0,1,0}, {4,0,1,0}, {5,0,0,0},
  {5,0,2,0}, {6,0,2,0}, {5,0,2,0}, {4,0,2,0},
  {3,0,1,0}, {1,0,1,0},
  {5,0,2,0}, {6,0,2,0}, {5,0,2,0}, {4,0,2,0},
  {3,0,1,0}, {1,0,1,0},
  {2,0,1,0}, {5,0,1,1}, {1,0,1,0},
  {2,0,1,0}, {5,0,1,1}, {1,0,1,0},
};

// Music_2 Happy Birthday
static const NoteSimplified_t sNoteSimplified_2[] =
{
  {5,0,2,1}, {5,0,2,1}, {6,0,1,1}, {5,0,1,1}, {1,0,1,0}, {7,0,0,1},
  {5,0,2,1}, {5,0,2,1}, {6,0,1,1}, {5,0,1,1}, {2,0,1,0}, {1,0,1,0},
  {5,0,2,1}, {5,0,2,1}, {5,0,1,0}, {3,0,1,0}, {1,0,1,0}, {7,0,1,1}, {6,0,1,1},
  {4,0,2,0}, {4,0,2,0}, {3,0,1,0}, {1,0,1,0}, {2,0,1,0}, {1,0,1,0},
};


static const ScoreSimplified_t sScoreSimplified[] =
{
  // Two Tigers
  { {0,0,0},    //Clef
    {0,0},      //Key Signature
    {4,4},      //Time Signature
    sizeof(sNoteSimplified_0),
    sNoteSimplified_0
  },
  // Happy Birthday
  { {0,0,0},    //Clef
    {0,0},      //Key Signature
    {3,4},      //Time Signature
    sizeof(sNoteSimplified_1),
    sNoteSimplified_1
  },
  // Yankee Doodle
  { {0,0,0},    //Clef
    {0,0},      //Key Signature
    {4,4},      //Time Signature
    sizeof(sNoteSimplified_2),
    sNoteSimplified_2
  },
};

uint8 NumNotaiton_NoteSimpToKeyNote(NoteSimplified_t note)
{
  //calculate the note from simplified note
  const uint8 offset[] = { 0, 2, 4, 5, 7, 9, 11 };
  uint8 octOffset = (note.octaves & 0x2) ? (4 - note.octaves) : (- note.octaves);

  if (note.note == 0) {
    return 0;
  }

  return KEY_PIANO_60 + offset[note.note - 1] + note.sharp + octOffset * KEY_OFFSET_PER_DEGREE;
}


#define SIZEOF_SCORESIMPLIFIED() (sizeof(sScoreSimplified) / sizeof(sScoreSimplified[0]))

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

void TMR0_IRQHandler(void)
{
    static uint32_t sec = 1;
    printf("%d sec\n", sec++);
	
		musicNameFlag=1;

    // clear timer interrupt flag
    TIMER_ClearIntFlag(TIMER0);

    /* To avoid the synchronization issue between system and APB clock domain */
    TIMER_GetIntFlag(TIMER0);

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
	
	/* Update System Core Clock */
    SystemCoreClockUpdate();
	
		CLK_EnableModuleClock(TMR0_MODULE);
	
		CLK_SetModuleClock(TMR0_MODULE, CLK_TMR0_SRC_EXT, 0);
		
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

void UART1_Init(void)
{
	/* Enable UART0 Module clock */
	CLK_EnableModuleClock(UART1_MODULE);
	/* UART0 module clock from EXT */
	CLK_SetModuleClock(UART1_MODULE, CLK_UART1_SRC_EXT, 0);
	
	SYS_ResetModule(UART1_RST);

	/* Configure UART0 and set UART1 baud rate */
	UART_Open(UART1, 115200);
	
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk) ) | SYS_GPB_MFPL_PB6MFP_UART1_TXD;
  SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk) ) | SYS_GPB_MFPL_PB7MFP_UART1_RXD;	
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
		uint32_t i,j=0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();
		UART1_Init();
	
	  NVIC_SetPriority(I2S_IRQn,1);
		NVIC_SetPriority(UART1_IRQn,1);

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
		
		// Set timer frequency to 1HZ
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 4);

    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);


    // Start Timer 0
    TIMER_Start(TIMER0);
		
		WAVPlayer("0:\\60.wav");
		
		I2S_Open(I2S, I2S_MODE_MASTER, u32WavSamplingRate, u32WavBit, u32WavChannel, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
		I2S_EnableMCLK(I2S, u32WavSamplingRate*256);		
		InternalCODEC_Setup();		
		
		while(1){
		
		for (i = 0; i < 42; i++)
		{
//			WAVPlay_Stop();
			
			switch(KEYNOTE_BY_KEY(NumNotaiton_NoteSimpToKeyNote(sScoreSimplified[0].notes[i])))
			{
				case   60:WAVPlayer("0:\\60.wav");break;
				case   61:WAVPlayer("0:\\61.wav");break;
				case   62:WAVPlayer("0:\\62.wav");break;
				case   63:WAVPlayer("0:\\63.wav");break;
				case   64:WAVPlayer("0:\\64.wav");break;
				case   65:WAVPlayer("0:\\65.wav");break;
				case   66:WAVPlayer("0:\\66.wav");break;
				case   67:WAVPlayer("0:\\67.wav");break;
				case   68:WAVPlayer("0:\\68.wav");break;
				case   69:WAVPlayer("0:\\69.wav");break;
				default:
					break;
				
			}
			while(1)
			{
				if(musicNameFlag==1)
				{
					musicNameFlag=0;
					break;
				}
				if((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1 )) 
				{   //all buffers are full, wait
						if(!bAudioPlaying) 
						{
								bAudioPlaying = 1;
								I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
								I2S_SET_RX_TH_LEVEL(I2S, I2S_FIFO_RX_LEVEL_WORD_16);
								I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &aPCMBuffer[0][0]);													// Tx Start Address
								I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &aPCMBuffer[0][PCM_BUFFER_SIZE-1]);			// Tx Threshold Address
								I2S_SET_TXDMA_EADDR(I2S, (uint32_t) &aPCMBuffer[1][PCM_BUFFER_SIZE-1]);			// Tx End Address
								I2S_ENABLE_TXDMA(I2S);
								I2S_ENABLE_TX(I2S);
								I2S_EnableInt(I2S, (I2S_IEN_TDMATIEN_Msk|I2S_IEN_TDMAEIEN_Msk));
								NVIC_EnableIRQ(I2S_IRQn);
								printf("Start Playing ...\n");
						}
							
						if(aPCMBuffer_Full[0] == 1)
								while(aPCMBuffer_Full[0]);
					}

					if ( u32WavBit == I2S_DATABIT_24 )
					{
						//12288=(PCM_BUFFER_SIZE*4)-(PCM_BUFFER_SIZE*4/4)
						res = f_read(&wavFileObject, &aPCMBuffer2[0], 12288, &ReturnSize);
						if(f_eof(&wavFileObject)) 
							break;
						// 4096=PCM_BUFFER_SIZE/4
						for ( i = 0; i < 4096; i++ )
							aPCMBuffer[u8PCMBufferTargetIdx][i] = (0 << 24) | (aPCMBuffer2[3*i+2] << 16) | (aPCMBuffer2[3*i+1] << 8) | aPCMBuffer2[3*i];
					}
					else
					{
						res = f_read(&wavFileObject, &aPCMBuffer[u8PCMBufferTargetIdx][0], PCM_BUFFER_SIZE*4, &ReturnSize);
						if(f_eof(&wavFileObject))   
							break;
					}
					
					NVIC_DisableIRQ(I2S_IRQn);
					aPCMBuffer_Full[u8PCMBufferTargetIdx] = 1;
					NVIC_EnableIRQ(I2S_IRQn);

					if(bAudioPlaying) 
					{
							if(aPCMBuffer_Full[u8PCMBufferTargetIdx^1] == 1)
									while(aPCMBuffer_Full[u8PCMBufferTargetIdx^1]);
					}

					u8PCMBufferTargetIdx ^= 1;
			}
			
		}
	}
    
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
