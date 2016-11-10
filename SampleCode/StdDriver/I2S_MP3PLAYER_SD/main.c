/**************************************************************************//**
 * @file     main.c
 * @version  V2.1 
 * $Revision: 8 $
 * $Date: 16/01/09 3:40p $
 * @brief    A MP3 file player demo using internal audio codec used to playback MP3 file stored in SD card.
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

uint32_t volatile u32BuffPos = 0;

FATFS FatFs[_VOLUMES];               /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[16] ;                   /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[16] ;       /* Working buffer */
#endif

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

#define RXBUFSIZE   1024

uint8_t musicNameFlag = 0;
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

extern struct mad_stream   Stream;

extern FIL             mp3FileObject;
extern size_t          ReadSize;
extern size_t          Remaining;
extern size_t          ReturnSize;


void UART_TEST_HANDLE();
void UART1_IRQHandler(void);

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
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
//    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

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
	//FIXME APLL CLOCK
		// ideal clock is 49.152MHz, real clock is 49152031Hz
//	CLK_SET_APLL(CLK_APLL_49152031);	// APLL is 49152031Hz for 48000Hz
		CLK_SET_APLL(CLK_APLL_45158425);	// APLL is 45158425Hz for 44100Hz
		CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);	// 0 means (APLL/1)
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
		FRESULT res;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();
	
		UART1_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   MP3 Player Sample with Internal CODEC                |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf(" Please put MP3 files on SD card \n");

    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
		// Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

	/* Init I2S, IP clock and multi-function I/O */
		I2S_Init();
		
		UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    MP3Player("0:\\1.mp3");

    while(1)
		{
			if(musicNameFlag==1)
			{
				musicNameFlag=0;
				I2S_DISABLE_TX(I2S);
				I2S_DISABLE_TXDMA(I2S);

				I2S_Close(I2S);
				I2S_DisableMCLK(I2S);
			}
			
			if(musicNameFlag==2)
			{
				musicNameFlag=0;
				I2S_DISABLE_TX(I2S);
				I2S_DISABLE_TXDMA(I2S);

				I2S_Close(I2S);
				I2S_DisableMCLK(I2S);
			}
			
			if(Stream.buffer==NULL || Stream.error==MAD_ERROR_BUFLEN) 
				{
            if(Stream.next_frame != NULL) {
                /* Get the remaining frame */
                Remaining = Stream.bufend - Stream.next_frame;
                memmove(MadInputBuffer, Stream.next_frame, Remaining);
                ReadStart = MadInputBuffer + Remaining;
                ReadSize = FILE_IO_BUFFER_SIZE - Remaining;
            } else {
                ReadSize = FILE_IO_BUFFER_SIZE,
                ReadStart = MadInputBuffer,
                Remaining = 0;
            }

            /* read the file from SDCard */
            res = f_read(&mp3FileObject, ReadStart, ReadSize, &ReturnSize);
            if((res != FR_OK) || f_eof(&mp3FileObject)) {
                printf("Stop !(%x)\n\r", res);
                goto stop;
            }

            /* if the file is over */
            if (ReadSize > ReturnSize) {
                GuardPtr=ReadStart+ReadSize;
                memset(GuardPtr,0,MAD_BUFFER_GUARD);
                ReadSize+=MAD_BUFFER_GUARD;
            }

            Mp3FileOffset = Mp3FileOffset + ReturnSize;
            /* Pipe the new buffer content to libmad's stream decoder
                     * facility.
            */
            mad_stream_buffer(&Stream,MadInputBuffer,ReadSize+Remaining);
            Stream.error=(enum  mad_error)0;
        }

        /* decode a frame from the mp3 stream data */
        if(mad_frame_decode(&Frame,&Stream)) {
            if(MAD_RECOVERABLE(Stream.error)) {
                /*if(Stream.error!=MAD_ERROR_LOSTSYNC ||
                   Stream.this_frame!=GuardPtr)
                {
                }*/
                continue;
            } else {
                /* the current frame is not full, need to read the remaining part */
                if(Stream.error==MAD_ERROR_BUFLEN) {
                    continue;
                } else {
                    printf("Something error!!\n");

                    /* play the next file */
                    audioInfo.mp3FileEndFlag = 1;
                    goto stop;
                }
            }
        }

        /* Once decoded the frame is synthesized to PCM samples. No errors
        * are reported by mad_synth_frame();
        */
        mad_synth_frame(&Synth,&Frame);

        //
        // decode finished, try to copy pcm data to audio buffer
        //

#if 1
        if(audioInfo.mp3Playing) {
            //if next buffer is still full (playing), wait until it's empty
            if(aPCMBuffer_Full[u8PCMBufferTargetIdx] == 1)
                while(aPCMBuffer_Full[u8PCMBufferTargetIdx]);
        } else {

            if((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1 )) {       //all buffers are full, wait
                StartPlay();
								while(aPCMBuffer_Full[0]);
            }
        }
#endif

        for(i=0; i<(int)Synth.pcm.length; i++) {
            /* Get the left/right samples */
            sampleL = Synth.pcm.samples[0][i];
            sampleR = Synth.pcm.samples[1][i];
					//FIXME mono application
//						if ( audioInfo.mp3Channel )
//							sampleR = sampleL;

            /* Fill PCM data to I2S(PDMA) buffer */
            aPCMBuffer[u8PCMBufferTargetIdx][pcmbuf_idx++] = sampleR | (sampleL << 16);

            /* Need change buffer ? */
            if(pcmbuf_idx == PCM_BUFFER_SIZE) {
								NVIC_DisableIRQ(I2S_IRQn);
                aPCMBuffer_Full[u8PCMBufferTargetIdx] = 1;      //set full flag
								NVIC_EnableIRQ(I2S_IRQn);
///				f_write(&file2, &aPCMBuffer[u8PCMBufferTargetIdx][0], PCM_BUFFER_SIZE*4, &s2);
                u8PCMBufferTargetIdx ^= 1;

                pcmbuf_idx = 0;
///               printf("change to ==>%d ..\n", u8PCMBufferTargetIdx);
                /* if next buffer is still full (playing), wait until it's empty */
//                if((aPCMBuffer_Full[u8PCMBufferTargetIdx] == 1) && (audioInfo.mp3Playing))
//                    while(aPCMBuffer_Full[u8PCMBufferTargetIdx]);
            }
        }
		}
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_RDAINT_Msk) 
		{
        while(UART_IS_RX_READY(UART1)) 
				{
            u8InChar = UART_READ(UART1);
					
					  if(u8InChar == '1')
						{
							musicNameFlag = 1; 
						}
						
						if(u8InChar == '2')
						{
							musicNameFlag = 2; 
						}

            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
						{
                /* Enqueue the character */
                g_u8RecData[g_u32comRtail] = u8InChar;
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
            }
        }
        printf("\nTransmission Test:");
    }

    if(u32IntSts & UART_INTSTS_THREINT_Msk) 
		{
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp) 
				{
            u8InChar = g_u8RecData[g_u32comRhead];
            UART_WRITE(UART1, u8InChar);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
