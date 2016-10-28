/**************************************************************************//**
 * @file     wavplayer.c
 * @version  V1.02
 * $Revision: 4 $
 * $Date: 14/11/10 1:52p $
 * @brief    NUC505 I2S Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"
#include "amrdec.h"

#include "diskio.h"
#include "ff.h"

FIL    fFileObj;
size_t ReturnSize;

#define PCM_BUFFER_SIZE AMR_FRAME_SIZE

__align(64) int16_t aPCMBuffer[PCM_BUFFER_SIZE*2*2];
volatile uint8_t aPCMBuffer_Full[2]= {0,0};
int16_t DecodeBuffer[AMR_FRAME_SIZE];

uint8_t bAudioPlaying = 0;


// Enable I2S TX with PDMA function
void StartPlay(void)
{
	/* I2S initialization */
	I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
	I2S_SET_TXDMA_STADDR(I2S, (uint32_t) aPCMBuffer);							// Tx Start Address
	I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &aPCMBuffer[PCM_BUFFER_SIZE*2-2]);		// Tx Threshold Address
	I2S_SET_TXDMA_EADDR(I2S, (uint32_t) &aPCMBuffer[PCM_BUFFER_SIZE*2*2-2]);	// Tx End Address

	I2S_ENABLE_TXDMA(I2S);
    I2S_ENABLE_TX(I2S);
	I2S_EnableInt(I2S, (I2S_IEN_TDMATIEN_Msk|I2S_IEN_TDMAEIEN_Msk));
	NVIC_EnableIRQ(I2S_IRQn);
}

// Disable I2S TX with PDMA function
void StopPlay(void)
{
    I2S_DISABLE_TXDMA(I2S);
    I2S_DISABLE_TX(I2S);
}

void AMRPlayer(char *pFilename)
{
    FRESULT res;
    uint8_t u8PCMBufferTargetIdx;

	char amr_buf[AMR_MAX_PACKET_SIZE];
	int amr_mode;
	int packet_size;
	uint8_t u8StreamPending;

	u8StreamPending = FALSE;
	u8PCMBufferTargetIdx = 0;
	aPCMBuffer_Full[0] = 0;
	aPCMBuffer_Full[1] = 0;

    res = f_open(&fFileObj, pFilename, FA_OPEN_EXISTING | FA_READ);
    if (res != FR_OK)
	{
        printf("ERROR: Open file error!\n");
        return;
    }

	f_read(&fFileObj, amr_buf, 6, &ReturnSize);
	if (strncmp(amr_buf, "#!AMR\n",6))
	{
		printf("ERROR: AMR file ID miss!\n");
		return;
	}

	f_read(&fFileObj, &amr_mode, 1, &ReturnSize);
	f_lseek(&fFileObj, 6);	// return to the packet start
	amr_mode = AMR_MODE(amr_mode);
	packet_size = amrPacketSize(amr_mode);

	/* When sampling rate is 8KHz, it has adjust clock divisor to program internal codec correctly */
	CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 1);
	
	I2S_Open(I2S, I2S_MODE_MASTER, 8000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
	I2S_EnableMCLK(I2S, 8000*256);

	InternalCODEC_Setup();

	/* amr decoder initialzation */
	amrInitDecode();

    while(1)
	{
        if((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1))
		{
            if(!bAudioPlaying)
			{
				// start playing when dual buffers are full
                bAudioPlaying = 1;
				StartPlay();

                printf("Start Playing ...\n");
            }
        }

		if (u8StreamPending == FALSE)
		{
			if (!f_eof(&fFileObj))
			{
				f_read(&fFileObj, amr_buf, packet_size, &ReturnSize);

				if (ReturnSize != packet_size)
				{
					// shoule not happen. clean the packet.
					printf("ERROR: AMR packet error. Ignore it.\n");
					memset(amr_buf, 0, packet_size);
				}

				amrDecode(amr_buf, (short *)DecodeBuffer);

				u8StreamPending = TRUE;
			}
			else
				break;
		}
		else
		{
			int i;
			int16_t *p;

			while(aPCMBuffer_Full[u8PCMBufferTargetIdx]);

			/* Dupicate audio data to two channels */
			p = (u8PCMBufferTargetIdx) ? (aPCMBuffer+PCM_BUFFER_SIZE*2) : (aPCMBuffer);
			for(i=0; i<AMR_FRAME_SIZE; i++)
			{
				*p++ = DecodeBuffer[i];
				*p++ = DecodeBuffer[i];
			}

			NVIC_DisableIRQ(I2S_IRQn);
			aPCMBuffer_Full[u8PCMBufferTargetIdx] = 1;
			NVIC_EnableIRQ(I2S_IRQn);

			u8PCMBufferTargetIdx ^= 1;

			u8StreamPending = FALSE;
		}
    }

	/* Releas memory used by AMR decoder */
	amrFinishDecode();

	/* Wait until dual buffer are empty. */
	while((aPCMBuffer_Full[0] == 0) && (aPCMBuffer_Full[1] == 0)) {};
    StopPlay();

	f_close(&fFileObj);
	printf("Stop ...\n");
}

