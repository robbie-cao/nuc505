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

#include "diskio.h"
#include "ff.h"

FIL    wavFileObject;
size_t ReturnSize;

uint32_t aPCMBuffer[2][PCM_BUFFER_SIZE];
#if 0
//12288=(PCM_BUFFER_SIZE*4)-(PCM_BUFFER_SIZE*4/4)
uint8_t aPCMBuffer2[12288];
#endif
volatile uint8_t aPCMBuffer_Full[2]= {0,0};
uint32_t aWavHeader[11];

int16_t PcmRxBuff[2][BUFF_LEN*2] = {0};
int16_t PcmTxBuff[2][BUFF_LEN*2] = {0};

uint8_t bAudioPlaying = 0;

void WAVPlayer(void)
{
    FRESULT res;
    uint8_t u8PCMBufferTargetIdx = 0;
    uint32_t u32WavSamplingRate, u32WavChannel, u32WavBit;
    uint32_t i;

    res = f_open(&wavFileObject, "0:\\test.wav", FA_OPEN_EXISTING | FA_READ);
    if (res != FR_OK) {
        printf("Open file error!\n");
        return;
    }

    /* Scan wave header and finally position file pointer to start of wave data region. */
    do {
        uint32_t ckID = (uint32_t) -1, ckSize = (uint32_t) -1, wavSig = (uint32_t) -1;
        uint32_t wavHdrPos = (uint32_t) -1;
        uint32_t wavDatPos = (uint32_t) -1;

        u32WavSamplingRate = 0;
        u32WavChannel = 0;
        u32WavBit = 0;

        f_read(&wavFileObject, &ckID, 4, &ReturnSize);      // "RIFF" Chunk ID.
        f_read(&wavFileObject, &ckSize, 4, &ReturnSize);    // Chunk size.
        f_read(&wavFileObject, &wavSig, 4, &ReturnSize);    // "WAVE" signature.
        if (ckID != 0x46464952 ||   // "RIFF"
                wavSig != 0x45564157) {  // "WAVE"
            printf("Incorrect wave format!\n");
            return;
        }
        while (1) {
            if(f_eof(&wavFileObject)) {
                printf("End of file reached!\n");
                return;
            }

            ckID = (uint32_t) -1;
            ckSize = (uint32_t) -1;
            f_read(&wavFileObject, &ckID, 4, &ReturnSize);      // Chunk ID.
            f_read(&wavFileObject, &ckSize, 4, &ReturnSize);    // Chunk size.
            if (ckID == 0x20746d66) {   // "fmt "
                uint16_t fmtTag, numChan;
                uint32_t sampleRate, byteRate;
                uint16_t blockAlign, sampleSize;

                wavHdrPos = f_tell(&wavFileObject);

                f_read(&wavFileObject, &fmtTag, 2, &ReturnSize);        // Format tag.
                f_read(&wavFileObject, &numChan, 2, &ReturnSize);       // Number of channels.
                f_read(&wavFileObject, &sampleRate, 4, &ReturnSize);    // Sample rate.
                f_read(&wavFileObject, &byteRate, 4, &ReturnSize);      // Byte rate.
                f_read(&wavFileObject, &blockAlign, 2, &ReturnSize);    // Block align.
                f_read(&wavFileObject, &sampleSize, 2, &ReturnSize);    // Bits per sample.

                printf("Compression code: %d\n", fmtTag);

                u32WavSamplingRate = sampleRate;
                u32WavChannel = numChan;
                u32WavBit = sampleSize;

                f_lseek(&wavFileObject, wavHdrPos + ckSize);    // Seek to next chunk.
            }
            else if (ckID == 0x61746164) {  // "data"
                wavDatPos = f_tell(&wavFileObject);

                f_lseek(&wavFileObject, wavDatPos + ckSize);    // Seek to next chunk.
            }
            else {
                f_lseek(&wavFileObject, f_tell(&wavFileObject) + ckSize);    // Seek to next chunk.
            }

            if (wavHdrPos != (uint32_t) -1 && wavDatPos != (uint32_t) -1) {
                f_lseek(&wavFileObject, wavDatPos);    // Seek to wave data.
                break;
            }
        }
    }
    while (0);

    // change sampling rate
    if ( u32WavSamplingRate % 8 )
    {
        // APLL = 45158425Hz
        CLK_SET_APLL(CLK_APLL_45158425);
        // I2S = 45158425Hz / (0+1) = 45158425Hz for 11025k, 22050k, and 44100k sampling rate
    }

    /* Check if sampling rate is supported. */
    do {
        uint32_t SRSupArr[] = { // Sampling rate support list
            8000, 16000, 32000,
            12000, 24000, 48000, 96000,
            11025, 22050, 44100,
        };
        uint32_t *SRSupInd;
        uint32_t *SRSupEnd = SRSupArr + sizeof (SRSupArr) / sizeof (SRSupArr[0]);

        for (SRSupInd = SRSupArr; SRSupInd != SRSupEnd; SRSupInd ++) {
            if (u32WavSamplingRate == *SRSupInd) {
                break;
            }
        }
        if (SRSupInd == SRSupEnd) {
            printf("%d sampling rate not support!\n", u32WavSamplingRate);
            return;
        }
    }
    while (0);

    printf("wav: sampling rate=%d, channel(s)=%d, bits=%d\n", u32WavSamplingRate, u32WavChannel, u32WavBit);

    if (u32WavBit == 16)
        u32WavBit = I2S_DATABIT_16;
    else if (u32WavBit == 24)
        u32WavBit = I2S_DATABIT_24;
    else if (u32WavBit == 32)
        u32WavBit = I2S_DATABIT_32;
    else
    {
        printf("bits not support!\n");
        return;
    }

    if (u32WavChannel == 2)
        u32WavChannel = I2S_STEREO;
    else if (u32WavChannel == 1)
        u32WavChannel = I2S_MONO;
    else
    {
        printf("channel(s) not support!\n");
        return;
    }

    I2S_Open(I2S, I2S_MODE_MASTER, u32WavSamplingRate, u32WavBit, u32WavChannel, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
    I2S_EnableMCLK(I2S, u32WavSamplingRate*256);
    InternalCODEC_Setup();

    while(1) {
        if((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1 )) {       //all buffers are full, wait
            if(!bAudioPlaying) {
                bAudioPlaying = 1;
                I2S_SET_TX_TH_LEVEL(I2S, I2S_FIFO_TX_LEVEL_WORD_15);
                I2S_SET_RX_TH_LEVEL(I2S, I2S_FIFO_RX_LEVEL_WORD_16);
                I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &aPCMBuffer[0][0]);                 // Tx Start Address
                I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &aPCMBuffer[0][PCM_BUFFER_SIZE-1]); // Tx Threshold Address
                I2S_SET_TXDMA_EADDR(I2S, (uint32_t) &aPCMBuffer[1][PCM_BUFFER_SIZE-1]);  // Tx End Address
                I2S_ENABLE_TXDMA(I2S);
                I2S_ENABLE_TX(I2S);
                I2S_EnableInt(I2S, (I2S_IEN_TDMATIEN_Msk|I2S_IEN_TDMAEIEN_Msk));
                I2S_SET_RXDMA_STADDR(I2S, (uint32_t) &PcmRxBuff[0]);								// Rx Start Address
                I2S_SET_RXDMA_THADDR(I2S, (uint32_t) &PcmRxBuff[0][BUFF_LEN*2-2]);	// Rx Threshold Address
                I2S_SET_RXDMA_EADDR( I2S, (uint32_t) &PcmRxBuff[1][BUFF_LEN*2-2]);	// Rx End Address
                I2S_ENABLE_RXDMA(I2S);
                I2S_ENABLE_RX(I2S);
                I2S_EnableInt(I2S, (I2S_IEN_RDMATIEN_Msk|I2S_IEN_RDMAEIEN_Msk));
                NVIC_EnableIRQ(I2S_IRQn);
                printf("Start Playing ...\n");
            }

            // while((aPCMBuffer_Full[0] == 1) && (aPCMBuffer_Full[1] == 1));
            if(aPCMBuffer_Full[0] == 1)
                while(aPCMBuffer_Full[0]);
            // printf(".");
        }

#if 0
        if ( u32WavBit == I2S_DATABIT_24 )
        {
            //12288=(PCM_BUFFER_SIZE*4)-(PCM_BUFFER_SIZE*4/4)
            res = f_read(&wavFileObject, &aPCMBuffer2[0], 12288, &ReturnSize);
            if(f_eof(&wavFileObject))   break;
            // 4096=PCM_BUFFER_SIZE/4
            for ( i = 0; i < 4096; i++ )
                aPCMBuffer[u8PCMBufferTargetIdx][i] = (0 << 24) | (aPCMBuffer2[3*i+2] << 16) | (aPCMBuffer2[3*i+1] << 8) | aPCMBuffer2[3*i];
        }
        else
#endif
        {
            res = f_read(&wavFileObject, &aPCMBuffer[u8PCMBufferTargetIdx][0], PCM_BUFFER_SIZE*4, &ReturnSize);
            for ( i = 0; i < PCM_BUFFER_SIZE; i++ ) {
                aPCMBuffer[u8PCMBufferTargetIdx][i] += PcmTxBuff[u8PCMBufferTargetIdx][i];
            }
            if(f_eof(&wavFileObject))   break;
        }

        NVIC_DisableIRQ(I2S_IRQn);
        aPCMBuffer_Full[u8PCMBufferTargetIdx] = 1;
        NVIC_EnableIRQ(I2S_IRQn);

        if(bAudioPlaying) {
            if(aPCMBuffer_Full[u8PCMBufferTargetIdx^1] == 1)
                while(aPCMBuffer_Full[u8PCMBufferTargetIdx^1]);
        }

        u8PCMBufferTargetIdx ^= 1;

        // printf("change to ==>%d\n", u8PCMBufferTargetIdx);
    }

    I2S_DISABLE_TX(I2S);
    I2S_DISABLE_TXDMA(I2S);
    f_close(&wavFileObject);
}

