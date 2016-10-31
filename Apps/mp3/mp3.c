/*
 * libmad - MPEG audio decoder library
 * Copyright (C) 2000-2004 Underbit Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: minimad.c,v 1.4 2004/01/23 09:41:32 rob Exp $
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NUC505Series.h"

#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "mad.h"

#define MP3_FILE    "0:\\mp3\\36.mp3"
///#define MP3_FILE2    "0:\\48k2.mp3"

/*
 * This is perhaps the simplest example use of the MAD high-level API.
 * Standard input is mapped into memory via mmap(), then the high-level API
 * is invoked with three callbacks: input, output, and error. The output
 * callback converts MAD's high-resolution PCM samples to 16 bits, then
 * writes them to standard output in little-endian, stereo-interleaved
 * format.
 */


struct mad_stream   Stream;
struct mad_frame    Frame;
struct mad_synth    Synth;

FIL             mp3FileObject;
size_t          ReadSize;
size_t          Remaining;
size_t          ReturnSize;

#ifdef __ICCARM__
#pragma data_alignment=32
// I2S PCM buffer x2
signed int aPCMBuffer[2][PCM_BUFFER_SIZE];
#endif

#ifdef __ARMCC_VERSION
// I2S PCM buffer x2
__align(32) signed int aPCMBuffer[2][PCM_BUFFER_SIZE];
#endif

// File IO buffer for MP3 library
unsigned char MadInputBuffer[FILE_IO_BUFFER_SIZE+MAD_BUFFER_GUARD];
// buffer full flag x2
volatile uint8_t aPCMBuffer_Full[2]= {0,0};
// audio information structure
struct AudioInfoObject audioInfo;

// Parse MP3 header and get some informations
int32_t MP3_ParseHeaderInfo(uint8_t *pFileName)
{
    FRESULT res;
    uint32_t i=0;
    int32_t i32Offset;

    res = f_open(&mp3FileObject, (void *)pFileName, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
        printf("file is opened!!\r\n");
        audioInfo.playFileSize = mp3FileObject.fsize;

        /* FIXME we just to seek mp3 data (1st frame) */
        res = f_read(&mp3FileObject, (char *)(&MadInputBuffer[0]), 10, &ReturnSize);
        if((res != FR_OK) || f_eof(&mp3FileObject)) {
            printf("Stop !(%x)\n\r", res);
            return -1;
        }
        i32Offset = 0;
        if ( (MadInputBuffer[0] == 'I') && (MadInputBuffer[1] == 'D') && (MadInputBuffer[2] == '3') )
        {
            i32Offset = (i32Offset << 7) | (MadInputBuffer[6] & 0x7F);
            i32Offset = (i32Offset << 7) | (MadInputBuffer[7] & 0x7F);
            i32Offset = (i32Offset << 7) | (MadInputBuffer[8] & 0x7F);
            i32Offset = (i32Offset << 7) | (MadInputBuffer[9] & 0x7F);
            i32Offset += 10;
        }
        printf("%d byte(s) to the 1st frame.\n", i32Offset);

        //			while(i++<=500)
        audioInfo.mp3SampleRate=0;
        i = f_size(&mp3FileObject);
        f_lseek(&mp3FileObject, i/2);
        while (audioInfo.mp3SampleRate==0)
        {
            res = f_read(&mp3FileObject, (char *)(&MadInputBuffer[0]), FILE_IO_BUFFER_SIZE, &ReturnSize);
            if ((res != FR_OK) && (audioInfo.mp3SampleRate==0))
            {
                f_close(&mp3FileObject);
                printf("Open File Error\r\n");
                return -1;
            }
            //parsing MP3 header
            mp3CountV1L3Headers((unsigned char *)(&MadInputBuffer[0]), ReturnSize);
        }
        audioInfo.mp3SampleRate=0;
        while (audioInfo.mp3SampleRate==0)
        {
            res = f_read(&mp3FileObject, (char *)(&MadInputBuffer[0]), FILE_IO_BUFFER_SIZE, &ReturnSize);
            if ((res != FR_OK) && (audioInfo.mp3SampleRate==0))
            {
                f_close(&mp3FileObject);
                printf("Open File Error\r\n");
                return -1;
            }
            //parsing MP3 header
            mp3CountV1L3Headers((unsigned char *)(&MadInputBuffer[0]), ReturnSize);
        }
    } else {
        f_close(&mp3FileObject);
        printf("Open File Error\r\n");
        return -1;
    }
    f_close(&mp3FileObject);

    printf("====[MP3 Info]======\r\n");
    printf("FileSize = %d\r\n", audioInfo.playFileSize);
    printf("SampleRate = %d\r\n", audioInfo.mp3SampleRate);
    printf("BitRate = %d\r\n", audioInfo.mp3BitRate);
    printf("Channel = %d\r\n", audioInfo.mp3Channel);
    printf("PlayTime = %d\r\n", audioInfo.mp3PlayTime);
    printf("=====================\r\n");

    return i32Offset;
}

// Enable I2S TX with PDMA function
void StartPlay(void)
{
    printf("Start playing ...\n");
    I2S_SET_TXDMA_STADDR(I2S, (uint32_t) &aPCMBuffer[0]);													// Tx Start Address
    I2S_SET_TXDMA_THADDR(I2S, (uint32_t) &aPCMBuffer[0][PCM_BUFFER_SIZE-1]);			// Tx Threshold Address
    I2S_SET_TXDMA_EADDR(I2S, (uint32_t) &aPCMBuffer[1][PCM_BUFFER_SIZE-1]);			// Tx End Address
    I2S_ENABLE_TXDMA(I2S);
    I2S_ENABLE_TX(I2S);


    I2S_EnableInt(I2S, (I2S_IEN_TDMATIEN_Msk|I2S_IEN_TDMAEIEN_Msk));
    NVIC_EnableIRQ(I2S_IRQn);

    audioInfo.mp3Playing = 1;
}

// Disable I2S TX with PDMA function
void StopPlay(void)
{
    I2S_DISABLE_TXDMA(I2S);
    I2S_DISABLE_TX(I2S);

    audioInfo.mp3Playing = 0;
    printf("Stop ...\n");
}

// MP3 decode player
void MP3Player(void)
{
    FRESULT res;
    uint8_t *ReadStart;
    uint8_t *GuardPtr;
    volatile uint8_t u8PCMBufferTargetIdx = 0;
    volatile uint32_t pcmbuf_idx, i;
    volatile unsigned int Mp3FileOffset=0;
    uint16_t sampleL, sampleR;
    int32_t i32Offset;
    ///	FIL file2;
    ///	UINT s2;

    pcmbuf_idx = 0;
    memset((void *)&audioInfo, 0, sizeof(audioInfo));

    /* Parse MP3 header */
    i32Offset = MP3_ParseHeaderInfo(MP3_FILE);

    if ( i32Offset < 0 )
        return;
    else
    {
        /* Open MP3 file */
        res = f_open(&mp3FileObject, MP3_FILE, FA_OPEN_EXISTING | FA_READ);
        ///	f_open(&file2, MP3_FILE2, FA_CREATE_ALWAYS | FA_WRITE);
        if (res != FR_OK) {
            printf("Open file error \r\n");
            return;
        }

        f_lseek(&mp3FileObject, i32Offset);

        if ( audioInfo.mp3SampleRate % 11025 )
        {
            CLK_SET_APLL(CLK_APLL_49152031);
        }

        I2S_Open(I2S, I2S_MODE_MASTER, audioInfo.mp3SampleRate, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, I2S_ENABLE_INTERNAL_CODEC);
        I2S_EnableMCLK(I2S, audioInfo.mp3SampleRate*256);

        /* Initialize Internal CODEC */
        InternalCODEC_Setup();

        // FIXME mono application
        if ( audioInfo.mp3Channel == 1 )
            I2S_SET_INTERNAL_CODEC(I2S, 0x01, 0x90);			// DAC Open
    }
#if 1
    /* First the structures used by libmad must be initialized. */
    mad_stream_init(&Stream);
    mad_frame_init(&Frame);
    mad_synth_init(&Synth);

    while(1) {
        if(Stream.buffer==NULL || Stream.error==MAD_ERROR_BUFLEN) {
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

stop:

    printf("Exit MP3\r\n");

    mad_synth_finish(&Synth);
    mad_frame_finish(&Frame);
    mad_stream_finish(&Stream);

    f_close(&mp3FileObject);
    ///	f_close(&file2);
    StopPlay();
#endif
}

