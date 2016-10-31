/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/05/29 1:14p $
 * @brief    NUC472/NUC442 I2S Driver Sample Configuration Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H
#include "TypeDefine.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* Size of one of two audio DMA buffers for ping-pong buffering. */
#define FSTLVL_BUFF_LEN     (160 * 2)
/* Size of 2nd level buffer. */
#define SECLVL_BUFF_LEN     (32)


#define MP3APP_PCM_BUF_SIZE  		(32)

#define TX_SAMPLE_COUNT              1024
#define TX_SAMPLE_SIZE               (TX_SAMPLE_COUNT * 2)

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void I2S_Init(void);
void demo_InternalDAC(void);

struct mp3Header {
    unsigned int sync : 11;
    unsigned int version : 2;
    unsigned int layer : 2;
    unsigned int protect : 1;
    unsigned int bitrate : 4;
    unsigned int samfreq : 2;
    unsigned int padding : 1;
    unsigned int private : 1;
    unsigned int channel : 2;
    unsigned int mode : 2;
    unsigned int copy : 1;
    unsigned int original : 1;
    unsigned int emphasis : 2;
};

struct AudioInfoObject {
    unsigned int playFileSize;
    unsigned int mp3FileEndFlag;
    unsigned int mp3SampleRate;
    unsigned int mp3BitRate;
    unsigned int mp3Channel;
    unsigned int mp3PlayTime;
    unsigned int mp3Playing;
};

int mp3CountV1L3Headers(unsigned char *pBytes, size_t size);

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
