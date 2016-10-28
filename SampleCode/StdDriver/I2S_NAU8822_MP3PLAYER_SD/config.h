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

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

#define PCM_BUFFER_SIZE        2304/2
#define FILE_IO_BUFFER_SIZE    1024*2

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
void ExternalCODEC_Setup(void);
void MP3Player(void);
#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
