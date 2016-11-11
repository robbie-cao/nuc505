/**************************************************************************//**
 * @file     config.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 I2S Driver Sample Configuration Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

#include "ff.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

#define PCM_BUFFER_SIZE 1024

void WAVPlayer(const char * fileName);
void InternalCODEC_Setup(void);
void WAVMixPlayer(const char * fileName1,const char * fileName2);
void WAVPlay_Stop(void);

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
