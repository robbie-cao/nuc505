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

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

#define PCM_BUFFER_SIZE 4*1024

void WAVPlayer(void);

void WAU8822_Setup(void);
void WAU8822_ConfigSampleRate(uint32_t u32SampleRate);

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
