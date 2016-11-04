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

#define PCM_BUFFER_SIZE 2*1024

#define BUFF_LEN    	2*512 //512

void WAVPlayer(void);
void InternalCODEC_Setup(void);

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
