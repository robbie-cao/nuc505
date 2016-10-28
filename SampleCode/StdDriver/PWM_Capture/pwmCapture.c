
/******************************************************************************
 * @file     pwmCapture.c
 * @version  V1.00
 * $Revision: 17 $
 * $Date: 14/05/30 6:01p $
 * @brief    NUC505 Series PWM Driver Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

#define SAMPLE_CNT 32

uint8_t volatile cap_index = 0;
uint32_t cap_val[SAMPLE_CNT >> 1][2];

void PWM_Capture(void);

void PWM_Capture(void)
{
    static uint8_t token = 0;
    uint32_t u32CapIntFlag;
    uint8_t u8Count = cap_index;

    if(u8Count >= SAMPLE_CNT) {
        // Disable PWM channel 2 rising and falling edge capture interrupt
        PWM_DisableCaptureInt(PWM,2,PWM_RISING_FALLING_LATCH_INT_ENABLE);
        return;
    }

    // Get channel 2 capture interrupt flag
    u32CapIntFlag = PWM_GetCaptureIntFlag(PWM, 2);

    // Rising latch condition happened
    if ((u32CapIntFlag & PWM_RISING_LATCH_INT_FLAG) && token == 0) {
        cap_val[u8Count >> 1][0] = PWM_GET_CAPTURE_RISING_DATA(PWM, 2);
        cap_index++;
        token = 1;
    }
    // Falling latch condition happened
    if ((u32CapIntFlag & PWM_FALLING_LATCH_INT_FLAG) && token == 1) {
        cap_val[u8Count >> 1][1] = PWM_GET_CAPTURE_FALLING_DATA(PWM, 2);
        cap_index++;
        token = 0;
    }

    // Clear channel 2 capture interrupt flag
    PWM_ClearCaptureIntFlag(PWM, 2, PWM_RISING_FALLING_LATCH_INT_FLAG);

    /* To avoid the synchronization issue between system and APB clock domain */
    u32CapIntFlag = PWM_GetCaptureIntFlag(PWM, 2);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


