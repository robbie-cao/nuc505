
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 14/05/30 6:01p $
 * @brief    Demonstrate the dead-zone feature with PWM.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

void PWM0_IRQHandler(void);

void PWM0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100) {
        if(out)
            PWM_EnableOutput(PWM, 0xF);
        else
            PWM_DisableOutput(PWM, 0xF);
        out ^= 1;
        cnt = 0;
    }
    // Clear channel 0 period interrupt flag
    PWM_ClearPeriodIntFlag(PWM, 0);

    /* To avoid the synchronization issue between system and APB clock domain */
    PWM_GetPeriodIntFlag(PWM, 0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(96000000);

    /* Set PCLK divider */
    CLK_SetModuleClock(PCLK_MODULE, NULL, 1);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(PWM_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

    /* Set EXT as PWM channel 0~3 clock source */
    CLK_SetModuleClock(PWM_MODULE,CLK_PWM_SRC_EXT,0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

    /* Set GPB10~GPB13 multi-function pins for PWM Channel 0~3 */
    SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk) ) | SYS_GPB_MFPH_PB10MFP_PWM_CH0;
    SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk) ) | SYS_GPB_MFPH_PB11MFP_PWM_CH1;
    SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk) ) | SYS_GPB_MFPH_PB12MFP_PWM_CH2;
    SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk) ) | SYS_GPB_MFPH_PB13MFP_PWM_CH3;

}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code will output PWM channel 0 to with different\n");
    printf("frequency and duty, enable dead zone function of all PWM pairs.\n");
    printf("And also enable/disable PWM output every 1 second.\n");
    // PWM0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM, 0, 100, 30);
    PWM_EnableDeadZone(PWM, 0, 100);

    // PWM2 frequency is 300Hz, duty 50%
    PWM_ConfigOutputChannel(PWM, 2, 300, 50);
    PWM_EnableDeadZone(PWM, 2, 200);

    // Enable output of all PWM channels
    PWM_EnableOutput(PWM, 0xF);

    // Enable PWM channel 0 period interrupt, use channel 0 to measure time.
    PWM_EnablePeriodInt(PWM, 0, 0);
    NVIC_EnableIRQ(PWM0_IRQn);

    // Start
    PWM_Start(PWM, 0xF);

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


