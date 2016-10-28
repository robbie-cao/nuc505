
/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 14/05/30 6:04p $
 * @brief    Use pin PA.12 to demonstrates timer event counter function
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

void TMR0_IRQHandler(void)
{

    printf("Count 1000 falling events! Test complete\n");
    TIMER_ClearIntFlag(TIMER0);

    /* To avoid the synchronization issue between system and APB clock domain */
    TIMER_GetIntFlag(TIMER0);

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
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMR0_SRC_EXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

    /* Set Timer event counting pin */
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~SYS_GPA_MFPH_PA12MFP_Msk) ) | SYS_GPA_MFPH_PA12MFP_TM0_CNT_OUT;

}


int main(void)
{
    int i;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code use TMR0_CNT_OUT(PA.12) to count PA.13 input event\n");
    printf("Please connect PA.12 to PA.13, press any key to continue\n");
    getchar();

    PA->DOUT |= 1 << GPIO_DOUT_DOUT13_Pos;                     // Set init state to high
    PA->MODE |= BIT13;  // Set to output mode

    // Give a dummy target frequency here. Will over write prescale and compare value with macro
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    // Update prescale and compare value to what we need in event counter mode.
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 1000);

    // Counter increase on falling edge
    TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_FALLING_EDGE);
    // Start Timer 0
    TIMER_Start(TIMER0);
    // Enable timer interrupt
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);


    for(i = 0; i < 1000; i++) {
        PA->DOUT &= ~(1 << GPIO_DOUT_DOUT13_Pos); // low
        PA->DOUT |= (1 << GPIO_DOUT_DOUT13_Pos);  // high
    }

    while(1);

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


