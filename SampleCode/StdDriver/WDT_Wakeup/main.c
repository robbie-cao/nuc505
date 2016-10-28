/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 14/05/30 6:08p $
 * @brief    Use WDT to wake system up from power-down mode periodically
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"


void WDT_IRQHandler(void)
{

    // Clear WDT interrupt flag
    WDT_CLEAR_TIMEOUT_INT_FLAG();

    // Check WDT wake up flag
    if(WDT_GET_TIMEOUT_WAKEUP_FLAG()) {
        printf("Wake up by WDT\n");
        // Clear WDT wake up flag
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

    /* To avoid the synchronization issue between system and APB clock domain */
    WDT_GET_TIMEOUT_WAKEUP_FLAG();

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
    CLK_EnableModuleClock(WDT_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    CLK_SetModuleClock(WDT_MODULE, CLK_WDT_SRC_RTC, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}


int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrate using WDT to wake system up from power down mode\n");

    // WDT timeout every 2^14 WDT clock, disable system reset, enable wake up system
    WDT_Open(WDT_TIMEOUT_2POW14, 0, FALSE, TRUE);

    SYS_ENABLE_WAKEUP(1<<SYS_WAKEUP_WDTWE_Pos);

    // Enable WDT timeout interrupt
    WDT_EnableInt();
    NVIC_EnableIRQ(WDT_IRQn);

    while(1) {
        // Wait 'til UART FIFO empty to get a cleaner console out
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_PowerDown();
    }

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/


