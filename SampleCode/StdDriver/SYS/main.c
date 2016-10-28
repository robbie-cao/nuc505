/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/24 4:08p $
 * @brief    NUC505 Series Global Control and Clock Control Driver Sample Code
 *
 * 1.	Demonstrate delay function by systick.
 * 2.	Demonstrate core clock switching.
 * 3.	Demonstrate how to enable module clock and set module clock divider.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"
void demo_SysTickDelay(void);
void demo_SysHclkSwitch(void);
void demo_ModuleClock(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set Core Clock */
    CLK_SetCoreClock(64000000);
    /* Update System Core Clock */
    SystemCoreClockUpdate();
    /* Set PCLK Divider = 1 (/2) */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);
}

void UART0_Init(void)
{
    /* Enable UART1 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* UART1 module clock from EXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    /* Set APLL to 49.142MHz for 8k, 12k, 16k, 24k, 32k, 48k and 96k sample rate */
    CLK_SET_APLL(CLK_APLL_49142000);
    printf("APLL = %d\n", CLK_GetAPLLClockFreq());

    printf("+------------------------------------------------+\n");
    printf("|           NUC505 Series SYS Sample            |\n");
    printf("+------------------------------------------------+\n");
    printf("CPU clock\t\t\t\t%dHz\n", SystemCoreClock);
    demo_SysTickDelay();
    demo_SysHclkSwitch();
    demo_ModuleClock();
    printf("SYS sample code done\n");
    while(1);
}




