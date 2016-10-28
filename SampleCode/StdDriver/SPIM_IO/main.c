/**************************************************************************//**
 * @file        main.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Demonstrate how to read/write SPI Flash in SPIM (SPI Master) I/O mode.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

void SYS_Init(void)
{

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    //SYS_UnlockReg();
     
    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;
    
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPIM_MODULE);
    //CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk;     // Enable UART0 IP clock.
    //CLK->AHBCLK |= CLK_AHBCLK_SPIMCKEN_Msk;     // Enable SPIM IP clock.

    /* Select IP clock source */
    /* PCLK divider = 1 (/2) */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);
    /* UART0 clock source = XIN */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    //CLK->CLKDIV3 &= ~(CLK_CLKDIV3_UART0DIV_Msk | CLK_CLKDIV3_UART0SEL_Msk);
    
    /* Update System Core Clock */
    CLK_SetCoreClock(100000000);
    SystemCoreClockUpdate();
    
    /* Init I/O multi-function pins */
    /* Configure multi-function pins for UART0 RXD and TXD */
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;
    /* Configure multi-function pins for SPIM, Slave I/F=GPIO. */
#if 0
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA8MFP_Msk | (1 << SYS_GPA_MFPH_PA8MFP_Pos);    // SPIM_SS
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA9MFP_Msk | (1 << SYS_GPA_MFPH_PA9MFP_Pos);    // SPIM_SCLK
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA10MFP_Msk | (1 << SYS_GPA_MFPH_PA10MFP_Pos);  // SPIM_D0
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA11MFP_Msk | (1 << SYS_GPA_MFPH_PA11MFP_Pos);  // SPIM_D1
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA12MFP_Msk | (1 << SYS_GPA_MFPH_PA12MFP_Pos);  // SPIM_D2
    SYS->GPA_MFPH = SYS->GPA_MFPH & ~SYS_GPA_MFPH_PA13MFP_Msk | (1 << SYS_GPA_MFPH_PA13MFP_Pos);  // SPIM_D3
#endif
    
    /* Lock protected registers */
    //SYS_LockReg();
        
}

extern void TestIOMode(void);

int main(void)
{
    
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
    
    printf("+------------------------------------------------+\n");
    printf("|           NUC505 Series SPIM Sample            |\n");
    printf("+------------------------------------------------+\n");
    printf("System core clock\t\t\t%dHz\n", SystemCoreClock);

    TestIOMode();       // Test SPIM I/O mode.
    
    while (1);
    //return 0;
}




