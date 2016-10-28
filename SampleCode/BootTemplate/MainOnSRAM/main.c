/**************************************************************************//**
 * @file        main.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Demonstrate how to locate a program mainly on SRAM for fast execution,
 *              except startup parts on SPI Flash for initialization.
 *
 * @note        The main() function cannot be debugged until C startup has completed.
 *              It is because C startup will be responsible for copying main() from ROM to RAM.
 *              So don't check the Run to main debug option or set BP before C startup is completed.
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
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
    //CLK->APBCLK = CLK_APBCLK_UART0CKEN_Msk;     // Enable UART0 IP clock.

    /* Select IP clock source */
    /* PCLK divider = 1 (/2) */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);
    /* UART0 clock source = XIN */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    //CLK->CLKDIV3 &= ~(CLK_CLKDIV3_UART0DIV_Msk | CLK_CLKDIV3_UART0SEL_Msk);
    
    /* Update System Core Clock */
    /* Note too high system clock will cause over-spec of SPI Flash read command on running code on SPI Flash. */
    CLK_SetCoreClock(100000000);
    SystemCoreClockUpdate();
    
    /* Init I/O multi-function pins */
    /* Configure multi-function pins for UART0 RXD and TXD */
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;
        
    /* Lock protected registers */
    //SYS_LockReg();
        
}

int main(void)
{
    
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
    
    printf("+------------------------------------------------+\n");
    printf("|           NUC505 Series Demo Sample            |\n");
    printf("+------------------------------------------------+\n");
    
    /* Relocate vector table in SRAM for fast interrupt handling. */
    {
#if defined ( __CC_ARM )
        extern uint32_t __Vectors[];
        extern uint32_t __Vectors_Size[];
        extern uint32_t Image$$ER_VECTOR2$$ZI$$Base[];
    
        printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", Image$$ER_VECTOR2$$ZI$$Base);
        memcpy((void *) Image$$ER_VECTOR2$$ZI$$Base, (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) Image$$ER_VECTOR2$$ZI$$Base;
        
#elif defined (__ICCARM__)
        #pragma section = "VECTOR2"
        extern uint32_t __Vectors[];
        extern uint32_t __Vectors_Size[];
        
        printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", __section_begin("VECTOR2"));
        memcpy((void *) __section_begin("VECTOR2"), (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) __section_begin("VECTOR2");
        
#endif
    }
    
    printf("Main code is now running on SRAM from SPI Flash for fast execution.\n");
    
    while (1);
    //return 0;
}


