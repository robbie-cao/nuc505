/**************************************************************************//**
 * @file        main.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Demonstrate how to launch a program via loader which is located fully on SRAM.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

#if defined ( __CC_ARM )
static __align(32) const uint8_t g_au8RamImg[] __attribute__((section("ramimg")));
static __align(32) const uint8_t g_au8RamImg[] = {
#   include "FullOnSRAM.dat"
};

#elif defined (__ICCARM__)
#pragma data_alignment=32
static const uint8_t g_au8RamImg[] @ "ramimg";
static const uint8_t g_au8RamImg[] = {
#   include "FullOnSRAM.dat"
};

#endif

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
    printf("|           NUC505 Series Loader Sample          |\n");
    printf("+------------------------------------------------+\n");
   
    /* Run FullOnSRAM. */
    {
#if defined ( __CC_ARM )
        extern uint32_t Load$$ER_RAMIMG$$RO$$Base[];
        extern uint32_t Load$$ER_RAMIMG$$RO$$Length[];
        extern uint32_t Image$$ER_RAMIMG$$RO$$Base[];
        
        printf("Load image(0x%08x, %d bytes) to 0x%08x.\n", Load$$ER_RAMIMG$$RO$$Base, Load$$ER_RAMIMG$$RO$$Length, Image$$ER_RAMIMG$$RO$$Base);
        memcpy((void *) Image$$ER_RAMIMG$$RO$$Base, Load$$ER_RAMIMG$$RO$$Base, (unsigned long) Load$$ER_RAMIMG$$RO$$Length);
        
#elif defined (__ICCARM__)
        #pragma section = "ramimg"
        #pragma section = "ramimg_init"
        
        printf("Load image(0x%08x, %d bytes) to 0x%08x.\n", __section_begin("ramimg_init"), __section_size("ramimg"), __section_begin("ramimg"));
        memcpy((void *) __section_begin("ramimg"), __section_begin("ramimg_init"), (unsigned long) __section_size("ramimg"));

#endif
        
        printf("Remap SRAM(0x%08X, %d KB) to 0x00000000 via VECMAP.\n", g_au8RamImg, 128);
        printf("Reset CPU to run loaded image.\n");
        printf("\n\n");
        
        SYS->LVMPADDR = (uint32_t) g_au8RamImg;         // Specify load VECMAP address.
        SYS->LVMPLEN = 128;                             // Specify load VECMAP length to total size of SRAM.
        SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;           // Reset CPU. VECMAP will take effect on CPU reset.
    }
    while (1);
    //return 0;
}
