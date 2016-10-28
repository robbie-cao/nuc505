/**************************************************************************//**
 * @file        main.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Special notes for code running on SPI Flash:
 *              1. Switch to different clock safely, especially higher system clock.
 *              2. Embed MTP signature in the predefined location for security function.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

/* CLK_SysTickDelay() will enable SysTick IRQ and program will hang in the default implementation of SysTick_Handler().
   So a null implementation of SysTick_Handler() is given to overwrite the default when there are calls to CLK_SysTickDelay(). */
void SysTick_Handler(void)
{
}

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
    /* SPIM IP should have been enabled for demo of running SPI Flash code */

    /* Select IP clock source */
    /* PCLK divider = 1 (/2) */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);
    /* UART0 clock source = XIN */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    //CLK->CLKDIV3 &= ~(CLK_CLKDIV3_UART0DIV_Msk | CLK_CLKDIV3_UART0SEL_Msk);
    
    /* Update System Core Clock */
    /* Note too high system clock will cause over-spec of SPI Flash read command on running SPIROM code. */
    CLK_SetCoreClock(100000000);
    SystemCoreClockUpdate();
    
    /* Init I/O multi-function pins */
    /* Configure multi-function pins for UART0 RXD and TXD */
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
	SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;
    
    /* Lock protected registers */
    //SYS_LockReg();
        
}

/* Code that will impact SPIM DMM is placed in the spimcode section, 
   which will be located at RAM instead of SPIROM at execution time. */
#if defined ( __CC_ARM )
//#pragma arm section code="spimcode"
//#pragma arm section
void SPIMCode_DelayMicroSec(uint32_t u32Delay) __attribute__((section("spimcode")));
void SPIMCode_SetBusClock(uint32_t u32SysClk, uint32_t u32SClk) __attribute__((section("spimcode")));
uint32_t SPIMCode_GetBusClock(void) __attribute__((section("spimcode")));
void SPIMCode_ConfigDMMMode(uint32_t u32SysClk, uint32_t u32SClk, uint32_t u32RdCmdCode) __attribute__((section("spimcode")));
#elif defined (__ICCARM__)
void SPIMCode_DelayMicroSec(uint32_t u32Delay) @ "spimcode";
void SPIMCode_SetBusClock(uint32_t u32SysClk, uint32_t u32SClk) @ "spimcode";
uint32_t SPIMCode_GetBusClock(void) @ "spimcode";
void SPIMCode_ConfigDMMMode(uint32_t u32SysClk, uint32_t u32SClk, uint32_t u32RdCmdCode) @ "spimcode";
#endif

void SPIMCode_DelayMicroSec(uint32_t u32Delay)
{
    /* CLK_SysTickDelay is located in SPIROM, so inline this function into SPIMCode_DelayMicroSec. */
    SysTick->LOAD = u32Delay * CyclesPerUs;
    SysTick->VAL  =  (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);   // Wait for down-count to zero.
}

/**
  * This function is the same as SPIM_SetBusClock except:
  * 1) Set SPI bus clock based on pass-in system clock instead of current system clock.
  * 2) Located in RAM and doesn't call outside functions.
  */
void SPIMCode_SetBusClock(uint32_t u32SysClk, uint32_t u32SClk)
{
    uint32_t u32Divider;
        
    if (u32SClk) {
        u32Divider = u32SysClk / (u32SClk * 2);
        if (u32Divider) {
            if (u32SClk < (u32SysClk / (u32Divider * 2))) { // Not divisible.
                u32Divider ++;
            }
        }
        else {
            if (u32SClk < u32SysClk) {  // u32SysClk x (1/2) < u32SClk < u32SysClk
                u32Divider ++;
            }
        }
    }
    else {
        u32Divider = 65535;
    }

    SPIM->CTL1 = (SPIM->CTL1 & (~SPIM_CTL1_DIVIDER_Msk)) | (u32Divider << SPIM_CTL1_DIVIDER_Pos);
}

/**
  * This function is the same as SPIM_SetBusClock except located in RAM and doesn't call outside functions.
  */
uint32_t SPIMCode_GetBusClock(void)
{
    uint32_t u32Divider = ((SPIM->CTL1 & SPIM_CTL1_DIVIDER_Msk) >> SPIM_CTL1_DIVIDER_Pos);
    
    return u32Divider ? SystemCoreClock / (u32Divider * 2) : SystemCoreClock;
}

void SPIMCode_ConfigDMMMode(uint32_t u32SysClk, uint32_t u32SClk, uint32_t u32RdCmdCode)
{   
    /* SPIM H/W may still be in operation due to in DMM mode, delay at least 250 peripheral cycles (SPI bus cycles). */
    SPIMCode_DelayMicroSec(250 * 1000000 / SPIMCode_GetBusClock());
    
    /* To avoid running SPIROM code just below at over-spec clock. */
    if (u32SysClk > SystemCoreClock) {
        SPIMCode_SetBusClock(u32SysClk, 45000000);
    }
    
    /* Change system core clock. Note this will call out SPIROM code. */
    CLK_SetCoreClock(u32SysClk);
    SystemCoreClockUpdate();
    
    /* Re-delay because CLK_SetCoreClock/SystemCoreClockUpdate are located in SPIROM. */
    SPIMCode_DelayMicroSec(250 * 1000000 / SPIMCode_GetBusClock());
    
    /* Change read command of SPI Flash. */
    SPIM_ENABLE_DMM_MODE(SPIM, u32RdCmdCode, 0);
    
    /* Change SPI bus clock. */
    SPIMCode_SetBusClock(SystemCoreClock, u32SClk);
}
//#pragma arm section

/* Declaration of MTP signature. Location of it is specified in scatter file. */
#if defined ( __CC_ARM )
const uint32_t MTPSIG __attribute__((section("mtpsig"), used));
#elif defined (__ICCARM__)
const uint32_t MTPSIG @ "mtpsig";
#endif

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
    printf("SPI bus clock\t\t\t\t%dHz\n", SPIM_GetBusClock(SPIM));
    printf("SPI Flash read command\t\t\t0x%02X\n", SPIM_CTL0_CMDCODE_READ_DATA >> SPIM_CTL0_CMDCODE_Pos);
    printf("\n");
    
    /* Note: Refer to SPI Flash spec for clock limit of different read commands. */
    
    SPIMCode_ConfigDMMMode(90000000, 45000000, SPIM_CTL0_CMDCODE_READ_DATA);
    printf("System core clock\t\t\t%dHz\n", SystemCoreClock);
    printf("SPI bus clock\t\t\t\t%dHz\n", SPIM_GetBusClock(SPIM));
    printf("SPI Flash read command\t\t\t0x%02X\n", SPIM_CTL0_CMDCODE_READ_DATA >> SPIM_CTL0_CMDCODE_Pos);
    printf("\n");
    
    SPIMCode_ConfigDMMMode(75000000, 75000000, SPIM_CTL0_CMDCODE_FAST_READ);
    printf("System core clock\t\t\t%dHz\n", SystemCoreClock);
    printf("SPI bus clock\t\t\t\t%dHz\n", SPIM_GetBusClock(SPIM));
    printf("SPI Flash read command\t\t\t0x%02X\n", SPIM_CTL0_CMDCODE_FAST_READ >> SPIM_CTL0_CMDCODE_Pos);
    printf("\n");
    
    SPIMCode_ConfigDMMMode(60000000, 60000000, SPIM_CTL0_CMDCODE_FAST_READ_DUAL_OUT);
    printf("System core clock\t\t\t%dHz\n", SystemCoreClock);
    printf("SPI bus clock\t\t\t\t%dHz\n", SPIM_GetBusClock(SPIM));
    printf("SPI Flash read command\t\t\t0x%02X\n", SPIM_CTL0_CMDCODE_FAST_READ_DUAL_OUT >> SPIM_CTL0_CMDCODE_Pos);
    printf("\n");
    
    SPIMCode_ConfigDMMMode(60000000, 60000000, SPIM_CTL0_CMDCODE_FAST_READ_QUAD_IO);
    printf("System core clock\t\t\t%dHz\n", SystemCoreClock);
    printf("SPI bus clock\t\t\t\t%dHz\n", SPIM_GetBusClock(SPIM));
    printf("SPI Flash read command\t\t\t0x%02X\n", SPIM_CTL0_CMDCODE_FAST_READ_QUAD_IO >> SPIM_CTL0_CMDCODE_Pos);
    printf("\n");
    
    printf("[SPIM][SPIROM]\t\t\t\tPASSED\n");
    
    while (1);
    //return 0;
}

/* Definition of MTP signature. */
const uint32_t MTPSIG = 0x8901c2a0;

