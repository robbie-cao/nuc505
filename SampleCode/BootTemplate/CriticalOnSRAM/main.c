/**************************************************************************//**
 * @file        main.c
 * @version     V1.00
 * $Revision:   1$
 * $Date:       14/07/10 5:00p$
 * @brief       Demonstrate how to locate a program mainly on SPI Flash for typical use, 
 *              except critical parts on SRAM for fast execution.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
//#include <time.h>
#include "NUC505Series.h"

/* My clock tick functions */
#define MYCLOCKS_PER_SEC    100
static volatile uint32_t u32TickCount = 0;
void SysTick_Handler(void)
{
    u32TickCount ++;
}

void MyClock_Init(void)
{
    SysTick_Config(SystemCoreClock/100);
}

extern uint32_t MyClock(void)
{
    return u32TickCount;
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

#define _Fibonacci(n, fn)  \
    do {    \
        int i, fn_1 = 0, fn_2 = 1;  \
        fn = 0; \
        for (i = 0 ; i < n; i++) {  \
            if (i <= 1) {   \
                fn = i; \
            }   \
            else {  \
                fn = fn_1 + fn_2;   \
                fn_1 = fn_2;    \
                fn_2 = fn;  \
            }   \
        }   \
    } while (0)
    
int Fibonacci1(int n)
{
    int fn;
    
    _Fibonacci(n, fn);
    return fn;
}

#if defined ( __CC_ARM )
#pragma arm section code="fastcode"
int Fibonacci2(int n)
#elif defined ( __ICCARM__ )
int Fibonacci2(int n)   @ "fastcode"
#else
int Fibonacci2(int n)
#endif
{
    int fn;
    
    _Fibonacci(n, fn);
    return fn;
}

#if defined ( __CC_ARM )
#pragma arm section
#endif

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
    
    /* Load code in SRAM dynamically by user for fast execution. */
    {
#if defined ( __CC_ARM )
        extern uint32_t Load$$ER_FASTCODE_UNINIT$$RO$$Base[];
        extern uint32_t Load$$ER_FASTCODE_UNINIT$$RO$$Length[];
        extern uint32_t Image$$ER_FASTCODE_UNINIT$$RO$$Base[];
        
        printf("Load Fibonacci2() in SRAM (0x%08X) for fast execution.\n", Fibonacci2);
        memcpy((void *) Image$$ER_FASTCODE_UNINIT$$RO$$Base, Load$$ER_FASTCODE_UNINIT$$RO$$Base, (unsigned long) Load$$ER_FASTCODE_UNINIT$$RO$$Length);

#elif defined (__ICCARM__)
        #pragma section = "fastcode"
        #pragma section = "fastcode_init"
        
        printf("Load Fibonacci2() in SRAM (0x%08X) for fast execution.\n", Fibonacci2);
        memcpy((void *) __section_begin("fastcode"), __section_begin("fastcode_init"), (unsigned long) __section_size("fastcode"));

#endif        
    }
    
    /* Compare performance between on SPI Flash and on SRAM. */
    {
        uint32_t t1, t2;
        const int n = 500000;
        
        MyClock_Init();
        
        t1 = MyClock();
        Fibonacci1(n);
        t2 = MyClock();
        printf("Fibonacci1(%d) (on SPI Flash) takes %d (ms).\n", n, (t2 - t1) * 1000 / MYCLOCKS_PER_SEC);
        
        t1 = MyClock();
        Fibonacci2(n);
        t2 = MyClock();
        printf("Fibonacci2(%d) (on SRAM) takes %d (ms).\n", n, (t2 - t1) * 1000 / MYCLOCKS_PER_SEC);
    }
    
    while (1);
    //return 0;
}


