/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 Power Down Consumption Sample Code
 *           Connect PB.10 to VSS will wake up system form Power-down mode
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "gpio.h"

#if defined ( __CC_ARM )
void EINT0_IRQHandler(void)
{
    int32_t i;
    i = GPIO_GET_INT_FLAG(PB, BIT10);
    GPIO->INTSTSA_B = (i << 16);

}
#elif defined (__ICCARM__)
void EINT0_IRQHandler_SRAM(void)
{
    int32_t i=8;
    i = GPIO_GET_INT_FLAG(PB, BIT10);
    GPIO->INTSTSA_B = (i << 16);
}
#endif




void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetCoreClock(96000000);
    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PCLK Divider */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);		/* PCLK divider = 1 */

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}
extern void DisableIPs(void);
/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|    GPIO Power-Down and Wake-up by PB.10 Sample Code    |\n");
    printf("+-------------------------------------+ \n");
    printf("     Press any key to start test \n\n");
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
        uint32_t* pu32Src;
        uint32_t* pu32Dst;

        pu32Src = (uint32_t *)&EINT0_IRQHandler_SRAM;
        printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", __section_begin("VECTOR2"));
        memcpy((void *) __section_begin("VECTOR2"), (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) __section_begin("VECTOR2");

        /* Change EINT0 vector to interrupt handler in SRAM */
        /* IAR compiler doesn't following initial configuration file to relocate EINT0_IRQHandler() */
        pu32Dst = (uint32_t*) ((uint32_t)__section_begin("VECTOR2")+0x50);
        *pu32Dst = (uint32_t)pu32Src;

#endif
    }
    /* Configure PB10 as Input mode pull up enable and enable interrupt by falling edge trigger */
    GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PB, BIT10, GPIO_PULL_UP_EN);
    GPIO_EnableInt(PB, 10, GPIO_INT_FALLING);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(0, GPIO_DBCTL_DBCLKSEL_128);
    GPIO_ENABLE_DEBOUNCE(PB, GPIO_DBCTL_EINT0_DBEN_MASK);

    /* Enable the wake up function of EINT0 */
    GPIO_ENABLE_WAKE_UP(GPIO_INTCTL_EINT0_WKEN_MASK);
    SYS_ENABLE_WAKEUP(SYS_WAKEUP_GPIOWE_Msk);

    /* Configure PB10 as EINT0 */
    GPIO_SetIntGroup(PB, 10, GPIO_INTSRCGP_EINT0);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Waiting for PB.10 falling-edge interrupt event */
    while(1)
    {
        printf("Enter to Power-Down ......\n");
        /* Turn off unnecessay IP as power down */
        DisableIPs();

        printf("Pressing any key to test again\n");
        getchar();
    }

}


