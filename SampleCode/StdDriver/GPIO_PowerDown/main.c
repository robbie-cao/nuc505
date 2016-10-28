/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 General Purpose I/O Driver Sample Code
 *           Connect PB.10 to VSS will wake up system form Power-down mode
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"
#include "gpio.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void EINT0_IRQHandler(void)
{
    int32_t u32Flag;

    u32Flag = GPIO_GET_INT_FLAG(PB, BIT10);
    GPIO->INTSTSA_B = (u32Flag << 16);

    /* To avoid the synchronization issue between system and APB clock domain */
    u32Flag = GPIO->INTSTSA_B;

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

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}

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
    getchar();

    /* Configure PB10 as Input mode pull up enable and enable interrupt by falling edge trigger */
    GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PB, BIT10, GPIO_PULL_UP_EN);
    GPIO_EnableInt(PB, 10, GPIO_INT_FALLING);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(NULL, GPIO_DBCTL_DBCLKSEL_128);
    GPIO_ENABLE_DEBOUNCE(PB, GPIO_DBCTL_EINT0_DBEN_MASK);

    /* Enable the wake up function of EINT0 */
    GPIO_ENABLE_WAKE_UP(GPIO_INTCTL_EINT0_WKEN_MASK);
    SYS_ENABLE_WAKEUP(SYS_WAKEUP_GPIOWE_Msk);

    /* Configure PB10 as EINT0 */
    GPIO_SetIntGroup(PB, 10, GPIO_INTSRCGP_EINT0);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Waiting for PB.10 falling-edge interrupt event */
    while(1) {
        printf("Enter to Power-Down ......\n");

        /* Enter to Power-down mode */
        PowerDownFunction();

        printf("System waken-up done.\n\n");
    }

}


