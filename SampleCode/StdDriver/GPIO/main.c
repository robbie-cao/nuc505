/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 14/05/29 1:14p $
 * @brief    NUC505 General Purpose I/O Driver Sample Code
 *           Connect PB.10 and PB.11 to test IO In/Out
 *           Test PB.10 and PB.11 interrupts
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"
#include "gpio.h"

void EINT0_IRQHandler(void)
{
    int32_t u32Flag;

    u32Flag = GPIO_GET_INT_FLAG(PB, BIT11);
    GPIO->INTSTSA_B = (u32Flag << 16);

    /* To avoid the synchronization issue between system and APB clock domain */
    u32Flag = GPIO->INTSTSA_B;

    printf("  EINT0 interrupt occurred. \n");
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
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-------------------------------------+ \n");
    printf("|           GPIO Driver Sample Code   | \n");
    printf("+-------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.10 and PB.11 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PB.10 as Output mode and PB.11 as Input mode pull up enable then close it */
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);
    GPIO_SetPullMode(PB, BIT11, GPIO_PULL_UP_EN);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    PB10_DOUT = 0;
    if (PB11_PIN != 0) {
        i32Err = 1;
    }

    PB10_DOUT = 1;
    if (PB11_PIN != 1) {
        i32Err = 1;
    }

    if ( i32Err ) {
        printf("  [FAIL] --- Please make sure PB.10 and PB.11 are connected. \n");
    } else {
        printf("  [OK] \n");
    }

    /* Configure PB.10 and PB.11 to default input mode */
    GPIO_SetMode(PB, BIT10, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  PB10 and PB11 are used to test interrupt\n");

    /* Configure PB.10 as Output mode */
    PB10_DOUT = 0;
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);

    /* Configure PB11 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT11, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 11, GPIO_INT_RISING);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(NULL, GPIO_DBCTL_DBCLKSEL_128);
    GPIO_ENABLE_DEBOUNCE(PB, GPIO_DBCTL_EINT0_DBEN_MASK);

    /* Configure PB11 as EINT0 */
    GPIO_SetIntGroup(PB, 11, GPIO_INTSRCGP_EINT0);
    NVIC_EnableIRQ(EINT0_IRQn);

    PB10_DOUT = 1;

    /* Waiting for interrupts */
    while (1);

}


