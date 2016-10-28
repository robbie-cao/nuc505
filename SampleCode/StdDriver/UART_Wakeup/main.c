/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 14/06/06 11:24a $
 * @brief    NUC505 Series UART Driver Sample Code
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "stdio.h"
#include "NUC505Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);


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
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    CLK_SetModuleClock(UART1_MODULE, CLK_UART1_SRC_EXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

    /* Configure multi-function pins for UART1 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk) ) | SYS_GPB_MFPL_PB6MFP_UART1_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk) ) | SYS_GPB_MFPL_PB7MFP_UART1_RXD;

    /* Configure multi-function pins for UART1 RTS and CTS */
    SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk) ) | SYS_GPB_MFPH_PB8MFP_UART1_nCTS;
    SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk) ) | SYS_GPB_MFPH_PB9MFP_UART1_nRTS;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 baud rate */
    UART_Open(UART1, 110);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    SYS_ENABLE_WAKEUP(1<<SYS_WAKEUP_UART1WE_Pos);

    /* UART Power-down and Wake-up sample function */
    UART_PowerDownWakeUpTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_WKIF_Msk) {              /* UART wake-up interrupt flag */
        if(u32IntSts & UART_INTSTS_DATWKIF_Msk) {       /* UART data wake-up interrupt flag */
            printf("UART data wake-up interrupt happen.\n");
            UART1->INTSTS |= UART_INTSTS_DATWKIF_Msk;   /* Clear UART data wake-up interrupt flag */
        }

        if(u32IntSts & UART_INTSTS_CTSWKIF_Msk) {       /* UART CTS wake-up interrupt flag */
            printf("UART nCTS wake-up interrupt happen.\n");
            UART1->INTSTS |= UART_INTSTS_DATWKIF_Msk;   /* Clear UART CTS wake-up interrupt flag */
        }
    }

    /* To avoid the synchronization issue between system and APB clock domain */
    u32IntSts = UART1->INTSTS;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{

    /* Enable UART data wake-up interrupt */
    UART_EnableInt(UART1, UART_INTEN_WKDATIEN_Msk);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 110bps to UART1 to wake-up system.\n");

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Wait to receive wake-up data */
    while(!UART_IS_RX_READY(UART1));
    printf("The first wake-up data is 0x%x.\n", UART1->DAT);

    /* Disable UART Data Wake-up Interrupt */
    UART_DisableInt(UART1, UART_INTEN_WKDATIEN_Msk);
    NVIC_DisableIRQ(UART1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{

    /* Clear MODEM interrupt before CTS wake-up interrupt */
    UART_ClearIntFlag(UART1, UART_INTSTS_MODEMINT_Msk);

    /* Enable UART CTS wake-up interrupt */
    UART_EnableInt(UART1, UART_INTEN_WKCTSIEN_Msk);

    printf("System enter to Power-down mode.\n");
    printf("Toggle nCTS of UART1 to wake-up system.\n");

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Clear MODEM interrupt after CTS wake-up interrupt */
    UART_ClearIntFlag(UART1, UART_INTSTS_MODEMINT_Msk);

    /* Disable UART CTS Wake-up Interrupt */
    UART_DisableInt(UART1, UART_INTEN_WKCTSIEN_Msk);
    NVIC_DisableIRQ(UART1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Data wake-up test                                     |\n");
    printf("| [2] nCTS wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item) {
    case '1':
        UART_DataWakeUp();
        break;
    case '2':
        UART_CTSWakeUp();
        break;
    default:
        break;
    }

    printf("\nUART Sample Program End.\n");
}
