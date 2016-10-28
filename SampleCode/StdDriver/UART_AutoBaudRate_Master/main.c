/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 14/06/06 11:24a $
 * @brief    NUC505 Series UART Interface Controller Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

#define RXBUFSIZE 1024


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void AutoBaudRate_TestItem(void);
void AutoBaudRate_TxTest(void);


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

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 module */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for testing */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART auto baud rate sample master function */
    AutoBaudRate_TxTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test Menu                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TestItem()
{
    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test (Master)                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] baud rate 38400 bps                                   |\n");
    printf("| [2] baud rate 57600 bps                                   |\n");
    printf("| [3] baud rate 115200 bps                                  |\n");
    printf("|                                                           |\n");
    printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TxTest()
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PB.6)  <==>  UART1_RXD(PB.7)--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");

    do {
        AutoBaudRate_TestItem();
        u32Item = getchar();
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch(u32Item) {
        case '1':
            UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 38400);
            break;
        case '2':
            UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 57600);
            break;
        default:
            UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
            break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART_WRITE(UART1, 0x1);

    } while(u32Item != 27);

}
