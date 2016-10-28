/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/06/06 11:24a $
 * @brief    NUC505 Series UART Interface Controller Driver Sample Code
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"


#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void AutoFlow_FunctionRxTest(void);


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

    /* UART auto flow sample slave function */
    AutoFlow_FunctionRxTest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    volatile uint32_t u32IntSts = UART1->INTSTS;;

    /* Rx Ready or Time-out INT */
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk) ||  UART_GET_INT_FLAG(UART1, UART_INTSTS_RXTOINT_Msk)) {
        /* Handle received data */
        g_u8RecData[g_i32pointer] = UART_READ(UART1);
        g_i32pointer++;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionRxTest()
{
    uint32_t u32i;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PB.6)  <==>  UART1_RXD(PB.7)--|Slave| |\n");
    printf("| |      |--UART1_nCTS(PB.8) <==> UART1_nRTS(PB.9)--|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test (Slave)                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave. Slave will check if received data is correct |\n");
    printf("|    after getting 1k bytes data.                           |\n");
    printf("|    Press any key to start...                              |\n");
    printf("+-----------------------------------------------------------+\n");
    GetChar();

    /* Enable RTS and CTS autoflow control */
    UART_EnableFlowCtrl(UART1);

    /* Set RTS Trigger Level as 8 bytes */
    UART1->FIFO &= ~UART_FIFO_RTSTRGLV_Msk;
    UART1->FIFO |= UART_FIFO_RTSTRGLV_8BYTES;

    /* Set RX Trigger Level as 8 bytes */
    UART1->FIFO &= ~UART_FIFO_RFITL_Msk;
    UART1->FIFO |= UART_FIFO_RFITL_8BYTES;

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    UART_SetTimeoutCnt(UART1, 0x3E);

    /* Enable RDA\RLS\RTO Interrupt  */
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while(g_i32pointer < RXBUFSIZE);

    /* Compare Data */
    for(u32i = 0; u32i < RXBUFSIZE; u32i++) {
        if(g_u8RecData[u32i] != (u32i & 0xFF)) {
            printf("Compare Data Failed\n");
            while(1);
        }
    }
    printf("\n Receive OK & Check OK\n");

    /* Disable RDA\RLS\RTO Interrupt */
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

}
