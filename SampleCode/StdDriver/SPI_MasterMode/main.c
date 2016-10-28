/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 4 $
 * $Date: 15/03/04 9:49a $
 * @brief    Demonstrate how to communicate with an off-chip SPI slave device. This sample code needs to work with SPI_SlaveMode.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"


#define TEST_COUNT 16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void SPI1_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    
		/* Init UART0 to 115200-8n1 for print message */
    UART0_Init();
		
		/* Init SPI1, IP clock and multi-function I/O */
		SPI1_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|             SPI Master Mode Sample Code                |\n");
    printf("+--------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI1 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI1:\n");
    printf("    SPI1_SS (PB.10)\n    SPI1_CLK (PB.11)\n");
    printf("    SPI1_MOSI (PB.12)\n    SPI1_MISO (PB.13)\n\n");
    printf("SPI controller will transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

		SPI_ENABLE(SPI1);
    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_SetFIFO(SPI1, 4, 4);
    SPI_EnableInt(SPI1, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(SPI1_IRQn);

    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_DisableInt(SPI1, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    NVIC_DisableIRQ(SPI1_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI1 peripheral clock */
    SPI_Close(SPI1);
    while(1);
}

void SPI1_IRQHandler(void)
{
    uint32_t u32Tmp;
    
    /* Check RX EMPTY flag */
    while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
    {
        u32Tmp = g_u32RxDataCount;
        /* Read RX FIFO */
        g_au32DestinationData[u32Tmp++] = SPI_READ_RX(SPI1);
        g_u32RxDataCount = u32Tmp;
    }
    /* Check TX FULL flag and TX data count */
    while((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI_WRITE_TX(SPI1, g_au32SourceData[g_u32TxDataCount++]);
    }
    if(g_u32TxDataCount >= TEST_COUNT)
        SPI_DisableInt(SPI1, SPI_FIFO_TXTH_INT_MASK); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI_GetIntFlag(SPI1, SPI_FIFO_RXTO_INT_MASK))
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
        {
            u32Tmp = g_u32RxDataCount;
            g_au32DestinationData[u32Tmp++] = SPI_READ_RX(SPI1);
            g_u32RxDataCount = u32Tmp;
        }
    }
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

    CLK_SetCoreClock(FREQ_96MHZ);
    
		/* PCLK divider */
		CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
		
    /* Lock protected registers */
    //SYS_LockReg();
        
}

void UART0_Init(void)
{
		/* Enable UART0 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
		/* UART0 module clock from EXT */
		CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    /* Reset IP */
    SYS_ResetModule(UART0_RST);    
    /* Configure UART0 and set UART0 Baud-rate */
		UART_Open(UART0, 115200);
		/*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;	
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;	
	
}

void SPI1_Init(void)
{
		/* Enable SPI1 Module clock */
    CLK_EnableModuleClock(SPI1_MODULE);
		/* SPI1 module clock from EXT */
		CLK_SetModuleClock(SPI1_MODULE, CLK_SPI1_SRC_PLL, 0);
    /* Reset IP */
    SYS_ResetModule(SPI1_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for SPI1 */
		SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB10MFP_Msk) ) | SYS_GPB_MFPH_PB10MFP_SPI1_SS;	
		SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk) ) | SYS_GPB_MFPH_PB11MFP_SPI1_CLK;	
		SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk) ) | SYS_GPB_MFPH_PB12MFP_SPI1_MOSI;	
		SYS->GPB_MFPH  = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk) ) | SYS_GPB_MFPH_PB13MFP_SPI1_MISO;	

		/*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI1 as a master, SPI clock rate 200 KHz,
       clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 32, 200000);
    /* Enable the automatic hardware slave selection function. Select the SPI1_SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI1, SPI_SS, SPI_SS_ACTIVE_LOW);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/

