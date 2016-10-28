/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 4 $
 * $Date: 15/03/04 9:49a $
 * @brief    Demonstrate how to communicate with an off-chip SPI master device. This sample code needs to work with SPI_MasterMode.
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
    volatile uint32_t u32TxDataCount, u32RxDataCount;
    uint32_t u32Tmp;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    
		/* Init UART0 to 115200-8n1 for print message */
    UART0_Init();
		
		/* Init SPI1, IP clock and multi-function I/O */
		SPI1_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           SPI Slave Mode Sample Code                |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI1 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI1:\n");
    printf("    SPI1_SS (PB.10)\n    SPI1_CLK (PB.11)\n");
    printf("    SPI1_MOSI (PB.12)\n    SPI1_MISO (PB.13)\n\n");
    printf("SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        u32Tmp = u32TxDataCount;
        g_au32SourceData[u32TxDataCount] = 0x00AA0000 + u32Tmp;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
		SPI_ENABLE(SPI1);
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX FIFO */
    while(u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (u32TxDataCount < TEST_COUNT))
            SPI_WRITE_TX(SPI1, g_au32SourceData[u32TxDataCount++]); /* Write to TX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0)
        {
            u32Tmp = u32RxDataCount;
            g_au32DestinationData[u32Tmp++] = SPI_READ_RX(SPI1); /* Read RX FIFO */
            u32RxDataCount = u32Tmp;
        }
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        u32Tmp = u32RxDataCount;
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32Tmp]);
    }
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI1 peripheral clock */
    SPI_Close(SPI1);
    while(1);
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
    /* Configure SPI1 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI_Open(SPI1, SPI_SLAVE, SPI_MODE_0, 32, NULL);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


