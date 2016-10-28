/**************************************************************************//**
 * @file     main.c
 * @version  V2.0
 * $Revision: 8 $
 * $Date: 15/03/04 9:48a $
 * @brief    Demonstrate SPI master loop back transfer. This sample code needs to connect the SPI0_MISO0 pin and the SPI0_MOSI0 pin together. It will compare the received data with transmitted data.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"

#define TEST_COUNT             64

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void SPI0_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount, u32TestCount, u32Err;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    
		/* Init UART0 to 115200-8n1 for print message */
    UART0_Init();
		
		/* Init SPI0, IP clock and multi-function I/O */
		SPI0_Init();
		
    printf("\n\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                   NUC505 SPI Driver Sample Code                  |\n");
    printf("+------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates SPI0 self loop back data transfer.\n");
    printf(" SPI0 configuration:\n");
    printf("     Master mode; data width 32 bits.\n");
    printf(" I/O connection:\n");
    printf("     PB.4 SPI0_MOSI <--> PB.5 SPI0_MISO \n");

    printf("\nSPI0 Loopback test ");

    /* set the source data and clear the destination buffer */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    u32Err = 0;
    for(u32TestCount = 0; u32TestCount < 0x1000; u32TestCount++)
    {
        /* set the source data and clear the destination buffer */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            g_au32SourceData[u32DataCount]++;
            g_au32DestinationData[u32DataCount] = 0;
        }

        u32DataCount = 0;

        if((u32TestCount & 0x1FF) == 0)
        {
            putchar('.');
        }
				
				SPI_ENABLE(SPI0);
				
        while(1)
        {
            /* Write to TX register */
            SPI_WRITE_TX(SPI0, g_au32SourceData[u32DataCount]);
            /* Check SPI0 busy status */
            while(SPI_IS_BUSY(SPI0));
            /* Read received data */
            g_au32DestinationData[u32DataCount] = SPI_READ_RX(SPI0);
            u32DataCount++;
            if(u32DataCount == TEST_COUNT)
                break;
        }

        /*  Check the received data */
        for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
        {
            if(g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if(u32Err)
            break;
    }

    if(u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Close SPI0 */
    SPI_Close(SPI0);

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

void SPI0_Init(void)
{
		/* Enable SPI0 Module clock */
    CLK_EnableModuleClock(SPI0_MODULE);
		/* SPI0 module clock from EXT */
		CLK_SetModuleClock(SPI0_MODULE, CLK_SPI0_SRC_PLL, 0);
    /* Reset IP */
    SYS_ResetModule(SPI0_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for SPI0 */
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB2MFP_Msk) ) | SYS_GPB_MFPL_PB2MFP_SPI0_SS;	
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB3MFP_Msk) ) | SYS_GPB_MFPL_PB3MFP_SPI0_CLK;	
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk) ) | SYS_GPB_MFPL_PB4MFP_SPI0_MOSI;	
		SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk) ) | SYS_GPB_MFPL_PB5MFP_SPI0_MISO;	

		/*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/

