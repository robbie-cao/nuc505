/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * $Revision: 7 $
 * $Date: 14/07/31 2:52p $
 * @brief    Demonstrate how I2S works in Master mode. This sample code needs to work with I2S_Slave.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"

uint32_t g_u32TxValue;
uint32_t g_u32DataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void I2S_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32RxValue1, u32RxValue2;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

		/* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

		/* Init I2S, IP clock and multi-function I/O */
		I2S_Init();

    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (master mode)           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x55005501, 0x55025503, ..., 0x55FE55FF, wraparound\n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC11)\n      I2S_BCLK(PC12)\n");
    printf("      I2S_DI (PC9)\n      I2S_DO (PC10)\n\n");
    printf("  NOTE: Connect with a I2S slave device.\n");
    printf("        This sample code will transmit a TX value 50000 times, and then change to the next TX value.\n");
    printf("        When TX value or the received value changes, the new TX value or the current TX value and the new received value will be printed.\n");

    /* Master mode, 16-bit word width, stereo mode, I2S format. Set TX and RX FIFO threshold to middle value. */
    I2S_Open(I2S, I2S_MODE_MASTER, 16000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S, 0);
		I2S_ENABLE_TX(I2S);
		I2S_ENABLE_RX(I2S);

    /* Initiate data counter */
    g_u32DataCount = 0;
    /* Initiate TX value and RX value */
    g_u32TxValue = 0x55005501;
    u32RxValue1 = 0;
    u32RxValue2 = 0;
    /* Enable TX threshold level interrupt */
    I2S_EnableInt(I2S, I2S_IEN_TXTHIEN_Msk);
    NVIC_EnableIRQ(I2S_IRQn);

    printf("Start I2S ...\nTX value: 0x%X\n", g_u32TxValue);

    while(1)
    {
        /* Check RX FIFO empty flag */
        if((I2S->STATUS & I2S_STATUS_RXEMPTY_Msk) == 0)
        {
            /* Read RX FIFO */
            u32RxValue2 = I2S_READ_RX_FIFO(I2S);
            if(u32RxValue1 != u32RxValue2)
            {
                u32RxValue1 = u32RxValue2;
                /* If received value changes, print the current TX value and the new received value. */
                printf("TX value: 0x%X;  RX value: 0x%X\n", g_u32TxValue, u32RxValue1);
            }
        }
        if(g_u32DataCount >= 50000)
        {
            g_u32TxValue = 0x55005500 | ((g_u32TxValue + 0x00020002) & 0x00FF00FF); /* g_u32TxValue: 0x55005501, 0x55025503, ..., 0x55FE55FF */
            printf("TX value: 0x%X\n", g_u32TxValue);
            g_u32DataCount = 0;
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

void I2S_Init(void)
{
		/* Enable I2S Module clock */
    CLK_EnableModuleClock(I2S_MODULE);
		/* I2S module clock from APLL */
		CLK_SET_APLL(CLK_APLL_49152031);
		CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 3);
    /* Reset IP */
    SYS_ResetModule(I2S_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for I2S */
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk) ) | SYS_GPC_MFPH_PC8MFP_I2S_MCLK;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk) ) | SYS_GPC_MFPH_PC9MFP_I2S_DIN;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk) ) | SYS_GPC_MFPH_PC10MFP_I2S_DOUT;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk) ) | SYS_GPC_MFPH_PC11MFP_I2S_LRCLK;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk) ) | SYS_GPC_MFPH_PC12MFP_I2S_BCLK;	
	
}

void I2S_IRQHandler()
{
    /* Write 2 TX values to TX FIFO */
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    I2S_WRITE_TX_FIFO(I2S, g_u32TxValue);
    g_u32DataCount += 2;
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
