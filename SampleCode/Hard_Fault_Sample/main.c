/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/07/21 1:14p $
 * @brief    HardFault project for NUC505 MCU
 *
 *           Accessing the memory space is not supported in NUC505 to generate bus fault exception. 
 *           If bus fault  exception is not enabled. Hard fault exception will take care of it.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "NUC505Series.h"

#include "uart.h"

#define RXBUFSIZE 1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[12] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);

void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
		CLK_SetCoreClock(64000000);
    /* Update System Core Clock */
    SystemCoreClockUpdate();
		/* Set PCLK Divider = 1 (/2) */
		CLK_SetModuleClock(PCLK_MODULE, 0, 1);			
}


void UART0_Init(void)
{
		/* Enable UART0 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
		/* UART0 module clock from EXT */
		CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);
    /* Reset IP */
    SYS_ResetModule(UART0_RST);    
    /* Configure UART0 and set UART0 Baudrate */
		UART_Open(UART0, 115200);
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

int main(void)
{
		uint32_t *tmp = (uint32_t*)0x10000000;

		/* Init UART0 for printf */
    UART0_Init();
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();


    strcpy((char*)tmp,"HardFaultTest");

    while(1);

}




