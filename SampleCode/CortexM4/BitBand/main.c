/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 14/10/06 9:44a $
 * @brief    Demonstrate the usage of Cortex?-M4 BitBand.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* Memory Address */
#define MEM_ADDR(address)       *((volatile unsigned long *) (address))
/* BitBand Address */
#define BITBAND(address,bitnum) ((address & 0xF0000000)+0x02000000+((address & 0xFFFFF)<<5)+(bitnum<<2))

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(96000000);
    /* Update System Core Clock */
    SystemCoreClockUpdate();

		// Set APB clock as 1/2 HCLK
		CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
	
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

void BitBand_Test(void)
{
    uint32_t u32Temp;
    uint32_t u32TestAddress;

    u32TestAddress = 0x20003C00;

    //Read SRAM Address(0x20003F00)
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at test address (0x20003F00) is: 0x%x", u32Temp);

    //Use BitBand function to read SRAM bit value
    u32Temp = MEM_ADDR(BITBAND(u32TestAddress, 3));
    printf("\n Use Bit-Band function to get value at bit 3: %x \n", u32Temp);

    //Use BitBand function set bit
    printf("\n Use Bit-Band function set test address (0x20003F00) bit 3 ");
    MEM_ADDR(BITBAND(u32TestAddress, 3)) = 1;
    //Read Test Address Value
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at address 0x20003F00 is: 0x%x \n", u32Temp);

    //Use BitBand function clear bit
    printf("\n Use Bit-Band function clear test address (0x20003F00) bit 3 ");
    MEM_ADDR(BITBAND(u32TestAddress, 3)) = 0;
    //Read Test Address Value
    u32Temp = MEM_ADDR(u32TestAddress);
    printf("\n The value at address 0x20003F00 is: 0x%x \n", u32Temp);
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n Start Bit-Band test: \n");

    BitBand_Test();

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
