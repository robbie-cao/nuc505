/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 14/10/06 9:45a $
 * @brief    Demonstrate the usage of Cortex?-M4 MPU.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

#define Region_Size_1K     0x9
#define Region_Size_16K    0xD
#define Region_Size_32K    0xE
#define Region_Size_64K    0xF
#define Region_Size_128K   0x10
#define Region_Size_512K   0x12

/* MPU Attribute Register: Access Permission Definition */
#define AP_No_Access       0x0
#define AP_Pri_RW_User_NO  0x1
#define AP_Pri_RW_User_RO  0x2
#define AP_Pri_RW_User_RW  0x3
#define AP_Pri_RO_User_NO  0x5
#define AP_Pri_RO_User_RO  0x6

/* MPU Attribute Register: Region Enable Bit */
#define MPU_ATTR_EN        1
#if defined ( __CC_ARM )//Keil environment
#pragma O0
uint32_t ReadMemCore(uint32_t address)
{
    __IO uint32_t val = 0;
    uint32_t *a = (uint32_t*) address;
    val = *a;

    return val;
}
#pragma O2
#elif defined (__ICCARM__)//IAR environment	
uint32_t ReadMemCore(uint32_t address)
{
    __IO uint32_t val = 0;
    uint32_t *a = (uint32_t*) address;
    val = *a;

    return val;
}
#endif

void MemManage_Handler(void)
{
    // Disable MPU to allow simple return from mem_manage handler
    // Mem_manage typically indicates code failure, and would
    // be resolved by reset or terminating faulty thread in OS.
    MPU->CTRL = 0x0;

    // Clear Fault status register
    SCB->CFSR = 0x000000BB;

    printf("\n Memory Fault !!");
}
void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(48000000);
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

void MPU_Test(void)
{
    uint8_t u8TestItem = 0;

    //------------------------------
    // Configure MPU memory regions
    //------------------------------

    // Region 1 (Flash Memory Space)
    // Start address = 0x0
    // Permission = Full access
    // Size = 128KB

    // Base address = Base address :OR: Region number :OR: VALID bit
    MPU->RBAR = ((0x00000000 & MPU_RBAR_ADDR_Msk) | (0x1 & MPU_RBAR_REGION_Msk) | MPU_RBAR_VALID_Msk);
    // Attribute = Full access :OR: SRD = 0 :OR: Size = 128KB :OR: ENABLE
    MPU->RASR = ((AP_Pri_RW_User_RW << MPU_RASR_AP_Pos)| ( Region_Size_128K << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk);

    // Region 2 (SRAM Memory Space)
    // Start address = 0x20000000
    // Permission = Full access
    // Size = 16KB

    // Base address = Base address :OR: Region number :OR: VALID bit
    MPU->RBAR = ((0x20000000 & MPU_RBAR_ADDR_Msk) | (0x2 & MPU_RBAR_REGION_Msk) | MPU_RBAR_VALID_Msk);
    // Attribute = Full access :OR: SRD = 0 :OR: Size = 16KB :OR: ENABLE
    MPU->RASR = ((AP_Pri_RW_User_RW << MPU_RASR_AP_Pos)| ( Region_Size_16K << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk);

    // Region 3 (Test Memory Space)
    // Start address = 0x20014000
    // Permission = Read Only
    // Size = 1KB

    // Base address = Base address :OR: Region number :OR: VALID bit
    MPU->RBAR = ((0x20014000 & MPU_RBAR_ADDR_Msk) | (0x3 & MPU_RBAR_REGION_Msk) | MPU_RBAR_VALID_Msk);
    // Attribute = Read Only :OR: SRD = 0 :OR: Size = 16KB :OR: ENABLE
    MPU->RASR = ((AP_No_Access << MPU_RASR_AP_Pos)| ( Region_Size_1K << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk);
		
    // Enable MemFault enable bit
    SCB->SHCSR = SCB->SHCSR | SCB_SHCSR_MEMFAULTENA_Msk;
    // Enable MPU
    MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

    printf("\n Please Press '1' to read memory from region 1 (Flash Memory)");

    while(u8TestItem != '1') u8TestItem = getchar();

    printf("\n Please Press '2' to read memory from region 2 (SRAM)");

    while(u8TestItem != '2') u8TestItem = getchar();
		
		/* MPU_CTRL_PRIVDEFENA_Msk bit enable 0 ~ 4GB accessable default */
    ReadMemCore(0x20010000);

    printf("\n Please Press '3' to read memory from region 3 (Test Memory)");

    while(u8TestItem != '3') u8TestItem = getchar();

    ReadMemCore(0x20014000);
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n Start MPU test: \n");

    MPU_Test();

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
