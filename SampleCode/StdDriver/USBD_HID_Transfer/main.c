/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Date: 14/05/30 6:07p $
 * @brief    Simulate an USB mouse and draws circle on the screen
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "hid_vendor.h"

#if defined (__ICCARM__)
void USBD_IRQHandler_SRAM(void);
#endif

/* Buffer for the Data of Command Read / Write */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t u8BufferBulk[BUFFER_SIZE];
#else
__align(4) uint8_t u8BufferBulk[BUFFER_SIZE];
#endif

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

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

    /* Enable USB IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select USB IP clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_USBD_SRC_EXT, 0);
		
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
void Erase(uint32_t u32StartSector, uint32_t u32Sectors)
{
		uint8_t *Buffer = (uint8_t *)((uint32_t)u8BufferBulk + u32StartSector * SECTOR_SIZE);
		uint32_t u32Size = u32Sectors * SECTOR_SIZE;
	
		if((u32StartSector + u32Sectors) * SECTOR_SIZE > BUFFER_SIZE)
				u32Size = BUFFER_SIZE - u32StartSector * SECTOR_SIZE;
		memset(Buffer, 0xFF, u32Size);
}

void PrepareReadData(uint32_t *pu32Address, uint32_t u32StartPage, uint32_t u32Pages)
{	
    if(u32Pages)	/* Prepare the Data for Read Command */
    {
				/* It's the buffer address of StartPage */
        *pu32Address = (uint32_t)u8BufferBulk + (u32StartPage * PAGE_SIZE);
    }
}

void PrepareWriteBuffer(uint32_t *pu32Address, uint32_t u32StartPage, uint32_t u32Pages)
{
    if(u32Pages)
    {         
				/* It's the buffer address of StartPage */
        *pu32Address = (uint32_t)u8BufferBulk + ((u32StartPage * PAGE_SIZE) % BUFFER_SIZE);
    }

}
void GetDatatForWrite(uint32_t u32Address, uint32_t u32StartPage, uint32_t u32Pages)
{
    if(u32Pages)
    {     
				uint32_t i,j;
				uint8_t *pu32Address = (uint8_t *)u32Address;	/* It's the buffer address of StartPage */
				for(j=0;j<u32Pages;j++)
				{
						for(i=0;i<PAGE_SIZE;i++)
						{
								if(pu32Address[j * PAGE_SIZE + i] != i % 256)
								{
											printf("index %d - %02x != %02x\n", j * PAGE_SIZE + i, pu32Address[j * PAGE_SIZE + i], i % 256);
											return;
								}
						}	        
				}       
    }
}

	
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

#if defined (__ICCARM__)
        #pragma section = "VECTOR2"              
        
				extern uint32_t __Vectors[];
				extern uint32_t __Vectors_Size[];	
				uint32_t* pu32Src;    
				uint32_t* pu32Dst;
        
				pu32Src = (uint32_t *)&USBD_IRQHandler_SRAM;        
				//printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", __section_begin("VECTOR2"));
				memcpy((void *) __section_begin("VECTOR2"), (void *) __Vectors, (unsigned int) __Vectors_Size);
				SCB->VTOR = (uint32_t) __section_begin("VECTOR2");
					
				/* Change USBD vector to interrupt handler in SRAM */
				/* IAR compiler doesn't following initial configuration file to relocate USBD IRQHandler() */
				pu32Dst = (uint32_t*) ((uint32_t)__section_begin("VECTOR2")+0x64);
				*pu32Dst = (uint32_t)pu32Src;
        
#endif    
    printf("\nNUC505 USB HID\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();

    /* Enable USBD interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    /* Start transaction */
    USBD_Start();

    while(1);
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

