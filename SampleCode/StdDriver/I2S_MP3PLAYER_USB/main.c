/**************************************************************************//**
 * @file     main.c
 * @version  V2.1 
 * $Revision: 8 $
 * $Date: 16/01/09 3:40p $
 * @brief    A MP3 file player demo using internal audio codec used to playback MP3 file stored in USB pen drive.
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"

#include "usbh_core.h"
#include "usbh_umas.h"

uint32_t volatile u32BuffPos = 0;

FATFS FatFs[_VOLUMES];               /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[1024] ;                   /* Working buffer */
#endif

#ifdef __ARMCC_VERSION
__align(32) BYTE Buff[1024] ;       /* Working buffer */
#endif

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime (void)
{
    unsigned long tmr;

    tmr=0x00000;

    return tmr;
}

void SYS_Init(void)
{

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    //SYS_UnlockReg();
     
    /* Enable  XTAL */
//    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(FREQ_96MHZ);
    
		/* PCLK divider */
		CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
		
    /* Lock protected registers */
    //SYS_LockReg();

		// Enable USB Host
    CLK->AHBCLK |= CLK_AHBCLK_USBHCKEN_Msk;
		
		// USB clock is HCLK divided by 2
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBHDIV_Msk) | CLK_CLKDIV0_USBHSEL_Msk |  (0x01 << CLK_CLKDIV0_USBHDIV_Pos);
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
	//FIXME APLL CLOCK
		// ideal clock is 49.152MHz, real clock is 49152031Hz
//	CLK_SET_APLL(CLK_APLL_49152031);	// APLL is 49152031Hz for 48000Hz
	CLK_SET_APLL(CLK_APLL_45158425);	// APLL is 45158425Hz for 44100Hz
	CLK_SetModuleClock(I2S_MODULE, CLK_I2S_SRC_APLL, 0);	// 0 means (APLL/1)
    /* Reset IP */
    SYS_ResetModule(I2S_RST);    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for I2S */
		// GPC[8]  = MCLK
		// GPC[9]  = DIN
		// GPC[10] = DOUT
		// GPC[11] = LRCLK
		// GPC[12] = BCLK
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC8MFP_Msk) ) | SYS_GPC_MFPH_PC8MFP_I2S_MCLK;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC9MFP_Msk) ) | SYS_GPC_MFPH_PC9MFP_I2S_DIN;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC10MFP_Msk) ) | SYS_GPC_MFPH_PC10MFP_I2S_DOUT;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC11MFP_Msk) ) | SYS_GPC_MFPH_PC11MFP_I2S_LRCLK;	
		SYS->GPC_MFPH  = (SYS->GPC_MFPH & (~SYS_GPC_MFPH_PC12MFP_Msk) ) | SYS_GPC_MFPH_PC12MFP_I2S_BCLK;	
	
}

void Delay(int count)
{
    volatile uint32_t i;
    for (i = 0; i < count ; i++);
}

#define HOST_LIKE_PORT1_0						0x10
#define HOST_LIKE_PORT1_1						0x20	
#define HOST_LIKE_PORT2_0						0x00
#define HOST_LIKE_PORT1_DISABLE			0xFF
#define HOST_LIKE_PORT2_DISABLE			0xFF

void USB_PortInit(uint32_t u32Port1, uint32_t u32Port2)
{
		SYS->WAKEUP = SYS->WAKEUP | SYS_WAKEUP_USBHWF_Msk;
		switch(u32Port1) 
		{//port 1
				case HOST_LIKE_PORT1_DISABLE:
						printf("USB host like port 1 Disable\n");
						break;
				case HOST_LIKE_PORT1_0:		
						SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk|SYS_GPB_MFPH_PB13MFP_Msk)) | (0x2 << SYS_GPB_MFPH_PB12MFP_Pos) | (0x2 << SYS_GPB_MFPH_PB13MFP_Pos);														
						printf("USB host like port 1 from GPB12 & GPB13\n");
						break;				
				case HOST_LIKE_PORT1_1:								
						SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk|SYS_GPB_MFPH_PB15MFP_Msk)) | (0x1 << SYS_GPB_MFPH_PB14MFP_Pos) | (0x1 << SYS_GPB_MFPH_PB15MFP_Pos);																
						printf("USB host like port 1 from GPB14 & GPB15\n");
						break;	
		}	
		switch(u32Port2)	
		{//port 2						
				case HOST_LIKE_PORT2_DISABLE:
						printf("USB host like port 2 Disable\n");
						break;		
				case HOST_LIKE_PORT2_0:			
						SYS->GPC_MFPH = (SYS->GPC_MFPH & ~(SYS_GPC_MFPH_PC13MFP_Msk|SYS_GPC_MFPH_PC14MFP_Msk)) | (0x1 << SYS_GPC_MFPH_PC13MFP_Pos) | (0x1 << SYS_GPC_MFPH_PC14MFP_Pos);				
						printf("USB host like port 2 from GPC13 & GPC14\n");
						break;
		}		
		USBH->HcMiscControl = (USBH->HcMiscControl & ~(USBH_HcMiscControl_DPRT1_Msk | USBH_HcMiscControl_DPRT2_Msk)) | ((u32Port1 & 0x01) << USBH_HcMiscControl_DPRT1_Pos) | ((u32Port2 & 0x01) << USBH_HcMiscControl_DPRT2_Pos);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
		uint32_t u32Item = 0;

		uint32_t u32UsbhPort1 = HOST_LIKE_PORT1_DISABLE;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   MP3 Player Sample with Internal CODEC                |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf(" Please put MP3 files on USB pen-drive\n");
		printf(" Please select the USB host port 0 through GPIO\n");
		printf(" GPC13 and GPC14\n");

		printf("\n\n");
    printf("+-----------------------------------------------+\n");
    printf("|                                               |\n");
    printf("|     USB Host Mass Storage sample program      |\n");
    printf("|                                               |\n");
    printf("+-----------------------------------------------+\n");
		do
		{
				printf("============================================================================================\n");
				printf("Please select the USB host port 1 through GPIO\n");
				printf("[1] GPB12 and GPB13\n");
				printf("[2] GPB14 and GPB15\n");
				printf("[3] Disable\n");
				printf("============================================================================================\n");
///				scanf("%d",&u32Item);
				u32Item = 3;
				switch(u32Item)
				{
						case 1: 		
											u32UsbhPort1 = HOST_LIKE_PORT1_0;	
											break;
						case 2: 		
											u32UsbhPort1 = HOST_LIKE_PORT1_1;	
											break;
						case 3: 
											break;		    		
    				
				}
		}while((u32Item> 3));	

		do
		{
				printf("============================================================================================\n");
				printf("Please select the USB host port 0 through GPIO\n");
				printf("[1] GPC13 and GPC14\n");
				printf("[2] Disable\n");
				printf("============================================================================================\n");
///				scanf("%d",&u32Item);
				u32Item = 1;
				switch(u32Item)
				{
						case 1: 	
										USB_PortInit(u32UsbhPort1, HOST_LIKE_PORT2_0);							
										goto start;
						case 2:   
										USB_PortInit(u32UsbhPort1, HOST_LIKE_PORT2_DISABLE);	
										goto start;		
				}
		}while((u32Item > 2));			
				
start:

    /* Configure USB Host */
    USBH_Open();

    USBH_MassInit();

//    Delay(0x500000);
		Delay(0x50000);
    USBH_ProcessHubEvents();

    printf("rc=%d\n", (WORD)disk_initialize(0));
    disk_read(0, Buff, 2, 1);
    //f_mount(0, &FatFs[0]);  // for FATFS v0.09
		// Register work area to the default drive
    f_mount(&FatFs[0], "", 0);  // for FATFS v0.11

	/* Init I2S, IP clock and multi-function I/O */
	I2S_Init();

    MP3Player();

    while(1);
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
