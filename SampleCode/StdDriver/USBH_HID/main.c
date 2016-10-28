/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 14/05/30 6:08p $
 * @brief    Use USB Host core driver and HID driver. This sample code reads raw data 
 *           from a USB mouse.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "usbh_core.h"
#include "usbh_hid.h"


/*
 *  The following path definitions are depended on device.
 */
#define   PATHLEN           3
int       PATH_IN[PATHLEN] = { 0x00010002, 0x00010001, 0x00010031 };
#define   RECV_PACKET_LEN       4
char      packet[RECV_PACKET_LEN];


uint8_t  desc_buff[1024];

void Delay(uint32_t delayCnt)
{
    while(delayCnt--) {
        __NOP();
        __NOP();
    }
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

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

		/* USB Host Clock Source MUST be multiple of 48MHz */
    CLK_SetCoreClock(96000000);
	
    /* Set PCLK divider */
    CLK_SetModuleClock(PCLK_MODULE, NULL, 1);
	
    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

	  /* Enable USB Host IP clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(USBH_MODULE, CLK_USBH_SRC_PLL, CLK_GetPLLClockFreq() / 48000000 - 1);	

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


void  int_read_callback(HID_DEV_T *hdev, uint8_t *rdata, int data_len)
{
    int  i;
    printf("INT-in pipe data %d bytes received =>\n", data_len);
    for (i = 0; i < data_len; i++)
        printf("0x%02x ", rdata[i]);
    printf("\n");
}

static uint8_t  _write_data_buff[4];

void  int_write_callback(HID_DEV_T *hdev, uint8_t **wbuff, int *buff_size)
{
    printf("INT-out pipe request to write data.\n");

    *wbuff = &_write_data_buff[0];
    *buff_size = 4;
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    int          i, ret;
    HID_DEV_T    *hdev;
		char Item ;
		uint32_t u32UsbhPort1 = HOST_LIKE_PORT1_DISABLE;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART0 for printf */
    UART0_Init();

    printf("+--------------------------------------+\n");
    printf("|                                      |\n");
    printf("|     USB Host HID sample program      |\n");
    printf("|                                      |\n");
    printf("+--------------------------------------+\n");
	
	
		do
		{
				printf("============================================================================================\n");
				printf("Please select the USB host port 1 through GPIO\n");
				printf("[A] GPB12 and GPB13\n");
				printf("[B] GPB14 and GPB15\n");
				printf("[C] Disable\n");
				printf("============================================================================================\n");

				scanf("%c",&Item);
				switch(Item)
				{
						case 'A': 	
                                                case 'a': 
											u32UsbhPort1 = HOST_LIKE_PORT1_0;	
											goto next;
						case 'B': 		
                                                case 'b': 
											u32UsbhPort1 = HOST_LIKE_PORT1_1;	
											goto next;
						case 'C': 
                                                case 'c': 
											goto next;	    		
    				
				}
		}while(1);	
next:
		do
		{
				printf("============================================================================================\n");
				printf("Please select the USB host port 2 through GPIO\n");
				printf("[A] GPC13 and GPC14\n");
				printf("[B] Disable\n");
				printf("============================================================================================\n");
				scanf("%c",&Item);
				switch(Item)
				{
						case 'A': 	
                                                case 'a': 	
										USB_PortInit(u32UsbhPort1, HOST_LIKE_PORT2_0);							
										goto start;
						case 'B': 		
                                                case 'b':   
										USB_PortInit(u32UsbhPort1, HOST_LIKE_PORT2_DISABLE);	
										goto start;		
				}
		}while(1);			
				
start:	

    USBH_Open();

    USBH_HidInit();

    Delay(0x1000);
    printf("Wait until any HID devices connected...\n");
    while (1) {
        USBH_ProcessHubEvents();             /* USB Host port detect polling and management */

        hdev = USBH_HidGetDeviceList();
        if (hdev != NULL)
            break;
    }

    ret = HID_HidGetReportDescriptor(hdev, desc_buff, 1024);
    if (ret > 0) {
        printf("\nDump report descriptor =>\n");
        for (i = 0; i < ret; i++) {
            if ((i % 16) == 0)
                printf("\n");
            printf("%02x ", desc_buff[i]);
        }
        printf("\n\n");
    }

    /*
     *  Example: GET_PROTOCOL request.
     */
    ret = HID_HidGetProtocol(hdev, desc_buff);
    printf("[GET_PROTOCOL] ret = %d, protocol = %d\n", ret, desc_buff[0]);

    /*
     *  Example: SET_PROTOCOL request.
     */
    ret = HID_HidSetProtocol(hdev, desc_buff[0]);
    printf("[SET_PROTOCOL] ret = %d, protocol = %d\n", ret, desc_buff[0]);

    /*
     *  Example: GET_REPORT request on report ID 0x1, report type FEATURE.
     */
    ret = HID_HidGetReport(hdev, RT_FEATURE, 0x1, desc_buff, 64);
    if (ret > 0) {
        printf("[GET_REPORT] Data => ");
        for (i = 0; i < ret; i++)
            printf("%02x ", desc_buff[i]);
        printf("\n");
    }

    printf("\nUSBH_HidStartIntReadPipe...\n");
    if (USBH_HidStartIntReadPipe(hdev, int_read_callback) == HID_RET_OK) {
        printf("Interrupt in transfer started...\n");
    }

    //if (USBH_HidStartIntWritePipe(hdev, int_write_callback) == HID_RET_OK)
    //{
    //  printf("Interrupt out transfer started...\n");
    //}

    printf("Done.\n");

    while (1);
}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
