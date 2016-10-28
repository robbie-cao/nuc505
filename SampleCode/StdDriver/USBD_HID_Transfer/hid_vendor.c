/**************************************************************************//**
 * @file     hid_mouse.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    NUC505 USBD driver Sample file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC505Series.h"
#include "hid_vendor.h"

volatile uint8_t g_hid_protocol = 0;
volatile uint8_t g_hid_report = 0;
volatile uint8_t g_hid_idle = 0;

/* Receive Command */
void EPB_Handler(void);
void MSC_BulkIn(uint32_t u32Addr, uint32_t u32Len);
void MSC_BulkOut(uint32_t u32Addr, uint32_t u32Len);

/* Buffer for Command Receive */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t u8BufferOUT[HID_MAX_PACKET_SIZE_INT_OUT];
#else
__align(4) uint8_t u8BufferOUT[HID_MAX_PACKET_SIZE_INT_OUT];
#endif


uint32_t g_u32TotalWritePage = 0;
CMD_T  gCmd;    
CMD_T  *pgCmd; 

uint32_t g_u32EpMaxPacketSize;

#if defined (__ICCARM__)
void USBD_IRQHandler_SRAM(void)
#else
void USBD_IRQHandler(void)
#endif
{
    __IO uint32_t IrqStL, IrqSt;

    IrqStL = USBD->GINTSTS & USBD->GINTEN;    /* get interrupt status */

    if (!IrqStL)    return;

    /* USB interrupt */
    if (IrqStL & USBD_GINTSTS_USBIF_Msk) {
        IrqSt = USBD->BUSINTSTS & USBD->BUSINTEN;

        if (IrqSt & USBD_BUSINTSTS_SOFIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SOFIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_RSTIF_Msk) {
            USBD_SwReset();

            USBD_ResetDMA();
            USBD->EP[EPA].EPRSPCTL = USBD_EPRSPCTL_FLUSH_Msk;
            USBD->EP[EPB].EPRSPCTL = USBD_EPRSPCTL_FLUSH_Msk;

            if (USBD->OPER & 0x04)  /* high speed */
                HID_InitForHighSpeed();
            else                    /* full speed */
                HID_InitForFullSpeed();
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_SET_ADDR(0);
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RSTIF_Msk);
            USBD_CLR_CEP_INT_FLAG(0x1ffc);
						
        }

        if (IrqSt & USBD_BUSINTSTS_RESUMEIF_Msk) {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_RESUMEIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_SUSPENDIF_Msk) {
            USBD_ENABLE_BUS_INT(USBD_BUSINTEN_RSTIEN_Msk | USBD_BUSINTEN_RESUMEIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_SUSPENDIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_HISPDIF_Msk) {
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_HISPDIF_Msk);
        }

        if (IrqSt & USBD_BUSINTSTS_DMADONEIF_Msk) {
            g_usbd_DmaDone = 1;
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_DMADONEIF_Msk);

            if (USBD->DMACTL & USBD_DMACTL_DMARD_Msk) {
                if (g_usbd_ShortPacket == 1) {
                    USBD->EP[EPA].EPRSPCTL = USBD->EP[EPA].EPRSPCTL & 0x10 | USB_EP_RSPCTL_SHORTTXEN;    // packet end
                    g_usbd_ShortPacket = 0;
                }
            }
        }

        if (IrqSt & USBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_PHYCLKVLDIF_Msk);

        if (IrqSt & USBD_BUSINTSTS_VBUSDETIF_Msk) {
            if (USBD_IS_ATTACHED()) {
                /* USB Plug In */
                USBD_ENABLE_USB();
            } else {
                /* USB Un-plug */
                USBD_DISABLE_USB();
            }
            USBD_CLR_BUS_INT_FLAG(USBD_BUSINTSTS_VBUSDETIF_Msk);
        }
    }

    if (IrqStL & USBD_GINTSTS_CEPIF_Msk) {
        IrqSt = USBD->CEPINTSTS & USBD->CEPINTEN;

        if (IrqSt & USBD_CEPINTSTS_SETUPTKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPTKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_SETUPPKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_SETUPPKIF_Msk);
            USBD_ProcessSetupPacket();
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_OUTTKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_OUTTKIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_INTKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
            if (!(IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk)) {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk);
                USBD_CtrlIn();
            } else {									
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_TXPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_PINGIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_PINGIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_TXPKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            if (g_usbd_CtrlInSize) {
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
            } else {
								if (g_usbd_CtrlZero == 1)
                    USBD_SET_CEP_STATE(USB_CEPCTL_ZEROLEN);		
                USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            }
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_TXPKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_RXPKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_RXPKIF_Msk);
            USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_NAKIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_NAKIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STALLIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STALLIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_ERRIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_ERRIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_STSDONEIF_Msk) {
            USBD_UpdateDeviceState();
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
            USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_BUFFULLIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_BUFFULLIF_Msk);
            return;
        }

        if (IrqSt & USBD_CEPINTSTS_BUFEMPTYIF_Msk) {
            USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_BUFEMPTYIF_Msk);
            return;
        }
    }

    /* interrupt in */
    if (IrqStL & USBD_GINTSTS_EPAIF_Msk) {
        IrqSt = USBD->EP[EPA].EPINTSTS & USBD->EP[EPA].EPINTEN;

        USBD_ENABLE_EP_INT(EPA, 0);
        USBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }
    /* interrupt out */
    if (IrqStL & USBD_GINTSTS_EPBIF_Msk) {
        IrqSt = USBD->EP[EPB].EPINTSTS & USBD->EP[EPB].EPINTEN;
			
        EPB_Handler();
        USBD_CLR_EP_INT_FLAG(EPB, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPCIF_Msk) {
        IrqSt = USBD->EP[EPC].EPINTSTS & USBD->EP[EPC].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPC, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPDIF_Msk) {
        IrqSt = USBD->EP[EPD].EPINTSTS & USBD->EP[EPD].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPD, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPEIF_Msk) {
        IrqSt = USBD->EP[EPE].EPINTSTS & USBD->EP[EPE].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPE, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPFIF_Msk) {
        IrqSt = USBD->EP[EPF].EPINTSTS & USBD->EP[EPF].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPF, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPGIF_Msk) {
        IrqSt = USBD->EP[EPG].EPINTSTS & USBD->EP[EPG].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPG, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPHIF_Msk) {
        IrqSt = USBD->EP[EPH].EPINTSTS & USBD->EP[EPH].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPH, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPIIF_Msk) {
        IrqSt = USBD->EP[EPI].EPINTSTS & USBD->EP[EPI].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPI, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPJIF_Msk) {
        IrqSt = USBD->EP[EPJ].EPINTSTS & USBD->EP[EPJ].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPJ, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPKIF_Msk) {
        IrqSt = USBD->EP[EPK].EPINTSTS & USBD->EP[EPK].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPK, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPLIF_Msk) {
        IrqSt = USBD->EP[EPL].EPINTSTS & USBD->EP[EPL].EPINTEN;
        USBD_CLR_EP_INT_FLAG(EPL, IrqSt);
    }
}

/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void HID_InitForHighSpeed(void)
{
    /* EPA ==> Interrupt IN endpoint, address 1 */
    USBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPA, EPA_MAX_PKT_SIZE);
    USBD_ConfigEp(EPA, HID_IN_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_IN);
	
	  /* EPB ==> Interrupt OUT endpoint, address 1 */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPB, EPB_MAX_PKT_SIZE);
    USBD_ConfigEp(EPB, HID_OUT_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_OUT);
	
		USBD->EP[EPA].EPRSPCTL = (USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_MODE_AUTO);
	
		USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);
	
    g_u32EpMaxPacketSize = EPA_MAX_PKT_SIZE;	
}

void HID_InitForFullSpeed(void)
{
    /* EPA ==> Interrupt IN endpoint, address 1 */
    USBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPA, EPA_FULL_MAX_PKT_SIZE);
    USBD_ConfigEp(EPA, HID_IN_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_IN);
	
	  /* EPB ==> Interrupt OUT endpoint, address 1 */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPB, EPB_FULL_MAX_PKT_SIZE);
    USBD_ConfigEp(EPB, HID_OUT_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_OUT);
		
		USBD->EP[EPA].EPRSPCTL = (USB_EP_RSPCTL_FLUSH|USB_EP_RSPCTL_MODE_AUTO);
	
		USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);
	
    g_u32EpMaxPacketSize = EPA_FULL_MAX_PKT_SIZE;
}

void HID_Init(void)
{
    /* Configure USB controller */
    /* Enable USB BUS, CEP and EPB global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIEN_Msk|USBD_GINTEN_CEPIEN_Msk|USBD_GINTEN_EPBIEN_Msk);
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    USBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);

    /*****************************************************/
    HID_InitForHighSpeed();
}

void HID_ClassRequest(void)
{
    if (gUsbCmd.bmRequestType & 0x80)
		{ /* request data transfer direction */
        // Device to host
        switch (gUsbCmd.bRequest) 
				{   
						case GET_IDLE:
						{
								USBD_PrepareCtrlIn((uint8_t *)&g_hid_idle, 1);				
								USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
								USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
								break;
						}		
						case GET_PROTOCOL:
						{
								USBD_PrepareCtrlIn((uint8_t *)&g_hid_protocol, 1);				
								USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
								USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
								break;
						}			
						case GET_REPORT:
//						{
//                 break;
//						}
						default: 
						{
								/* Setup error, stall the device */
								USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
								break;
						}
        }
    } 
		else 
		{
        // Host to device
        switch (gUsbCmd.bRequest)
				{
						case SET_REPORT: {
								if (((gUsbCmd.wValue >> 8) & 0xff) == 3) {
										/* Request Type = Feature */
										USBD_CtrlOut((uint8_t*)&g_hid_report, gUsbCmd.wLength);
										USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
										USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
										USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);									
							//	printf("SET_REPORT %d\n",g_hid_report);
								}
								break;
						}
						case SET_IDLE: 
						{
								g_hid_idle = (gUsbCmd.wValue >> 8) & 0xff;
								/* Status stage */
								USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
								USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
								USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
		//								printf("Set Idle\n");
								break;
						}
						case SET_PROTOCOL:
						{
								g_hid_protocol = gUsbCmd.wValue;
								/* Status stage */
								USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
								USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
								USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
		//								printf("Set Protocol\n");
								break;
						}		
						default: {
								// Stall
								/* Setup error, stall the device */
								USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
								break;
						}
        }
    }
}


int HID_CmdEraseSectors(CMD_T *pCmd)
{
    uint32_t u32StartSector;
    uint32_t u32Sectors;
    
    u32StartSector = pCmd->u32Arg1;
    u32Sectors = pCmd->u32Arg2;

		printf("Erase command - Sector: %d   Sector Cnt: %d\n", u32StartSector, u32Sectors);
		Erase(u32StartSector, u32Sectors);
		/* To note the command has been done */
		pCmd->u8Cmd = HID_CMD_NONE;
		
		return 0;
}

int HID_CmdReadPages(CMD_T *pCmd)
{
    uint32_t u32StartPage;
    uint32_t u32Pages;
    uint32_t u32Address;
		uint32_t u32Len;
		uint32_t u32Loop;
		uint32_t i;
    u32StartPage = pCmd->u32Arg1;
    u32Pages     = pCmd->u32Arg2;

    if(u32Pages)
    {			
				printf("Read  command - Start page: %5d    Pages Numbers: %5d\n", u32StartPage, u32Pages);
			
				u32Len = u32Pages * PAGE_SIZE;

				u32Loop = u32Len / BUFFER_SIZE;
				for(i=0;i<u32Loop;i++)
				{
						/* Prepare Buffer & Data for Read */
						PrepareReadData(&u32Address, u32StartPage, BUFFER_SIZE);
					
						/* Trigger HID IN */
						MSC_BulkIn((uint32_t)u32Address, BUFFER_SIZE);
				}
				u32Loop = u32Len % BUFFER_SIZE;
				
				if(u32Loop)
				{
						/* Prepare Buffer & Data for Read */
						PrepareReadData(&u32Address, u32StartPage, u32Loop);
			
						/* Trigger HID IN */
						MSC_BulkIn((uint32_t)u32Address, u32Loop);				
				}	
    }

		return 0;
}

int HID_CmdWritePages(CMD_T *pCmd)
{
    uint32_t u32StartPage;
    uint32_t u32Pages;
    uint32_t u32Address;
		uint32_t u32Len;
		uint32_t u32Loop;
		uint32_t i;
    
    u32StartPage = pCmd->u32Arg1;
    u32Pages     = pCmd->u32Arg2;
	
		printf("Write command - Start page: %5d    Pages Numbers: %5d\n", u32StartPage, u32Pages);
	
		if(u32Pages)
    {    
				u32Len = u32Pages * PAGE_SIZE;

				u32Loop = u32Len / BUFFER_SIZE;
				for(i=0;i<u32Loop;i++)
				{					
						/* Prepare Buffer for Write */
						PrepareWriteBuffer(&u32Address , u32StartPage + i * BUFFER_SIZE, BUFFER_SIZE / PAGE_SIZE);
					
						/* Trigger HID OUT */
						MSC_BulkOut((uint32_t)u32Address, BUFFER_SIZE);
					
						/* Get Data from Buffer for Write - u32Address is the buffer address of (u32StartPage + i * BUFFER_SIZE) */
						GetDatatForWrite(u32Address, u32StartPage + i * BUFFER_SIZE,  BUFFER_SIZE / PAGE_SIZE);					
				}
				
				if(u32Len % BUFFER_SIZE)
				{						
						/* Prepare Buffer for Write */
						PrepareWriteBuffer(&u32Address , u32StartPage + u32Loop * BUFFER_SIZE, (u32Len % BUFFER_SIZE) / PAGE_SIZE);								
					
						/* Trigger HID OUT */
						MSC_BulkOut((uint32_t)u32Address, u32Len % BUFFER_SIZE);
					
						/* Get Data from Buffer for Write - u32Address is the buffer address of (u32StartPage + u32Loop * BUFFER_SIZE) */
						GetDatatForWrite(u32Address, u32StartPage + u32Loop * BUFFER_SIZE, (u32Len % BUFFER_SIZE) / PAGE_SIZE);			
				}
    }
		return 0;
}

int gi32CmdTestCnt = 0;
int HID_CmdTest(CMD_T *pCmd)
{
    int i;
    uint8_t *pu8;
    
    pu8 = (uint8_t *)pCmd;
    printf("Get test command #%d (%d bytes)\n", gi32CmdTestCnt++, pCmd->u8Size);
    for(i=0;i<pCmd->u8Size;i++)
    {
        if((i&0xF) == 0)
        {
            printf("\n");
        }
        printf(" %02x", pu8[i]);
    }
    
    printf("\n");    
    
		/* To note the command has been done */
		pCmd->u8Cmd = HID_CMD_NONE;
	
		return 0;
}

uint32_t CalCheckSum(uint8_t *buf, uint32_t size)
{
    uint32_t sum;
    int32_t i;

    i = 0;
    sum = 0;
    while(size--)
    {
        sum+=buf[i++];
    }

    return sum;
}

int32_t ProcessCommand(uint8_t *pu8Buffer, uint32_t u32BufferLen)
{
		uint32_t u32sum;

		pgCmd = (CMD_T  *)( (uint32_t)pu8Buffer);

    /* Check size */
    if((pgCmd->u8Size > sizeof(gCmd)) || (pgCmd->u8Size > u32BufferLen))    
        return -1;
        
    /* Check signature */
    if(pgCmd->u32Signature != HID_CMD_SIGNATURE)
        return -1;
        
    /* Calculate checksum & check it*/        
    u32sum = CalCheckSum((uint8_t *)pgCmd, pgCmd->u8Size);
    if(u32sum != pgCmd->u32Checksum)
        return -1;

		/* It's Command and Copy command from Buffer to Command Parameter */        
    memcpy((uint8_t *)&gCmd, (uint8_t *)pgCmd, sizeof(gCmd));
    
    /* Check Command */
		switch(gCmd.u8Cmd)
		{
				case HID_CMD_ERASE:
				{
						HID_CmdEraseSectors(&gCmd);
						break;
				}		
				case HID_CMD_READ:
				{
						HID_CmdReadPages(&gCmd);
						break;
				}		
				case HID_CMD_WRITE:
				{
						HID_CmdWritePages(&gCmd);
						break;		
				}
				case HID_CMD_TEST:
				{
						HID_CmdTest(&gCmd);
						break;
				}
				default:
						return -1;
		}	
		
		return 0;
}


void HID_SetOutReport(uint8_t *pu8EpBuf, uint32_t u32Size)
{   
    /* Check and process the command packet */
    if(ProcessCommand(pu8EpBuf, u32Size))
    {
        printf("Unknown HID command!\n");
    }   
}

/* USB Endpoint B Interrupt Callback function */
void EPB_Handler(void)
{
		int size = 0;
		/* Receive data from HOST (CMD/Data) */
		if(USBD_GET_EP_INT_FLAG(EPB) & USBD_EPINTSTS_RXPKIF_Msk)
		{
				size = USBD->EP[EPB].EPDATCNT;
				/* Trigger DMA to move data from USB Engine Buffer to DRAM buffer */
				MSC_BulkOut((uint32_t)u8BufferOUT,  size);
				/* Deal with the Data/Command */
				HID_SetOutReport(u8BufferOUT, size);
		}
}

void MSC_ActiveDMA(uint32_t u32Addr, uint32_t u32Len)
{
    /* Enable BUS interrupt */
    USBD_SET_DMA_ADDR(u32Addr);
    USBD_SET_DMA_LEN(u32Len);
    g_usbd_DmaDone = 0;
	
    USBD_ENABLE_DMA();
    while(1) 
		{
        if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
            break;
				else
			   if (!USBD_IS_ATTACHED())
            break;
    }
}

void MSC_BulkOut(uint32_t u32Addr, uint32_t u32Len)
{
    uint32_t u32Loop;
    uint32_t i;

    /* bulk out, dma write, epnum = 2 */
    USBD_SET_DMA_WRITE(HID_OUT_EP_NUM);
    g_usbd_ShortPacket = 0;

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++) {
        MSC_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN);
    }

    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop) {
        MSC_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, u32Loop);
    }
}

void MSC_BulkIn(uint32_t u32Addr, uint32_t u32Len)
{
    uint32_t u32Loop;
    uint32_t i, addr, count;
    /* bulk in, dma read, epnum = 1 */
    USBD_SET_DMA_READ(HID_IN_EP_NUM);

    u32Loop = u32Len / USBD_MAX_DMA_LEN;
    for (i=0; i<u32Loop; i++) {
        USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
        g_usbd_ShortPacket = 0;
        while(1) {
            if (USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
                MSC_ActiveDMA(u32Addr+i*USBD_MAX_DMA_LEN, USBD_MAX_DMA_LEN);
                break;
            }
        }
    }

    addr = u32Addr + i * USBD_MAX_DMA_LEN;
    u32Loop = u32Len % USBD_MAX_DMA_LEN;
    if (u32Loop) {
        count = u32Loop / g_u32EpMaxPacketSize;
        if (count) {
            USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
            g_usbd_ShortPacket = 0;
            while(1) {
                if (USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
                    MSC_ActiveDMA(addr, count * g_u32EpMaxPacketSize);
                    break;
                }
            }
            addr += (count * g_u32EpMaxPacketSize);
        }
        count = u32Loop % g_u32EpMaxPacketSize;
        if (count) {
            USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
            g_usbd_ShortPacket = 1;
            while(1) {
                if (USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk) {
                    MSC_ActiveDMA(addr, count);
                    break;
                }
            }
        }
    }
}
