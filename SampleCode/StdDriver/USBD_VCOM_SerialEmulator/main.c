/******************************************************************************
 * @file     main.c
 * @brief    Demonstrate how to implement a USB virtual com port device.
 * @version  2.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC505Series.h"
#include "vcom_serial.h"

#if defined (__ICCARM__)
#include <string.h>
void USBD_IRQHandler_SRAM(void);
#endif

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
#ifdef __ICCARM__
#pragma data_alignment=4
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint8_t comTbuf[TXBUFSIZE];
uint8_t gRxBuf[64] = {0};
uint8_t gUsbRxBuf[64] = {0};
#else
__align(4) volatile uint8_t comRbuf[RXBUFSIZE];
__align(4) volatile uint8_t comTbuf[TXBUFSIZE];
__align(4) uint8_t gRxBuf[64] = {0};
__align(4) uint8_t gUsbRxBuf[64] = {0};
#endif


volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

/*--------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint8_t bInChar;
    int32_t size;
    uint32_t u32IntStatus;

    u32IntStatus = UART0->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAINT_Msk) || (u32IntStatus & UART_INTSTS_RXTOINT_Msk)) {
        /* Receiver FIFO threashold level is reached or Rx time out */

        /* Get all the input characters */
        while( (!UART_GET_RX_EMPTY(UART0)) ) {
            /* Get the character from UART Buffer */
            bInChar = UART_READ(UART0);    /* Rx trigger level is 1 byte*/

            /* Check if buffer full */
            if(comRbytes < RXBUFSIZE) {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;
                if(comRtail >= RXBUFSIZE)
                    comRtail = 0;
                comRbytes++;
            } else {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREINT_Msk) {

        if(comTbytes) {
            /* Fill the Tx FIFO */
            size = comTbytes;
            if(size >= TX_FIFO_SIZE) {
                size = TX_FIFO_SIZE;
            }

            while(size) {
                bInChar = comTbuf[comThead++];
                UART_WRITE(UART0, bInChar);
                if(comThead >= TXBUFSIZE)
                    comThead = 0;
                comTbytes--;
                size--;
            }
        } else {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check if any data to send to USB & USB is ready to send them out */
    if(comRbytes && (gu32TxSize == 0)) {
        i32Len = comRbytes;
        if(i32Len > EPA_MAX_PKT_SIZE)
            i32Len = EPA_MAX_PKT_SIZE;

        for(i=0; i<i32Len; i++) {
            gRxBuf[i] = comRbuf[comRhead++];
            if(comRhead >= RXBUFSIZE)
                comRhead = 0;
        }

        NVIC_DisableIRQ(UART0_IRQn);
        comRbytes -= i32Len;
        NVIC_EnableIRQ(UART0_IRQn);

        gu32TxSize = i32Len;
        for (i=0; i<i32Len; i++)
            USBD->EP[EPA].EPDAT_BYTE = gRxBuf[i];
        USBD->EP[EPA].EPRSPCTL = USB_EP_RSPCTL_SHORTTXEN;    // packet end
        USBD->EP[EPA].EPTXCNT = i32Len;
        USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_INTKIEN_Msk);
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes)) {
        for(i=0; i<gu32RxSize; i++) {
            comTbuf[comTtail++] = gUsbRxBuf[i];
            if(comTtail >= TXBUFSIZE)
                comTtail = 0;
        }

        NVIC_DisableIRQ(UART0_IRQn);
        comTbytes += gu32RxSize;
        NVIC_EnableIRQ(UART0_IRQn);

        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */
    }

    /* Process the software Tx FIFO */
    if(comTbytes) {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0) {
            /* Send one bytes out */
            UART_WRITE(UART0, comTbuf[comThead++]);
            if(comThead >= TXBUFSIZE)
                comThead = 0;

            NVIC_DisableIRQ(UART0_IRQn);
            comTbytes--;
            NVIC_EnableIRQ(UART0_IRQn);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    SYS_Init();
 
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
	
  
		/* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    printf("NuMicro USB CDC VCOM\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    NVIC_EnableIRQ(USBD_IRQn);

    /* Start transaction */
    while(1) {
        if (USBD_IS_ATTACHED()) {
            USBD_Start();
            break;
        }
    }

    while(1) {
        VCOM_TransferData();
    }
}



/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

