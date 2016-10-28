/******************************************************************************
 * @file     usbd_audio.c
 * @brief    NuMicro series USBD driver Sample file
 * @version  V4.2
 * @date     2015/09/30 08:23 p.m.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NUC505Series.h"
#include "usbd_audio.h"

#define TIME_OUT 1000

/*--------------------------------------------------------------------------*/
volatile uint8_t bPlayVolumeLAdjust = FALSE;
volatile uint8_t bPlayVolumeRAdjust = FALSE;
volatile uint8_t bSampleRateAdjust = FALSE;
volatile uint8_t bPlayMuteAdjust = FALSE;

/*--------------------------------------------------------------------------*/
/* Global variables for Audio class */
#ifdef __ICCARM__
#pragma data_alignment=4
volatile uint32_t g_usbd_SampleRate = PLAY_DEF_RATE;
volatile uint32_t g_usbd_SampleBitRate = PLAY_BIT_RATE;
static volatile uint32_t g_usbd_temp1;
#else
__align(4) volatile uint32_t g_usbd_SampleRate = PLAY_DEF_RATE;
__align(4) volatile uint32_t g_usbd_SampleBitRate = PLAY_BIT_RATE;
__align(4) static volatile uint32_t g_usbd_temp1;
#endif

uint32_t g_ubsd_rx_flag = 0;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  g_usbd_PlayMute = 0x00;
int16_t  g_usbd_PlayVolumeL = 16;
int16_t  g_usbd_PlayVolumeR = 16;
int16_t  g_usbd_PlayMaxVolume = 32;
int16_t  g_usbd_PlayMinVolume = 0;
int16_t  g_usbd_PlayResVolume = 1;
#else
__align(4)  uint8_t  g_usbd_PlayMute = 0x00;
__align(4)  int16_t  g_usbd_PlayVolumeL = 16;
__align(4)  int16_t  g_usbd_PlayVolumeR = 16;
__align(4)  int16_t  g_usbd_PlayMaxVolume = 32;
__align(4)  int16_t  g_usbd_PlayMinVolume = 0;
__align(4)  int16_t  g_usbd_PlayResVolume = 1;
#endif
/* Playback Buffer and its pointer */
#ifdef __ICCARM__
#pragma data_alignment=4
int32_t PcmPlayBuff[RING_BUF_SMPL_CNT] = {0};

#pragma data_alignment=4
int32_t PcmPlayBuff2[192] = {0};

#else
__align(4) int32_t PcmPlayBuff[RING_BUF_SMPL_CNT] = {0};

__align(4) int32_t PcmPlayBuff2[192] = {0};

#endif

volatile static uint32_t s_u32PlayBufPos=0;

volatile uint8_t g_u8AudioPlaying=0;

/* FIXME */
volatile static uint32_t s_u32Temp0 = 0;
volatile static uint32_t s_u32Cnt = 0;
volatile uint32_t g_u32Temp1;
volatile uint32_t g_u32Temp2;
volatile static uint32_t s_u32CntDnFlg = 0;
volatile static int32_t s_i32CntDn;

//extern int32_t g_volume_flag;
extern uint32_t g_u32Temp3;
extern uint32_t g_u32Temp4;
//extern uint32_t g_u32Temp5;
extern uint32_t g_u32Temp6;
extern uint32_t g_u32Temp7;
extern uint32_t g_u32Temp8;
extern uint32_t g_u32Temp9;

extern volatile uint32_t g_u32DecreaseAPLL;
extern volatile uint32_t g_u32CurrentAPLL;
extern volatile uint32_t g_u32IncreaseAPLL;
extern volatile uint32_t g_u32RefAPLL;

volatile static uint32_t s_u32SpkChkCnt = 0;

volatile static uint32_t s_u32SpkEnabled = 0;

//extern volatile int32_t g_i32AudioPlayingCnt;

volatile uint32_t g_u32Flag1 = 0;
volatile uint32_t g_u32Flag2 = 0;
volatile uint32_t g_u32Flag3 = 0;

#ifdef __HID__
uint8_t volatile g_u8EPCReady = 0;
void EPC_Handler(void)  /* Interrupt IN handler */
{
    g_u8EPCReady = 1;
}
#endif
/*--------------------------------------------------------------------------*/
/**
 * @brief       USBD Interrupt Service Routine
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is the USBD ISR
 */
void USBD_IRQHandler(void)
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
						g_ubsd_rx_flag = 0;
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

            if (!(USBD->DMACTL & USBD_DMACTL_DMARD_Msk)) {
                USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk);
            }

            if (USBD->DMACTL & USBD_DMACTL_DMARD_Msk) {
                if (g_usbd_ShortPacket == 1) {
                    USBD->EP[EPA].EPRSPCTL = USB_EP_RSPCTL_SHORTTXEN;    // packet end
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
						
						g_ubsd_rx_flag = 0;
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

    /* Isochronous in */
    if (IrqStL & USBD_GINTSTS_EPAIF_Msk) {
        IrqSt = USBD->EP[EPA].EPINTSTS & USBD->EP[EPA].EPINTEN;
       // USBD_ENABLE_EP_INT(EPA, 0);
        USBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }
    /* Isochronous out */
    if (IrqStL & USBD_GINTSTS_EPBIF_Msk) {
        IrqSt = USBD->EP[EPB].EPINTSTS & USBD->EP[EPB].EPINTEN;
				g_ubsd_rx_flag++;
        USBD_CLR_EP_INT_FLAG(EPB, IrqSt);
    }

    if (IrqStL & USBD_GINTSTS_EPCIF_Msk) {
        IrqSt = USBD->EP[EPC].EPINTSTS & USBD->EP[EPC].EPINTEN;
#ifdef __HID__			
        EPC_Handler();		
#endif			
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

/**
 * @brief       EPB Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EPB event
 */
/* Play */
void EPB_Handler(void)
{
	uint32_t u32BuffIndex = 0, u32DMAIndex;
	int32_t i32Temp1;

	uint32_t u32len;
	uint32_t i;

	uint8_t *PcmPlayBuff3;
	uint32_t *PcmPlayBuff4 = NULL;

	uint16_t *PcmPlayBuff33;
	uint16_t *PcmPlayBuff44 = NULL;

	int8_t i8PlayLVol, i8PlayRVol;

	while(1) {
		if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
			break;

		if (!USBD_IS_ATTACHED())
			break;
	}
	
	// get data from FIFO
	u32len = USBD->EP[EPB].EPDATCNT & 0xffff;

	/* FIXME */
	if (u32len != g_u32Temp1)
	{
		if (u32len != g_u32Temp2)
		{
			USBD->EP[EPB].EPRSPCTL |= USBD_EPRSPCTL_FLUSH_Msk;
//			printf("s %d\n", u32len);
			return;
		}
	}

	if (g_u32Flag1)
	{
		if (USBD->CEPINTSTS & USBD_CEPINTSTS_RXPKIF_Msk)
		{
			USBD_CtrlOUT((uint32_t)&g_usbd_PlayVolumeL, gUsbCmd.wLength);
			bPlayVolumeLAdjust = TRUE;
			g_u32Flag1 = 0;
			/* Status stage */
			USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
			USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
			USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
		}
	}

	if (g_u32Flag2)
	{
		if (USBD->CEPINTSTS & USBD_CEPINTSTS_RXPKIF_Msk)
		{
			USBD_CtrlOUT((uint32_t)&g_usbd_PlayVolumeR, gUsbCmd.wLength);
			bPlayVolumeRAdjust = TRUE;
			g_u32Flag2 = 0;
			/* Status stage */
			USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
			USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
			USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
		}
	}

	if (g_u32Flag3)
	{
		if (USBD->CEPINTSTS & USBD_CEPINTSTS_RXPKIF_Msk)
		{
			USBD_CtrlOUT((uint32_t)&g_usbd_PlayMute, gUsbCmd.wLength);
			bPlayMuteAdjust = TRUE;
			g_u32Flag3 = 0;
			/* Status stage */
			USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
			USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
			USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
		}
	}

	if (bPlayVolumeLAdjust)
	{
		bPlayVolumeLAdjust = FALSE;
		if ( g_usbd_PlayMute ) return;
		/* FIXME: need to ramp up or ramp down to avoid pop-noise */
		i8PlayLVol = g_usbd_PlayVolumeL;
		if (i8PlayLVol <= 0 )
		{
			i8PlayLVol = 31;	// Analog Mute
		}
		else
		{
			i8PlayLVol--;
			if (i8PlayLVol > 30)	i8PlayLVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
			i8PlayLVol = 30 - i8PlayLVol;
		}
		I2S_SET_INTERNAL_CODEC(I2S, 0x08, i8PlayLVol);
//		printf("L %d\n", g_usbd_PlayVolumeL);
		/* FIXME: we modify min, max, and res for H/W range */
	}

	if (bPlayVolumeRAdjust)
	{
		bPlayVolumeRAdjust = FALSE;
		if ( g_usbd_PlayMute ) return;
		/* FIXME: need to ramp up or ramp down to avoid pop-noise */
		i8PlayRVol = g_usbd_PlayVolumeR;
		if (i8PlayRVol <= 0 )
		{
			i8PlayRVol = 31;	// Analog Mute
		}
		else
		{
			i8PlayRVol--;
			if (i8PlayRVol > 30)	i8PlayRVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
			i8PlayRVol = 30 - i8PlayRVol;
		}
		I2S_SET_INTERNAL_CODEC(I2S, 0x09, i8PlayRVol);
//		printf("R %d\n", g_usbd_PlayVolumeR);
		/* FIXME: we modify min, max, and res for H/W range */
	}

	/* FIXME workaround for mute/un-mute noise */
	if (bPlayMuteAdjust)
	{
		bPlayMuteAdjust = FALSE;
		if (g_usbd_PlayMute)
		{
			/* FIXME: need to ramp down to avoid pop-noise */
			UAC_DeviceDisable();
		}
		else
		{
			s_u32CntDnFlg = 1;
			s_i32CntDn = 100;
		}

//		printf("PM %d\n", g_usbd_PlayMute);
	}

	if ( g_usbd_PlayMute )
	{
		USBD->EP[EPB].EPRSPCTL |= USBD_EPRSPCTL_FLUSH_Msk;
		return;
	}

	if ( s_u32CntDnFlg )
	{
		if ( s_i32CntDn-- <= 0 )
		{
			/* FIXME: need to ramp up to avoid pop-noise */
			i8PlayLVol = g_usbd_PlayVolumeL;
			if (i8PlayLVol <= 0 )
			{
				i8PlayLVol = 31;	// Analog Mute
			}
			else
			{
				i8PlayLVol--;
				if (i8PlayLVol > 30)	i8PlayLVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
				i8PlayLVol = 30 - i8PlayLVol;
			}
			i8PlayRVol = g_usbd_PlayVolumeR;
			if (i8PlayRVol <= 0 )
			{
				i8PlayRVol = 31;	// Analog Mute
			}
			else
			{
				i8PlayRVol--;
				if (i8PlayRVol > 30)	i8PlayRVol = 30;	// from 0 dB, -2 dB, -4 dB, ... , -60 dB
				i8PlayRVol = 30 - i8PlayRVol;
			}
			I2S_SET_INTERNAL_CODEC(I2S, 0x08, i8PlayLVol);
			I2S_SET_INTERNAL_CODEC(I2S, 0x09, i8PlayRVol);
			s_u32CntDnFlg = 0;
		}
		else
		{
			USBD->EP[EPB].EPRSPCTL |= USBD_EPRSPCTL_FLUSH_Msk;
			return;
		}
	}

	if ( g_usbd_SampleBitRate == 24 )
	PcmPlayBuff4 = (uint32_t *)PcmPlayBuff;
	else if ( g_usbd_SampleBitRate == 16 )
	PcmPlayBuff44 = (uint16_t *)PcmPlayBuff;
	else
		printf( "Error! Bit rate not supported %d\n", g_usbd_SampleBitRate );

	/* FIXME to detect noise and try to fix */
	if ( g_u8AudioPlaying )
	{
		if ( s_u32SpkChkCnt++ % 2 )
		{
			u32DMAIndex = I2S_GET_TXDMA_CADDR(I2S);
//			printf("0x%x\n", u32DMAIndex);
			if ( g_usbd_SampleBitRate == 24 )
			u32BuffIndex = (uint32_t)&PcmPlayBuff4[s_u32PlayBufPos];
			else if ( g_usbd_SampleBitRate == 16 )
			u32BuffIndex = (uint32_t)&PcmPlayBuff44[s_u32PlayBufPos];
			else
				printf( "Error! Bit rate not supported %d\n", g_usbd_SampleBitRate );

			i32Temp1 = u32BuffIndex - u32DMAIndex;

			if ( i32Temp1 < 0 )
				i32Temp1 = RING_BUF_SMPL_SZ + i32Temp1;

			if ( i32Temp1 < g_u32Temp6 )
			{
				if (g_u32RefAPLL != g_u32DecreaseAPLL)
				{
					/* TX too fast, slowing down TX to avoid noise */
					CLK_SET_APLL(g_u32DecreaseAPLL);
					g_u32RefAPLL = g_u32DecreaseAPLL;
					printf("s-%d\n", i32Temp1);
				}
//				else
//				{
//					printf("s- %d\n", i32Temp1);
//				}
			}
			else if ( i32Temp1 > RING_BUF_SMPL_SZ_CP )
			{
				if (g_u32RefAPLL != g_u32CurrentAPLL)
				{
					/* TX too slow, speeding up TX to avoid noise */
					CLK_SET_APLL(g_u32CurrentAPLL);
					g_u32RefAPLL = g_u32CurrentAPLL;
					printf("s+%d\n", i32Temp1);
				}
//				else
//				{
//					printf("s+ %d\n", i32Temp1);
//				}
			}

			if ( i32Temp1 > g_u32Temp7 )
			{
				if (g_u32RefAPLL != g_u32IncreaseAPLL)
				{
					/* TX too slow, speeding up TX to avoid noise */
					CLK_SET_APLL(g_u32IncreaseAPLL);
					g_u32RefAPLL = g_u32IncreaseAPLL;
					printf("s++%d\n", i32Temp1);
				}
//				else
//				{
//					printf("s++ %d\n", i32Temp1);
//				}
			}

			if ( i32Temp1 < g_u32Temp1 )
			{
				/* FIXME may causes noise */
				printf( "s<%d\n", i32Temp1 );
			}
			else if ( i32Temp1 > g_u32Temp9 )
			{
				/* FIXME may causes noise */
				printf( "s>%d\n", i32Temp1 );
			}
//			else
//			{
				/* Speaker: here is safe range and noise is under controlled. */
//				printf("\t%d\n", i32Temp1);
//			}
		}
	}

		/* USB DMA */
		USBD_SET_DMA_WRITE(ISO_OUT_EP_NUM);
		USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
		USBD_SET_DMA_ADDR((uint32_t)&PcmPlayBuff2[0]);
		USBD_SET_DMA_LEN(u32len);
		g_usbd_DmaDone = 0;
		USBD_ENABLE_DMA();

		while(1) {
				if (g_usbd_DmaDone)
						break;

				if (!USBD_IS_ATTACHED())
						break;
		}

		/* FIXME workaround playback TX DMA data repeatedly */
//		g_i32AudioPlayingCnt = 1000;

		/* FIXME: CPU ring buffer process */
	if ( g_usbd_SampleBitRate == 24 )
	{
		PcmPlayBuff3 = (uint8_t *)PcmPlayBuff2;
		for (i = 0; i < u32len; i+=3)
		{
			if (s_u32PlayBufPos == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32PlayBufPos = 0;
//			PcmPlayBuff4[s_u32PlayBufPos++] = (0x00 << 24) | (PcmPlayBuff3[i+2] << 16) | (PcmPlayBuff3[i+1] << 8) | PcmPlayBuff3[i+0];
			/* FIXME: workaround for external CODEC as slave @ 24-bit sample size */
			PcmPlayBuff4[s_u32PlayBufPos++] = (PcmPlayBuff3[i+2] << 24) | (PcmPlayBuff3[i+1] << 16) | (PcmPlayBuff3[i+0] << 8) | 0x00;
		}
	}
	else if ( g_usbd_SampleBitRate == 16 )
	{
		PcmPlayBuff33 = (uint16_t *)PcmPlayBuff2;
		u32len = u32len >> 1;   /* byte size to sample count */
		for (i = 0; i < u32len; i+=2)
		{
			/* if over flow */
			if (s_u32PlayBufPos == RING_BUF_SMPL_CNT2)
				s_u32PlayBufPos = 0;

			/* left channel */
			PcmPlayBuff44[s_u32PlayBufPos++] = PcmPlayBuff33[i+1];

			/* if over flow */
			if (s_u32PlayBufPos == RING_BUF_SMPL_CNT2)
				s_u32PlayBufPos = 0;
	
			/* right channel */
			PcmPlayBuff44[s_u32PlayBufPos++] = PcmPlayBuff33[i];
		}
	}
	else
		printf( "Error! Bit rate not supported %d\n", g_usbd_SampleBitRate );

		/* FIXME: H/W start to playback TX DMA play buffer */
		/* FIXME: g_u32Temp4 too small will cause noise, too large will cause long delay */
		/* FIXME: for now, we will playback TX DMA play buffer after 3ms */
		/* FIXME: g_u32Temp4 is set in main and please be careful to modify it if needed */
		if ( g_u8AudioPlaying == 0 )
		{
//			printf("  %d\n", (s_u32PlayBufPos * 3));
			if ( s_u32PlayBufPos >= g_u32Temp4 )
			{
//				u32DMAIndex = I2S_GET_TXDMA_CADDR(I2S);
				/* buffer has enough data, play it!! */
				AudioStartPlay();
				g_u8AudioPlaying = 1;
//				printf("   %d 0x%x\n", (s_u32PlayBufPos * 3), u32DMAIndex);
			}
		}
}

/*--------------------------------------------------------------------------*/
/**
 * @brief       UAC Class Initial
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to configure endpoints for UAC class
 */
void UAC_Init(void)
{
    /* Configure USB controller */
    USBD->OPER = 0; /* Full Speed */
#ifdef __HID__	
    /* Enable USB BUS, CEP and EPA , EPB , EPC global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIEN_Msk|USBD_GINTEN_CEPIEN_Msk|USBD_GINTEN_EPAIEN_Msk|USBD_GINTEN_EPBIEN_Msk|USBD_GINTEN_EPCIEN_Msk);
#else
    /* Enable USB BUS, CEP and EPA , EPB global interrupt */
    USBD_ENABLE_USB_INT(USBD_GINTEN_USBIEN_Msk|USBD_GINTEN_CEPIEN_Msk|USBD_GINTEN_EPAIEN_Msk|USBD_GINTEN_EPBIEN_Msk);
#endif	
    /* Enable BUS interrupt */
    USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_RESUMEIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
    /* Reset Address to 0 */
    USBD_SET_ADDR(0);

    /*****************************************************/
    /* Control endpoint */
    USBD_SetEpBufAddr(CEP, CEP_BUF_BASE, CEP_BUF_LEN);
    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_SETUPPKIEN_Msk|USBD_CEPINTEN_STSDONEIEN_Msk);

    /*****************************************************/
    /* EPB ==> ISO OUT endpoint, address 2 */
    USBD_SetEpBufAddr(EPB, EPB_BUF_BASE, EPB_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPB, EPB_MAX_PKT_SIZE);
    USBD_ConfigEp(EPB, ISO_OUT_EP_NUM, USB_EP_CFG_TYPE_ISO, USB_EP_CFG_DIR_OUT);
    USBD_ENABLE_EP_INT(EPB, USBD_EPINTEN_RXPKIEN_Msk|USBD_EPINTEN_SHORTRXIEN_Msk);
#ifdef __HID__		
    /* EPC ==> Interrupt IN endpoint, address 3 */
    USBD_SetEpBufAddr(EPC, EPC_BUF_BASE, EPC_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPC, EPC_MAX_PKT_SIZE);
    USBD_ConfigEp(EPC, HID_IN_EP_NUM, USB_EP_CFG_TYPE_INT, USB_EP_CFG_DIR_IN);		
		USBD_ENABLE_EP_INT(EPC, USBD_EPINTEN_INTKIEN_Msk);
#endif		
}


/**
 * @brief       UAC class request
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process UAC class requests
 */
void USBD_CtrlOUT(uint32_t u32addr, uint32_t u32Len)
{
		uint32_t volatile count = 0;
		uint32_t volatile dmactrl = 0, dmaaddr = 0,dmacnt = 0;
		while(1) 
		{
				if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
						break;

				if(count > TIME_OUT)	
				{
						printf("CTRL 0x%X Len %d Data %d ADDR 0x%X\n",USBD->DMACTL, USBD->DMACNT, USBD->CEPDATCNT,USBD->DMAADDR);
					
						count	= 0;
				}
				else
					count++;
				if (!USBD_IS_ATTACHED())
						break;
		}
		dmactrl = USBD->DMACTL;
		dmaaddr = USBD->DMAADDR;
		dmacnt = USBD->DMACNT;
		USBD_SET_DMA_WRITE(0);
		USBD_SET_DMA_ADDR(u32addr);
		USBD_SET_DMA_LEN(u32Len);
		USBD_ENABLE_DMA();	
		while(1) 
		{
				if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
						break;

				if (!USBD_IS_ATTACHED())
						break;
		}	
		USBD->CEPINTSTS = USBD_CEPINTSTS_RXPKIF_Msk;	
		USBD->DMACTL = dmactrl;
		USBD->DMAADDR = dmaaddr;
		USBD->DMACNT = dmacnt; 		
}

void UAC_ClassRequest(void)
{
		uint32_t volatile i = 0;
//		printf("%02X %02X %04X %04X %04X\n",gUsbCmd.bmRequestType, gUsbCmd.bRequest, gUsbCmd.wValue, gUsbCmd.wIndex, gUsbCmd.wLength);	
	
    if (gUsbCmd.bmRequestType & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (gUsbCmd.bRequest)
        {
            case UAC_GET_CUR:
            {
								if ((gUsbCmd.wIndex & 0xf) == ISO_OUT_EP_NUM) {    /* request to endpoint */
                    USBD_PrepareCtrlIn((uint8_t *)&g_usbd_SampleRate, gUsbCmd.wLength);
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
//									printf("d2h hgetc PR %d\n", g_usbd_SampleRate);
									
                }						
                else {							
							
										switch ((gUsbCmd.wValue & 0xff00) >> 8)
										{
												case MUTE_CONTROL:
												{
														if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
																USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMute, 1);
///														printf("  UAC_GET_CUR - MUTE_CONTROL\n");
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
														break;
												}
												case VOLUME_CONTROL:
												{
														if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
														{
																if((gUsbCmd.wValue & 0xff) == 1)
																		USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayVolumeL, 2);
																else
																		USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayVolumeR, 2);
///																printf("  PLAY - UAC_GET_CUR - VOLUME_CONTROL\n");
														}
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
														break;
												}
#if 0
												case AUTOMATIC_GAIN_CONTROL:
												{
														if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
																USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMute, 1);
///														printf("  UAC_GET_CUR - AUTOMATIC_GAIN_CONTROL\n");
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
														break;
												}
#endif
												default:
												{
														USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
														/* Setup error, stall the device */
														USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
														break;
												}
										}
								}
                break;
            }

            case UAC_GET_MIN:
            {
                switch ((gUsbCmd.wValue & 0xff00) >> 8)
                {
                    case VOLUME_CONTROL:
                    {
                        if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
												{
                            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMinVolume, 2);													
///														printf("  PLAY - UAC_GET_MIN\n");
												}
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                        break;
                    }
                    default:
										{
												USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
                        /* STALL control pipe */
                        USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
												break;
										}
                }
                break;
            }

            case UAC_GET_MAX:
            {
                switch ((gUsbCmd.wValue & 0xff00) >> 8)
                {
                    case VOLUME_CONTROL:
                    {
                        if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
												{
                            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayMaxVolume, 2);
													
///														printf("  PLAY - UAC_GET_MAX\n");
												}
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                        break;
                    }
                    default:
										{
												USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
                        /* STALL control pipe */
                        USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
												break;
										}
                }
                break;
            }

            case UAC_GET_RES:
            {
                switch ((gUsbCmd.wValue & 0xff00) >> 8)
                {
                    case VOLUME_CONTROL:
                    {
                        if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
												{
                            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_PlayResVolume, 2);
													
///														printf("  PLAY - UAC_GET_RES\n");
												}
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
                        break;
                    }
                    default:
										{
												USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
                        /* STALL control pipe */
                        USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);											
												break;
										}
                }
                break;
            }

            default:
            {
								USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
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
            case UAC_SET_CUR:
            {
                USBD_ENABLE_CEP_INT(USBD_CEPINTEN_OUTTKIEN_Msk | USBD_CEPINTEN_RXPKIEN_Msk);

								if ((gUsbCmd.wIndex & 0xf) == ISO_OUT_EP_NUM) {    /* request to endpoint */
										USBD_CtrlOUT((uint32_t)&g_usbd_SampleRate, gUsbCmd.wLength);
                    /* Status stage */
                    USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                    USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
//									printf("h2d set PS %d, %d, ", g_usbd_SampleRate, bSampleRateAdjust);
										bSampleRateAdjust = TRUE;
//									printf("%d\n", bSampleRateAdjust);
                }						
                else {  /* request to interface */							
							
							
										switch ((gUsbCmd.wValue & 0xff00) >> 8)
										{
												case MUTE_CONTROL:
														if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
														{
#if 0
																USBD_CtrlOUT((uint32_t)&g_usbd_PlayMute, gUsbCmd.wLength);
//															printf("h2d set PM %d, %d, ", g_usbd_PlayMute, bPlayMuteAdjust);
																bPlayMuteAdjust = TRUE;
//															printf("%d\n", bPlayMuteAdjust);
#else
																g_u32Flag3 = 1;
																return;
#endif
														}
														else
														{
																USBD_CtrlOUT((uint32_t)&g_usbd_temp1, gUsbCmd.wLength);											
//																printf("h2d set %d\n", g_usbd_temp1);															
																printf("g\n");
														}

														/* Status stage */
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
														USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
														break;

												case VOLUME_CONTROL:
														if (PLAY_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
														{
																if ((gUsbCmd.wValue & 0xff) == 1)
																{
#if 0													
																		USBD_CtrlOUT((uint32_t)&g_usbd_PlayVolumeL, gUsbCmd.wLength);
//																	printf("h2d set PL %d, %d, ", g_usbd_PlayVolumeL, bPlayVolumeLAdjust);
																		bPlayVolumeLAdjust = TRUE;
//																	printf("%d\n", bPlayVolumeLAdjust);
#else
																		g_u32Flag1 = 1;
																		return;
#endif
																}
																else
																{
#if 0
																		USBD_CtrlOUT((uint32_t)&g_usbd_PlayVolumeR, gUsbCmd.wLength);																		
//																		if(g_usbd_PlayVolumeR & 0x8000)
//																				g_usbd_PlayVolumeR = (g_usbd_PlayVolumeR & 0x7FFF) >> 8;
//																		else
//																				g_usbd_PlayVolumeR = (g_usbd_PlayVolumeR >> 7);
//																	printf("h2d set PR %d, %d, ", g_usbd_PlayVolumeR, bPlayVolumeRAdjust);
																		bPlayVolumeRAdjust = TRUE;
//																	printf("%d\n", bPlayVolumeRAdjust);
#else
																		g_u32Flag2 = 1;
																		return;
#endif
																}
														}
														else if (0x0d == ((gUsbCmd.wIndex >> 8) & 0xff))
														{
																USBD_CtrlOUT((uint32_t)&g_usbd_temp1, gUsbCmd.wLength);
//															printf("h2d set %d\n", g_usbd_temp1);
																printf("a\n");
														}
														else
														{
																USBD_CtrlOUT((uint32_t)&g_usbd_temp1, gUsbCmd.wLength);													
//																printf("h2d set %d\n", g_usbd_temp1);															
																printf("b\n");
														}
														/* Status stage */
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
														USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
														break;

												default:
												{
														USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
														/* STALL control pipe */
														USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
														printf("c\n");
														break;
												}
										}
								}
                break;
            }
#ifdef __HID__						
						case SET_REPORT:
						{
								if (((gUsbCmd.wValue >> 8) & 0xff) == 2) 
								{
										/* Request Type = Feature */
                    USBD_CtrlOUT((uint32_t)&g_usbd_temp1, gUsbCmd.wLength);
                    /* Status stage */		
										USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
										USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
										USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
//										printf("SET_REPORT %d %d\n",gUsbCmd.wLength,*(uint8_t *)g_usbd_temp1);
									printf("e\n");
								}
								break;
						}
						case SET_IDLE: 
						{
								/* Status stage */
								USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
								USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
								USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
//								printf("Set Idle\n");
							printf("f\n");
								break;
						}
						case SET_PROTOCOL:
#endif
            default:
            {
								USBD->CEPCTL = USBD_CEPCTL_FLUSH_Msk;
                /* Setup error, stall the device */
                USBD_SET_CEP_STATE(USBD_CEPCTL_STALLEN_Msk);
								printf("d\n");
                break;
            }
        }                
    }
}

/**
 * @brief       Set Interface standard request
 *
 * @param[in]   u32AltInterface Interface
 *
 * @return      None
 *
 * @details     This function is used to set UAC Class relative setting
 */
void UAC_SetInterface(uint32_t u32AltInterface)
{
     /* Audio Iso OUT interface */
        if (u32AltInterface == 2)
				{
					s_u32SpkEnabled = 1;
					g_usbd_SampleBitRate = 24;
//					bSampleRateAdjust = TRUE;
					printf("4-24bits\n");
				}
				else if (u32AltInterface == 1)
				{
					s_u32SpkEnabled = 1;
					g_usbd_SampleBitRate = 16;
//					bSampleRateAdjust = TRUE;
					printf("4\n");
//            UAC_DeviceEnable();
				}
        else
				{
					printf("2\n");
            UAC_DeviceDisable();
				}
}

/**   
  * @brief  UAC_DeviceEnable. To enable the device to play audio data.
  * @retval None.
  */
void UAC_DeviceEnable(void)
{
			if ( g_usbd_PlayMute )
			{
				I2S_SET_INTERNAL_CODEC(I2S, 0x08, 0x1F);
				I2S_SET_INTERNAL_CODEC(I2S, 0x09, 0x1F);
//				printf("PM1\n");
			}
}


/**   
  * @brief  UAC_DeviceDisable. To disable the device to play audio data.
  * @retval None.
  */
void UAC_DeviceDisable(void)
{
	uint32_t u32BuffIndex, u32DMAIndex=0;

	if (g_u32RefAPLL != g_u32CurrentAPLL)
	{
		CLK_SET_APLL(g_u32CurrentAPLL);
		g_u32RefAPLL = g_u32CurrentAPLL;
	}

			memset(PcmPlayBuff, 0, sizeof(PcmPlayBuff));

			/* FIXME: before stop DMA, wait current address to start address */
			if ( g_u8AudioPlaying )
			{
				u32DMAIndex = I2S_GET_TXDMA_CADDR(I2S);
				u32BuffIndex = (uint32_t)&PcmPlayBuff[0];
				while ( u32DMAIndex != u32BuffIndex )
					u32DMAIndex = I2S_GET_TXDMA_CADDR(I2S);
//				printf("0x%x\n", u32DMAIndex);
			}

        /* Disable play hardware/stop play */
        /* Disable I2S Tx function */
			I2S_DISABLE_TXDMA(I2S);
			I2S_DISABLE_TX(I2S);
			
			/* Reset some variables */
			s_u32PlayBufPos = 0;

			g_u8AudioPlaying = 0;
			s_u32SpkChkCnt = 0;
			s_u32SpkEnabled = 0;
			
			USBD->EP[EPB].EPRSPCTL |= USBD_EPRSPCTL_FLUSH_Msk;
			if((USBD->DMACTL & 0xF) == ISO_OUT_EP_NUM)
			{					
            USBD_ResetDMA();
			}
}

void AudioStartPlay(void)
{
	UAC_DeviceEnable();
	
  /* Enable I2S Tx function */
	I2S_ENABLE_TX(I2S);
  I2S_ENABLE_TXDMA(I2S);
}
