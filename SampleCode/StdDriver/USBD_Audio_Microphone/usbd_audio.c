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
volatile uint8_t bRecVolumeAdjust = FALSE;
volatile uint8_t bSampleRateAdjust = FALSE;
volatile uint8_t bRecMuteAdjust = FALSE;

/*--------------------------------------------------------------------------*/
/* Global variables for Audio class */
#ifdef __ICCARM__
#pragma data_alignment=4
volatile uint32_t g_usbd_SampleRate = REC_DEF_RATE;
#pragma data_alignment=4
volatile uint32_t g_usbd_SampleBitRate = REC_BIT_RATE;
#pragma data_alignment=4
static volatile uint32_t g_usbd_temp1;
#else
__align(4) volatile uint32_t g_usbd_SampleRate = REC_DEF_RATE;
__align(4) volatile uint32_t g_usbd_SampleBitRate = REC_BIT_RATE;
__align(4) static volatile uint32_t g_usbd_temp1;
#endif 

//uint32_t g_usbd_UsbAudioState = UAC_STOP_AUDIO_RECORD;

uint32_t g_ubsd_tx_flag = 0;
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  g_usbd_RecMute = 0x00;
#pragma data_alignment=4
int16_t  g_usbd_RecVolume = 8;
#pragma data_alignment=4
int16_t  g_usbd_RecMaxVolume = 16;
#pragma data_alignment=4
int16_t  g_usbd_RecMinVolume = 0;
#pragma data_alignment=4
int16_t  g_usbd_RecResVolume = 1;
#else
__align(4)  uint8_t  g_usbd_RecMute = 0x00;
__align(4)  int16_t  g_usbd_RecVolume = 8;
__align(4)  int16_t  g_usbd_RecMaxVolume = 16;
__align(4)  int16_t  g_usbd_RecMinVolume = 0;
__align(4)  int16_t  g_usbd_RecResVolume = 1;
#endif
/* Record Buffer and its pointer */
#ifdef __ICCARM__
#pragma data_alignment=4
int16_t PcmRecBuff[RING_BUF_SMPL_CNT] = {0};

#pragma data_alignment=4
int16_t PcmRecBuff2[192] = {0};

#else
__align(4) int16_t PcmRecBuff[RING_BUF_SMPL_CNT] = {0};

__align(4) int16_t PcmRecBuff2[192] = {0};

#endif

/* FIXME */
volatile uint32_t g_u32AudioRecordingStage = 0;
volatile static uint32_t s_u32Temp0 = 0;
volatile static uint32_t s_u32Cnt = 0;
volatile uint32_t g_u32Temp1;
volatile uint32_t g_u32Temp2;

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

volatile static uint32_t s_u32MicChkCnt = 0;

volatile static uint32_t s_u32SpkEnabled = 0;

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
						g_ubsd_tx_flag = 0;
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
						
						g_ubsd_tx_flag = 0;
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
				g_ubsd_tx_flag++;
       // USBD_ENABLE_EP_INT(EPA, 0);
        USBD_CLR_EP_INT_FLAG(EPA, IrqSt);
    }
    /* Isochronous out */
    if (IrqStL & USBD_GINTSTS_EPBIF_Msk) {
        IrqSt = USBD->EP[EPB].EPINTSTS & USBD->EP[EPB].EPINTEN;
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
 * @brief       EPA Handler
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to process EPA event
 */
/* Record */
void EPA_Handler(void)
{
	uint32_t u32BuffIndex, u32DMAIndex;
	int32_t i32Temp1;

	uint32_t i;

#if(REC_CHANNELS == 1)
	/* FIXME output stereo and input mono case */
	uint32_t j;
	int16_t i16leftch, i16rightch;
	int32_t i32Avg;
#endif

	int8_t i8RecVol;

	if (!(USBD_GET_EP_INT_FLAG(EPA) & USBD_EPINTSTS_BUFEMPTYIF_Msk))
		return;

	if(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk)
		return;

	/* FIXME UAC microphone + native microphone case */
	if ( g_u32AudioRecordingStage == 0 )
	{
		u32DMAIndex = I2S_GET_RXDMA_CADDR(I2S);
		u32BuffIndex = (uint32_t)&PcmRecBuff[0];
		if ( u32DMAIndex != u32BuffIndex )
		{
			USBD->EP[EPA].EPRSPCTL = USBD_EPRSPCTL_ZEROLEN_Msk;
			return;
		}
		else
		{
//			printf("init!\n");
			g_u32AudioRecordingStage = 1;
		}
	}

	if ( g_u32AudioRecordingStage == 1 )
	{
		UAC_DeviceEnable();

		u32DMAIndex = I2S_GET_RXDMA_CADDR(I2S);
		u32BuffIndex = (uint32_t)&PcmRecBuff[0];
		i32Temp1 = u32DMAIndex - u32BuffIndex;

		/* FIXME: decide when to send recorded data to host */
		/* FIXME: smaller value causes noise, larger value causes long delay */
		if ( i32Temp1 >= g_u32Temp8 )
		{
			g_u32AudioRecordingStage = 2;
//			printf(" %d 0x%x\n", i32Temp1, u32DMAIndex);
		}
		else
		{
//			printf("%d 0x%x\n", i32Temp1, u32DMAIndex);
			USBD->EP[EPA].EPRSPCTL = USBD_EPRSPCTL_ZEROLEN_Msk;
			return;
		}
	}

	if (bRecMuteAdjust)
	{
		bRecMuteAdjust = FALSE;
		if (g_usbd_RecMute)
		{
			/* FIXME: need to ramp down to avoid pop-noise */
			I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);	// Volume Mute
			I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);	// Volume Mute
		}
		else
		{
			/* FIXME: need to ramp up to avoid pop-noise */
			i8RecVol = g_usbd_RecVolume;
			if (i8RecVol <= 0)
			{
				i8RecVol = 15;	// Volume Mute
				I2S_SET_INTERNAL_CODEC(I2S, 0x10, i8RecVol);	//Mute the ADC Left channel volume and set Pre-Gain 0 dB
			}
			else
			{
				i8RecVol--;
				if (i8RecVol > 14)	i8RecVol = 14;	// from 0 dB, 1.6 dB, 3.2 dB, ... , 22.4 dB
				I2S_SET_INTERNAL_CODEC(I2S, 0x10, i8RecVol|0x10);	//Un-Mute the ADC Left channel volume and set Pre-Gain 20 dB
			}
			I2S_SET_INTERNAL_CODEC(I2S, 0x11, i8RecVol);	// from 0 dB, 1.6 dB, 3.2 dB, ... , 22.4 dB
		}
		printf("RM %d\n", g_usbd_RecMute);
	}

	if ( g_u32AudioRecordingStage == 3 )
	{
		if ( s_u32MicChkCnt++ % 2 )
		{
			/* FIXME to detect noise and try to fix */
			u32DMAIndex = I2S_GET_RXDMA_CADDR(I2S);

			u32BuffIndex = (uint32_t)&PcmRecBuff[s_u32Temp0];

			i32Temp1 = u32DMAIndex - u32BuffIndex;

			if ( i32Temp1 < 0 )
				i32Temp1 = RING_BUF_SMPL_SZ + i32Temp1;

			if ( s_u32SpkEnabled == 0 )
			{
				if ( i32Temp1 > g_u32Temp7 )
				{
					if (g_u32RefAPLL != g_u32DecreaseAPLL)
					{
						/* RX too fast, slowing down RX to avoid noise */
						CLK_SET_APLL(g_u32DecreaseAPLL);
						g_u32RefAPLL = g_u32DecreaseAPLL;
						printf("m-%d\n", i32Temp1);
					}
//					else
//					{
//						printf("m- %d\n", i32Temp1);
//					}
				}
				else if ( i32Temp1 < 1024 )
				{
					if (g_u32RefAPLL != g_u32CurrentAPLL)
					{
						/* RX too slow, speeding up RX to avoid noise */
						CLK_SET_APLL(g_u32CurrentAPLL);
						g_u32RefAPLL = g_u32CurrentAPLL;
						printf("m+%d\n", i32Temp1);
					}
//					else
//					{
//						printf("m+ %d\n", i32Temp1);
//					}
				}

				if ( i32Temp1 < g_u32Temp6 )
				{
					if (g_u32RefAPLL != g_u32IncreaseAPLL)
					{
						/* RX too slow, speeding up RX to avoid noise */
						CLK_SET_APLL(g_u32IncreaseAPLL);
						g_u32RefAPLL = g_u32IncreaseAPLL;
						printf("m++%d\n", i32Temp1);
					}
//					else
//					{
//						printf("m++ %d\n", i32Temp1);
//					}
				}
			}

			if ( i32Temp1 < g_u32Temp1 )
			{
				/* FIXME may causes noise */
				printf( "m<%d\n", i32Temp1 );
			}
			else if ( i32Temp1 > g_u32Temp9 )
			{
				/* FIXME may causes noise */
				printf( "m>%d\n", i32Temp1 );
			}
//			else
//			{
				/* Microphone: here is safe range and noise is under controlled. */
//				printf( "%d\n", i32Temp1 );
//			}
		}
	}

	USBD_SET_DMA_READ(ISO_IN_EP_NUM);
	USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_TXPKIEN_Msk);
	USBD_ENABLE_BUS_INT(USBD_BUSINTEN_DMADONEIEN_Msk|USBD_BUSINTEN_SUSPENDIEN_Msk|USBD_BUSINTEN_RSTIEN_Msk|USBD_BUSINTEN_VBUSDETIEN_Msk);
	USBD_SET_DMA_ADDR((uint32_t)&PcmRecBuff2[0]);

	if (s_u32Cnt < g_u32Temp3)
	{
#if(REC_CHANNELS == 1)
		/* FIXME output stereo and input mono case */
		j = 0;
		for (i = 0; i < (g_u32Temp1 >> 1); i+=2)
		{
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			i16leftch = PcmRecBuff[s_u32Temp0++];
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			i16rightch = PcmRecBuff[s_u32Temp0++];
			i32Avg = i16leftch + i16rightch;
#ifdef __ICCARM__                        
			PcmRecBuff2[j++] = __SSAT( i32Avg, 16 );
#else
                        PcmRecBuff2[j++] = __ssat( i32Avg, 16 );
#endif                        
		}
		USBD_SET_DMA_LEN(g_u32Temp1 >> 1);
#else
		/* FIXME output stereo and input stereo case */
		for (i = 0; i < (g_u32Temp1 >> 1); i+=2)
		{
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			/* left channel */
			PcmRecBuff2[i+1] = PcmRecBuff[s_u32Temp0++];
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			/* right channel */
			PcmRecBuff2[i] = PcmRecBuff[s_u32Temp0++];
		}
		USBD_SET_DMA_LEN(g_u32Temp1);
#endif
		s_u32Cnt++;
	}
	else
	{
#if(REC_CHANNELS == 1)
		/* FIXME output stereo and input mono case */
		j = 0;
		for (i = 0; i < (g_u32Temp2 >> 1); i+=2)
		{
			/* left channel */
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			i16leftch = PcmRecBuff[s_u32Temp0++];
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			i16rightch = PcmRecBuff[s_u32Temp0++];
			i32Avg = i16leftch + i16rightch;
			/* right channel */
#ifdef __ICCARM__ 
			PcmRecBuff2[j++] = __SSAT( i32Avg, 16 );
#else                        
			PcmRecBuff2[j++] = __ssat( i32Avg, 16 );
#endif                        
		}
		USBD_SET_DMA_LEN(g_u32Temp2 >> 1);
#else
		/* FIXME output stereo and input stereo case */
		for (i = 0; i < (g_u32Temp2 >> 1); i+=2)
		{
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			/* left channel */
			PcmRecBuff2[i+1] = PcmRecBuff[s_u32Temp0++];
			if (s_u32Temp0 == RING_BUF_SMPL_CNT)
				/* if over flow */
				s_u32Temp0 = 0;
			/* right channel */
			PcmRecBuff2[i] = PcmRecBuff[s_u32Temp0++];
		}
		USBD_SET_DMA_LEN(g_u32Temp2);
#endif
		s_u32Cnt = 0;
	}

	g_u32AudioRecordingStage = 3;
	g_usbd_ShortPacket = 1;
	USBD_ENABLE_DMA();

	if (bRecVolumeAdjust)
	{
		bRecVolumeAdjust = FALSE;
		if ( g_usbd_RecMute ) return;
		/* FIXME: need to ramp up or ramp down to avoid pop-noise */
		i8RecVol = g_usbd_RecVolume;
		if (i8RecVol <= 0)
		{
			i8RecVol = 15;	// Volume Mute
			I2S_SET_INTERNAL_CODEC(I2S, 0x10, i8RecVol);	//Mute the ADC Left channel volume and set Pre-Gain 0 dB
		}
		else
		{
			i8RecVol--;
			if (i8RecVol > 14)	i8RecVol = 14;	// from 0 dB, 1.6 dB, 3.2 dB, ... , 22.4 dB
			I2S_SET_INTERNAL_CODEC(I2S, 0x10, i8RecVol|0x10);	//Un-Mute the ADC Left channel volume and set Pre-Gain 20 dB
		}
		I2S_SET_INTERNAL_CODEC(I2S, 0x11, i8RecVol);	// from 0 dB, 1.6 dB, 3.2 dB, ... , 22.4 dB
		printf("RecV %d\n", g_usbd_RecVolume);
		/* FIXME: we modify min, max, and res for H/W range */
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
    /* EPA ==> ISO IN endpoint, address 1 */
    USBD_SetEpBufAddr(EPA, EPA_BUF_BASE, EPA_BUF_LEN);
    USBD_SET_MAX_PAYLOAD(EPA, EPA_MAX_PKT_SIZE);
    USBD_ConfigEp(EPA, ISO_IN_EP_NUM, USB_EP_CFG_TYPE_ISO, USB_EP_CFG_DIR_IN);

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
		while(1) 
		{
				if (!(USBD->DMACTL & USBD_DMACTL_DMAEN_Msk))
						break;

				if (!USBD_IS_ATTACHED())
						break;
		}
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
								if ((gUsbCmd.wIndex & 0xf) == ISO_IN_EP_NUM) {    /* request to endpoint */
                    USBD_PrepareCtrlIn((uint8_t *)&g_usbd_SampleRate, gUsbCmd.wLength);
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
//									printf("d2h hgetc RR %d\n", g_usbd_SampleRate);
                }								
                else {							
							
										switch ((gUsbCmd.wValue & 0xff00) >> 8)
										{
												case MUTE_CONTROL:
												{
														if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
																USBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMute, 1);
///														printf("  UAC_GET_CUR - MUTE_CONTROL\n");
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
														break;
												}
												case VOLUME_CONTROL:
												{
														if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
														{
																USBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecVolume, 2);
///																printf("  REC - UAC_GET_CUR - VOLUME_CONTROL\n");
														}
														USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
														USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
														break;
												}
#if 0
												case AUTOMATIC_GAIN_CONTROL:
												{
														if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
																USBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMute, 1);											
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
                        if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
												{
                            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMinVolume, 2);
													
///														printf("  REC - UAC_GET_MIN\n");
												}
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
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
                break;
            }

            case UAC_GET_MAX:
            {
                switch ((gUsbCmd.wValue & 0xff00) >> 8)
                {
                    case VOLUME_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
												{
                            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecMaxVolume, 2);
													
///														printf("  REC - UAC_GET_MAX\n");
												}
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
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
                break;
            }

            case UAC_GET_RES:
            {
                switch ((gUsbCmd.wValue & 0xff00) >> 8)
                {
                    case VOLUME_CONTROL:
                    {
                        if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
												{
                            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_RecResVolume, 2);
													
///														printf("  REC - UAC_GET_RES\n");
												}
                        USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_INTKIF_Msk);
                        USBD_ENABLE_CEP_INT(USBD_CEPINTEN_INTKIEN_Msk);
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

								if ((gUsbCmd.wIndex & 0xf) == ISO_IN_EP_NUM) {    /* request to endpoint */
										USBD_CtrlOUT((uint32_t)&g_usbd_SampleRate, gUsbCmd.wLength);
                    /* Status stage */
                    USBD_CLR_CEP_INT_FLAG(USBD_CEPINTSTS_STSDONEIF_Msk);
                    USBD_SET_CEP_STATE(USB_CEPCTL_NAKCLR);
                    USBD_ENABLE_CEP_INT(USBD_CEPINTEN_STSDONEIEN_Msk);
//									printf("h2d set RS %d, %d, ", g_usbd_SampleRate, bSampleRateAdjust);
									bSampleRateAdjust = TRUE;
//									printf("%d\n", bSampleRateAdjust);
                }								
                else {  /* request to interface */					
							
							
										switch ((gUsbCmd.wValue & 0xff00) >> 8)
										{
												case MUTE_CONTROL:
														if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
														{
																USBD_CtrlOUT((uint32_t)&g_usbd_RecMute, gUsbCmd.wLength);
//															printf("h2d set RM %d, %d, ", g_usbd_RecMute, bRecMuteAdjust);
															bRecMuteAdjust = TRUE;
//															printf("%d\n", bRecMuteAdjust);
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
														if (REC_FEATURE_UNITID == ((gUsbCmd.wIndex >> 8) & 0xff))
														{												
																USBD_CtrlOUT((uint32_t)&g_usbd_RecVolume, gUsbCmd.wLength);
//															printf("h2d set RecV %d, %d, ", g_usbd_RecVolume, bRecVolumeAdjust);
															bRecVolumeAdjust = TRUE;
//															printf("%d\n", bRecVolumeAdjust);
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
														/* Setup error, stall the device */
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
	
    if ((gUsbCmd.wIndex & 0xff) == 1)
    { /* Audio Iso IN interface */
        if (u32AltInterface == 1)
        {
//            g_usbd_UsbAudioState = UAC_START_AUDIO_RECORD;
            USBD->EP[EPA].EPRSPCTL = USBD_EPRSPCTL_ZEROLEN_Msk;
					printf("3\n");
//            UAC_DeviceEnable();
						USBD_ENABLE_EP_INT(EPA, USBD_EPINTEN_INTKIEN_Msk);	
        }
        else if (u32AltInterface == 0)
        {
					printf("1\n");
            UAC_DeviceDisable();
            USBD->EP[EPA].EPRSPCTL = USBD_EPRSPCTL_ZEROLEN_Msk;
//            g_usbd_UsbAudioState = UAC_STOP_AUDIO_RECORD;
        }
    }
}

/**   
  * @brief  UAC_DeviceEnable. To enable the device to record audio data.
  * @retval None.
  */
void UAC_DeviceEnable(void)
{
			if ( g_usbd_RecMute )
			{
				I2S_SET_INTERNAL_CODEC(I2S, 0x10, 0x0F);
				I2S_SET_INTERNAL_CODEC(I2S, 0x11, 0x0F);
//				printf("RM1\n");
			}
}


/**   
  * @brief  UAC_DeviceDisable. To disable the device to record audio data.
  * @retval None.
  */
void UAC_DeviceDisable(void)
{
	if (g_u32RefAPLL != g_u32CurrentAPLL)
	{
		CLK_SET_APLL(g_u32CurrentAPLL);
		g_u32RefAPLL = g_u32CurrentAPLL;
	}

			USBD->EP[EPA].EPRSPCTL |= USBD_EPRSPCTL_FLUSH_Msk;
			if((USBD->DMACTL & 0xF) == ISO_IN_EP_NUM)
			{					
            USBD_ResetDMA();
			}
			
      /* Disable record hardware/stop record */
//			memset(PcmRecBuff, 0, sizeof(PcmRecBuff));
			/* FIXME: before stop DMA, wait current address to start address */
//			if ( g_u32AudioRecording )
//			{
//				u32DMAIndex = I2S_GET_RXDMA_CADDR(I2S);
//				u32BuffIndex = (uint32_t)&PcmRecBuff[0];
//				while ( u32DMAIndex != u32BuffIndex )
//					u32DMAIndex = I2S_GET_RXDMA_CADDR(I2S);
//				printf("0x%x\n", u32DMAIndex);
//			}

			/* Disable I2S Rx function */
//			I2S_DISABLE_RXDMA(I2S);
//			I2S_DISABLE_RX(I2S);

			/* Reset some variables */
			g_u32AudioRecordingStage = 0;
			s_u32Temp0 = 0;
			s_u32Cnt = 0;
			s_u32MicChkCnt = 0;
}
