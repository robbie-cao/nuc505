/******************************************************************************
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  V2.0 
 * @date     2015/09/23 07:15 p.m.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

#define __HID__
//#define __MEDIAKEY__
#define __KEYBOARD__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#ifdef __HID__
#ifdef __KEYBOARD__
#define USBD_PID        0x1261
#elif defined __MEDIAKEY__	
#define USBD_PID        0x1262
#endif
#else
#define USBD_PID        0x1260
#endif	

/*!<Define Audio information */
/* FIXME this is the two sampling rate default setting */
/* we can modify supported sampling rate */
/* if need to add more sampling rate or sample size, please contact nuvoton */
#define REC_RATE        48000       /* The record sampling rate. It is 16KHz */
#define REC_DEF_RATE		44100
#define REC_CHANNELS    1                /* FIXME output stereo and input mono case */
//#define REC_CHANNELS    AUDIO_CHANNELS   /* FIXME output stereo and input stereo case */
#define REC_BIT_RATE    16   /* 16-bit data rate */


#define REC_FEATURE_UNITID      0x05
#define PLAY_FEATURE_UNITID     0x06

/* Define Descriptor information */
#if(REC_CHANNELS == 1)
#define REC_CH_CFG     1
#endif
#if(REC_CHANNELS == 2)
#define REC_CH_CFG     3
#endif


#define REC_RATE_LO     (REC_RATE & 0xFF)
#define REC_RATE_MD     ((REC_RATE >> 8) & 0xFF)
#define REC_RATE_HI     ((REC_RATE >> 16) & 0xFF)

#define REC_DEF_RATE_LO     (REC_DEF_RATE & 0xFF)
#define REC_DEF_RATE_MD     ((REC_DEF_RATE >> 8) & 0xFF)
#define REC_DEF_RATE_HI     ((REC_DEF_RATE >> 16) & 0xFF)

/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
//#define UAC_STOP_AUDIO_RECORD           0
//#define UAC_START_AUDIO_RECORD          1
//#define UAC_PROCESSING_AUDIO_RECORD     2
//#define UAC_BUSY_AUDIO_RECORD           3

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF
#define HID_SET_REPORT              0x09
#define HID_SET_IDLE                0x0A
#define HID_SET_PROTOCOL            0x0B

#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02
//#define AUTOMATIC_GAIN_CONTROL			0x07



/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define CEP_MAX_PKT_SIZE        64
#define CEP_OTHER_MAX_PKT_SIZE  64
#define EPA_MAX_PKT_SIZE        96000*REC_CHANNELS*2/1000
#define EPA_OTHER_MAX_PKT_SIZE  96000*REC_CHANNELS*2/1000
#define EPC_MAX_PKT_SIZE        8
#define EPC_OTHER_MAX_PKT_SIZE  8

#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX_PKT_SIZE
#ifdef __HID__
#define EPC_BUF_BASE    (CEP_BUF_BASE + CEP_BUF_LEN)
#define EPC_BUF_LEN     EPC_MAX_PKT_SIZE
#define EPA_BUF_BASE    (EPC_BUF_BASE + EPC_BUF_LEN)
#define EPA_BUF_LEN     (0x800 - CEP_BUF_LEN - EPC_BUF_LEN)
#else
#define EPA_BUF_BASE    (CEP_BUF_BASE + CEP_BUF_LEN)
#define EPA_BUF_LEN     (0x800 - CEP_BUF_LEN)

#endif

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM    0x01
#define ISO_OUT_EP_NUM   0x02
#define HID_IN_EP_NUM    0x03

/* FIXME: ring buffer size as sample count */
/* if sample size is 2 bytes, the ring buffer sample count is 2048 samples */
#define RING_BUF_SMPL_CNT2   1024

/* if sample size if 3 or 4 bytes, the ring buffer sample count is 1024 samples */
#define RING_BUF_SMPL_CNT   1024

/* FIXME: ring buffer size as byte and * 4 means sample size is supported by 2, 3, or 4 bytes */
#define RING_BUF_SMPL_SZ    RING_BUF_SMPL_CNT * 2

/* FIXME: check-poing ring buffer sample byte size */
#define RING_BUF_SMPL_SZ_CP   (RING_BUF_SMPL_SZ / 2)

/*!<Define HID Class Specific Request */
#define GET_REPORT              0x01
#define GET_IDLE                0x02
#define GET_PROTOCOL            0x03
#define SET_REPORT              0x09
#define SET_IDLE                0x0A
#define SET_PROTOCOL            0x0B

#ifdef __HID__
#ifdef __MEDIAKEY__
#define HID_CTRL_MUTE        0x01
#define HID_CTRL_VOLUME_INC  0x02
#define HID_CTRL_VOLUME_DEC  0x04

#define HID_CTRL_EJECT       0x08
#define HID_CTRL_PLAY        0x01
#define HID_CTRL_STOP        0x02
#define HID_CTRL_PAUSE       0x04
#define HID_CTRL_NEXT        0x08
#define HID_CTRL_PREVIOUS    0x10
#define HID_CTRL_RECORD      0x20
#define HID_CTRL_REWIND      0x40
#define HID_CTRL_FF          0x80
#endif
#endif

/*-------------------------------------------------------------*/
void UAC_DeviceEnable(void);
void UAC_DeviceDisable(void);


/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(uint32_t u32AltInterface);

void EPA_Handler(void);
void EPC_Handler(void);
void DMA_Init(void);

void Audio_MicrophoneSpeedAdjust(void);

void USBD_CtrlOUT(uint32_t u32addr, uint32_t u32Len);

#endif  /* __USBD_UAC_H_ */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
