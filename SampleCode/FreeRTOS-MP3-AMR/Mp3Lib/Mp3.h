/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __MP3_H__
#define __MP3_H__

//#include "Platform.h"
#include "AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define	MP3_MAJOR_NUM 3
#define	MP3_MINOR_NUM 30
#define	MP3_BUILD_NUM 1
#define MP3_VERSION_NUM	_SYSINFRA_VERSION(MP3_MAJOR_NUM, MP3_MINOR_NUM, MP3_BUILD_NUM)

#define MP3_DECODE_SAMPLE_PER_FRAME		(32)		// counts per frame after decoding
//#define MP3_DECODE_WORK_BUF_SIZE		0x1944		// CH_STEREO_MIXED_MONO	// BUFFER_SIZE==1547
//#define MP3_DECODE_WORK_BUF_SIZE		0x2204		// CH_STEREO_DEFAULT 	// BUFFER_SIZE==1547
//#if defined(__NUC505__)
#define MP3_DECODE_WORK_BUF_SIZE		0x23f8		// CH_STEREO_DEFAULT 	// BUFFER_SIZE==2048
//#else
//#define MP3_DECODE_WORK_BUF_SIZE		0x1b38		// CH_STEREO_MIXED_MONO	// BUFFER_SIZE==2048
//#endif
#define MP3_DECODE_TEMP_BUF_SIZE		0x1a8		// bytes
	
typedef void (*PFN_AUDIO_SUBFRAME_CALLBACK)(INT16 *pi16SmplBuf, INT16 i16SmplCnt);
typedef void (*PFN_AUDIO_SAMPLERATE_CALLBACK)(UINT16 u16SampleRate);
	

/******************************************************************************************
 * Description:
 * 	Version control API.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
UINT32 Mp3_GetVersion
	(void
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
UINT32 Mp3_DecodeInitiate(
	UINT8 *pu8WorkBuf,
	UINT8 *pu8TempBuf,
	UINT32 u32StartAddr,
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
INT32 Mp3_DecodeProcess(
	UINT8 *pu8WorkBuf,
	UINT8 *pu8TempBuf,
	PINT16 pi16DecodedPcmBuf,
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,
	PFN_AUDIO_USREVENT pfnUserEventCallback
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
BOOL Mp3_DecodeIsEnd(
	UINT8 *pu8WorkBuf
	);

//---------------------------------------------------------------------------
// Set a callback function for MP3 library to callback as sample rate information is decoded.
//---------------------------------------------------------------------------
void Mp3_EnableSampleRateCallback(UINT8 *pu8WorkBuf, PFN_AUDIO_SAMPLERATE_CALLBACK pfnSampleRateCallback);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef __MP3_H__


