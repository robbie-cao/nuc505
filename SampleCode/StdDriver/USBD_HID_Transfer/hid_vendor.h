/******************************************************************************
 * @file     hid_mouse.h
 * @brief    NUC505 USB driver header file
 * @version  2.0.0
 * @date     22, Sep, 2014
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_HID_H__
#define __USBD_HID_H__

/* Define the vendor id and product id */
#define USBD_VID                0x0416
#define USBD_PID                0x5020

/*!<Define HID Class Specific Request */
#define GET_REPORT              0x01
#define GET_IDLE                0x02
#define GET_PROTOCOL            0x03
#define SET_REPORT              0x09
#define SET_IDLE                0x0A
#define SET_PROTOCOL            0x0B

/*!<USB HID Interface Class protocol */
#define HID_NONE                0x00
#define HID_KEYBOARD            0x01
#define HID_MOUSE               0x02

/*!<USB HID Class Report Type */
#define HID_RPT_TYPE_INPUT      0x01
#define HID_RPT_TYPE_OUTPUT     0x02
#define HID_RPT_TYPE_FEATURE    0x03

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define CEP_MAX_PKT_SIZE        64
#define CEP_OTHER_MAX_PKT_SIZE  64

#define EPA_MAX_PKT_SIZE        		512
#define EPA_FULL_MAX_PKT_SIZE  			64
//#define EPA_HS_OTHER_MAX_PKT_SIZE		64
//#define EPA_FS_OTHER_MAX_PKT_SIZE  	512

#define EPB_MAX_PKT_SIZE       		 512
#define EPB_FULL_MAX_PKT_SIZE  			64
//#define EPB_HS_OTHER_MAX_PKT_SIZE  64
//#define EPB_FS_OTHER_MAX_PKT_SIZE  512

#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX_PKT_SIZE
#define EPA_BUF_BASE     (CEP_BUF_BASE + CEP_BUF_LEN)
#define EPA_BUF_LEN     EPA_MAX_PKT_SIZE
#define EPB_BUF_BASE    (EPA_BUF_BASE + EPA_BUF_LEN)
#define EPB_BUF_LEN     EPB_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define HID_IN_EP_NUM			4
#define HID_OUT_EP_NUM		5

#define HID_DEFAULT_INT_IN_INTERVAL	1
#define HID_IS_SELF_POWERED         0
#define HID_IS_REMOTE_WAKEUP        0
#define HID_MAX_POWER               50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */


#define BUFFER_SIZE 0x8000 

#define USBD_MAX_DMA_LEN BUFFER_SIZE

/* Define EP maximum packet size */
#define	HID_MAX_PACKET_SIZE_CTRL        64
#define HID_MAX_PACKET_SIZE_INT_IN			512
#define HID_MAX_PACKET_SIZE_INT_OUT     512

#define HID_MAX_PACKET_SIZE_INT_IN_FS		64
#define HID_MAX_PACKET_SIZE_INT_OUT_FS  64

#define LEN_CONFIG_AND_SUBORDINATE 			(LEN_CONFIG+LEN_INTERFACE+LEN_HID+LEN_ENDPOINT*2)

#define PAGE_SIZE		 256
#define SECTOR_SIZE 4096
void GetDatatForWrite(uint32_t u32Address, uint32_t u32StartPage, uint32_t u32Pages);
void PrepareWriteBuffer(uint32_t *pu32Address, uint32_t u32StartPage, uint32_t u32Pages);
void PrepareReadData(uint32_t *pu32Address, uint32_t u32StartPage, uint32_t u32Pages);
void Erase(uint32_t u32StartSector, uint32_t u32Sectors);

/***************************************************************/
#define HID_CMD_SIGNATURE   0x43444948

/* HID Transfer Commands */
#define HID_CMD_NONE     0x00
#define HID_CMD_ERASE    0x71
#define HID_CMD_READ     0xD2
#define HID_CMD_WRITE    0xC3
#define HID_CMD_TEST     0xB4

#define PAGE_SIZE		 256
#define USB_WRITE_PAGE_UNIT	64

typedef __packed struct {
    uint8_t		u8Cmd;
    uint8_t		u8Size;
    uint32_t	u32Arg1;
    uint32_t	u32Arg2;
    uint32_t	u32Signature;
    uint32_t	u32Checksum;
}CMD_T;

/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/
void HID_InitForHighSpeed(void);
void HID_InitForFullSpeed(void);
void HID_Init(void);
void HID_ClassRequest(void);

void EPA_Handler(void);
void HID_UpdateMouseData(void);

#endif  /* __USBD_HID_H_ */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
