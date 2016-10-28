/**************************************************************************//**
 * @file     descriptors.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    NUC505 USBD driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DESCRIPTORS_C__
#define __DESCRIPTORS_C__

/*!<Includes */
#include "NUC505Series.h"
#include "hid_vendor.h"

/*!<USB HID Report Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t HID_ReportDescriptor[] = {
#else
__align(4) uint8_t HID_ReportDescriptor[] = {
#endif
		0x06, 0x06, 0xFF,	/* USAGE_PAGE (Vendor Defined)*/
		0x09, 0x01, 			/* USAGE (0x01)*/
		0xA1, 0x01,				/* COLLECTION (Application)*/
		0x15, 0x00,				/*     LOGICAL_MINIMUM (0)*/
		0x26, 0xFF, 0x00,	/*     LOGICAL_MAXIMUM (255)*/
		0x75, 0x08,				/*     REPORT_SIZE (8)*/
		0x96, 
		HID_MAX_PACKET_SIZE_INT_IN & 0x00FF,           /*     REPORT_COUNT*/
		(HID_MAX_PACKET_SIZE_INT_IN & 0xFF00) >> 8, 	
		0x09, 0x01,
		0x81, 0x02,				/*     INPUT (Data,Var,Abs)*/
		0x96,  	
		HID_MAX_PACKET_SIZE_INT_OUT & 0x00FF,           /*     REPORT_COUNT*/
		(HID_MAX_PACKET_SIZE_INT_OUT & 0xFF00) >> 8, 	 	 	
		0x09, 0x01,
		0x91, 0x02,				/*   OUTPUT (Data,Var,Abs)*/
		0x95, 0x08,				/*     REPORT_COUNT (8) */
		0x09, 0x01,
		0xB1, 0x02,				/*     FEATURE */
		0xC0							/* END_COLLECTION*/
};

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t HID_ReportDescriptorFS[] = {
#else
__align(4) uint8_t HID_ReportDescriptorFS[] = {
#endif
		0x06, 0x06, 0xFF,	/* USAGE_PAGE (Vendor Defined)*/
		0x09, 0x01, 			/* USAGE (0x01)*/
		0xA1, 0x01,				/* COLLECTION (Application)*/
		0x15, 0x00,				/*     LOGICAL_MINIMUM (0)*/
		0x26, 0xFF, 0x00,	/*     LOGICAL_MAXIMUM (255)*/
		0x75, 0x08,				/*     REPORT_SIZE (8)*/
		0x96, 
		HID_MAX_PACKET_SIZE_INT_IN_FS & 0x00FF,           /*     REPORT_COUNT*/
		(HID_MAX_PACKET_SIZE_INT_IN_FS & 0xFF00) >> 8, 	
		0x09, 0x01,
		0x81, 0x02,				/*     INPUT (Data,Var,Abs)*/
		0x96,  	
		HID_MAX_PACKET_SIZE_INT_OUT_FS & 0x00FF,           /*     REPORT_COUNT*/
		(HID_MAX_PACKET_SIZE_INT_OUT_FS & 0xFF00) >> 8, 	 	 	
		0x09, 0x01,
		0x91, 0x02,				/*   OUTPUT (Data,Var,Abs)*/
		0x95, 0x08,				/*     REPORT_COUNT (8) */
		0x09, 0x01,
		0xB1, 0x02,				/*     FEATURE */
		0xC0							/* END_COLLECTION*/
};

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8DeviceDescriptor[] = {
#else
__align(4) uint8_t gu8DeviceDescriptor[] = {
#endif
		LEN_DEVICE,		    		/* bLength 				*/
		DESC_DEVICE,	    		/* bDescriptorType 		*/
		0x00, 0x02,		    		/* bcdUSB 				*/
		0x00,			    		/* bDeviceClass 		*/
		0x00,			    		/* bDeviceSubClass 		*/
		0x00,			    		/* bDeviceProtocol 		*/
		HID_MAX_PACKET_SIZE_CTRL,	/* bMaxPacketSize0 	    */
		USBD_VID & 0x00FF,			/* Vendor ID 			*/
		(USBD_VID & 0xFF00) >> 8, 
		USBD_PID & 0x00FF,			/* Product ID 			*/
		(USBD_PID & 0xFF00) >> 8, 
		0x00, 0x00,		    		/* bcdDevice 			*/
		0x01,			    		/* iManufacture 		*/
		0x02,			    		/* iProduct				*/
		0x00,						/* iSerialNumber - no serial */
		0x01			    		/* bNumConfigurations 	*/
};

/*!<USB Qualifier Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8QualifierDescriptor[] = {
#else
__align(4) uint8_t gu8QualifierDescriptor[] = {
#endif
    LEN_QUALIFIER,  /* bLength */
    DESC_QUALIFIER, /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_OTHER_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00
};

/*!<USB Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ConfigDescriptor[] = {
#else
__align(4) uint8_t gu8ConfigDescriptor[] = {
#endif
		LEN_CONFIG,		/* bLength */
		DESC_CONFIG,	/* bDescriptorType */
		/* wTotalLength */
		LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
		(LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8,
		0x01,			/* bNumInterfaces */
		0x01,			/* bConfigurationValue */
		0x00,			/* iConfiguration */
		0xC0,			/* bmAttributes */
		0x32,			/* MaxPower */

		LEN_INTERFACE,
		DESC_INTERFACE,
		0x00,           /* bInterfaceNumber */
		0x00,           /* bAlternateSetting */
		0x02,           /* bNumEndpoints */
		0x03,           /* bInterfaceClass */
		0x00,           /* bInterfaceSubClass */
		0x00,   		/* bInterfaceProtocol */
		0x00,           /* iInterface */

		LEN_HID,        /* bLength */
		DESC_HID,       /* bDescriptorType */
		0x10, 0x01,     /* bcdHID */
		0x00,           /* bCountryCode */
		0x01,           /* bNumDescriptor */
		DESC_HID_RPT,   /* bDescriptorType */
		sizeof(HID_ReportDescriptor) & 0x00FF,            /* wDescriptorLen */
		(sizeof(HID_ReportDescriptor) & 0xFF00) >> 8,
		
		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_IN_EP_NUM | EP_INPUT,                           /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_IN & 0x00FF,            /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_IN & 0xFF00) >> 8,         
		HID_DEFAULT_INT_IN_INTERVAL,                    /* bInterval */

		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_OUT_EP_NUM,                                 /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_OUT & 0x00FF,           /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_OUT & 0xFF00) >> 8,
		HID_DEFAULT_INT_IN_INTERVAL                     /* bInterval */
};


/*!<USB Other Speed Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8OtherConfigDescriptorHS[] = {
#else
__align(4) uint8_t gu8OtherConfigDescriptorHS[] = {
#endif
		LEN_CONFIG,		/* bLength */
		DESC_OTHERSPEED,	/* bDescriptorType */
		/* wTotalLength */
		LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
		(LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8,
		0x01,			/* bNumInterfaces */
		0x01,			/* bConfigurationValue */
		0x00,			/* iConfiguration */
		0xC0,			/* bmAttributes */
		0x32,			/* MaxPower */

		LEN_INTERFACE,
		DESC_INTERFACE,
		0x00,           /* bInterfaceNumber */
		0x00,           /* bAlternateSetting */
		0x02,           /* bNumEndpoints */
		0x03,           /* bInterfaceClass */
		0x00,           /* bInterfaceSubClass */
		0x00,   		/* bInterfaceProtocol */
		0x00,           /* iInterface */

		LEN_HID,        /* bLength */
		DESC_HID,       /* bDescriptorType */
		0x10, 0x01,     /* bcdHID */
		0x00,           /* bCountryCode */
		0x01,           /* bNumDescriptor */
		DESC_HID_RPT,   /* bDescriptorType */
		sizeof(HID_ReportDescriptor) & 0x00FF,            /* wDescriptorLen */
		(sizeof(HID_ReportDescriptor) & 0xFF00) >> 8,
		
		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_IN_EP_NUM | EP_INPUT,                           /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_IN_FS & 0x00FF,            /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_IN_FS & 0xFF00) >> 8,         
		HID_DEFAULT_INT_IN_INTERVAL,                    /* bInterval */

		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_OUT_EP_NUM,                                 /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_OUT_FS & 0x00FF,           /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_OUT_FS & 0xFF00) >> 8,
		HID_DEFAULT_INT_IN_INTERVAL                     /* bInterval */
};


/*!<USB Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ConfigDescriptorFS[] = {
#else
__align(4) uint8_t gu8ConfigDescriptorFS[] = {
#endif
		LEN_CONFIG,		/* bLength */
		DESC_CONFIG,	/* bDescriptorType */
		/* wTotalLength */
		LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
		(LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8,
		0x01,			/* bNumInterfaces */
		0x01,			/* bConfigurationValue */
		0x00,			/* iConfiguration */
		0xC0,			/* bmAttributes */
		0x32,			/* MaxPower */

		LEN_INTERFACE,
		DESC_INTERFACE,
		0x00,           /* bInterfaceNumber */
		0x00,           /* bAlternateSetting */
		0x02,           /* bNumEndpoints */
		0x03,           /* bInterfaceClass */
		0x00,           /* bInterfaceSubClass */
		0x00,   		/* bInterfaceProtocol */
		0x00,           /* iInterface */

		LEN_HID,        /* bLength */
		DESC_HID,       /* bDescriptorType */
		0x10, 0x01,     /* bcdHID */
		0x00,           /* bCountryCode */
		0x01,           /* bNumDescriptor */
		DESC_HID_RPT,   /* bDescriptorType */
		sizeof(HID_ReportDescriptor) & 0x00FF,            /* wDescriptorLen */
		(sizeof(HID_ReportDescriptor) & 0xFF00) >> 8,
		
		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_IN_EP_NUM | EP_INPUT,                           /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_IN_FS & 0x00FF,            /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_IN_FS & 0xFF00) >> 8,         
		HID_DEFAULT_INT_IN_INTERVAL,                    /* bInterval */

		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_OUT_EP_NUM,                                 /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_OUT_FS & 0x00FF,           /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_OUT_FS & 0xFF00) >> 8,
		HID_DEFAULT_INT_IN_INTERVAL                     /* bInterval */
};


/*!<USB Other Speed Configure Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8OtherConfigDescriptorFS[] = {
#else
__align(4) uint8_t gu8OtherConfigDescriptorFS[] = {
#endif
		LEN_CONFIG,		/* bLength */
		DESC_OTHERSPEED,	/* bDescriptorType */
		/* wTotalLength */
		LEN_CONFIG_AND_SUBORDINATE & 0x00FF,
		(LEN_CONFIG_AND_SUBORDINATE & 0xFF00) >> 8,
		0x01,			/* bNumInterfaces */
		0x01,			/* bConfigurationValue */
		0x00,			/* iConfiguration */
		0xC0,			/* bmAttributes */
		0x32,			/* MaxPower */

		LEN_INTERFACE,
		DESC_INTERFACE,
		0x00,           /* bInterfaceNumber */
		0x00,           /* bAlternateSetting */
		0x02,           /* bNumEndpoints */
		0x03,           /* bInterfaceClass */
		0x00,           /* bInterfaceSubClass */
		0x00,   		/* bInterfaceProtocol */
		0x00,           /* iInterface */

		LEN_HID,        /* bLength */
		DESC_HID,       /* bDescriptorType */
		0x10, 0x01,     /* bcdHID */
		0x00,           /* bCountryCode */
		0x01,           /* bNumDescriptor */
		DESC_HID_RPT,   /* bDescriptorType */
		sizeof(HID_ReportDescriptor) & 0x00FF,            /* wDescriptorLen */
		(sizeof(HID_ReportDescriptor) & 0xFF00) >> 8,
		
		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_IN_EP_NUM | EP_INPUT,                           /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_IN & 0x00FF,            /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_IN & 0xFF00) >> 8,         
		HID_DEFAULT_INT_IN_INTERVAL,                    /* bInterval */

		LEN_ENDPOINT,                                   /* bLength */
		DESC_ENDPOINT,                                  /* bDescriptorType */
		HID_OUT_EP_NUM,                                 /* bEndpointAddress */
		EP_INT,                                         /* bmAttributes */
		HID_MAX_PACKET_SIZE_INT_OUT & 0x00FF,           /* wMaxPacketSize */
		(HID_MAX_PACKET_SIZE_INT_OUT & 0xFF00) >> 8,
		HID_DEFAULT_INT_IN_INTERVAL                     /* bInterval */
};


/*!<USB Language String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8StringLang[4] = {
#else
__align(4) uint8_t gu8StringLang[4] = {
#endif
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8VendorStringDesc[] = {
#else
__align(4) uint8_t gu8VendorStringDesc[] = {
#endif
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t gu8ProductStringDesc[] = {
#else
__align(4) uint8_t gu8ProductStringDesc[] = {
#endif
    26,
    DESC_STRING,
		'H', 0, 'I', 0, 'D', 0, ' ', 0, 'T', 0, 'r', 0, 'a', 0, 'n', 0, 's', 0, 'f', 0, 'e', 0, 'r', 0
};

uint8_t *gpu8UsbString[4] = {
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    NULL,
};

uint8_t *gu8UsbHidReport[3] = {
    HID_ReportDescriptor,
    NULL,
    NULL,
};

uint32_t gu32UsbHidReportLen[3] = {
    sizeof(HID_ReportDescriptor),
    0,
    0,
};

S_USBD_INFO_T gsInfo = {
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gu8QualifierDescriptor,
		gu8ConfigDescriptorFS,
    gu8OtherConfigDescriptorHS,
		gu8OtherConfigDescriptorFS,
		NULL,
    gu8UsbHidReport,
    gu32UsbHidReportLen,
};


#endif  /* __DESCRIPTORS_C__ */
