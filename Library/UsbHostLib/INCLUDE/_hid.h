#ifndef _HID_DEF_H_
#define _HID_DEF_H_

#include "usbh_hid.h"

/// @cond HIDDEN_SYMBOLS

#define HID_DEBUG


#define USB_TIMEOUT 			10000 

#define HID_REPORT_GET 			0x01
#define HID_REPORT_SET 			0x09

#define HID_GET_IDLE   			0x02
#define HID_SET_IDLE   			0x0A
  
#define HID_RT_INPUT 			0x01
#define HID_RT_OUTPUT 			0x02
#define HID_RT_FEATURE 			0x03

#define USB_ENDPOINT_IN			USB_DIR_IN
#define USB_ENDPOINT_OUT		USB_DIR_OUT


#define CONFIG_HID_MAX_DEV		1
#define HID_MAX_BUFFER_SIZE		2048	/* 64(min.) ~ 4096(max.) */
#define HID_CONTROL_FIFO_SIZE	256
#define HID_OUTPUT_FIFO_SIZE	64

#ifdef HID_DEBUG
#define HID_DBGMSG		printf
#else
#define HID_DBGMSG(...)
#endif


/*
 * USB HID (Human Interface Device) interface class code
 */

#define USB_INTERFACE_CLASS_HID			3

/*
 * USB HID interface subclass and protocol codes
 */

#define USB_INTERFACE_SUBCLASS_BOOT		1
#define USB_INTERFACE_PROTOCOL_KEYBOARD	1
#define USB_INTERFACE_PROTOCOL_MOUSE	2

/*
 * HID class requests
 */

#define HID_REQ_GET_REPORT				0x01
#define HID_REQ_GET_IDLE				0x02
#define HID_REQ_GET_PROTOCOL			0x03
#define HID_REQ_SET_REPORT				0x09
#define HID_REQ_SET_IDLE				0x0A
#define HID_REQ_SET_PROTOCOL			0x0B

/*
 * HID class descriptor types
 */

#define HID_DT_HID						(USB_TYPE_CLASS | 0x01)
#define HID_DT_REPORT					(USB_TYPE_CLASS | 0x02)
#define HID_DT_PHYSICAL					(USB_TYPE_CLASS | 0x03)

#define HID_MAX_DESCRIPTOR_SIZE			4096

#define HID_MAX_FIELDS 					128


/*
 * USB-specific HID struct, to be pointed to
 * from struct hid_device->driver_data
 */

typedef struct usbhid_device {
	int					handle;
	USB_DEV_T 			*udev;
	int 				ifnum;          /* USB interface number */
	URB_T				*urbin;         /* Input URB */
	URB_T				*urbout;        /* Output URB */
	uint8_t				inbuf[HID_MAX_BUFFER_SIZE];     /* Input buffer */
	HID_INT_READ_FUNC   *read_func;
	HID_INT_WRITE_FUNC  *write_func;
}	HID_DEV_T;



extern EP_INFO_T *hid_get_ep_info(USB_DEV_T *dev, int ifnum, uint16_t endpoint);
extern HID_DEV_T *find_hid_deivce_by_udev(USB_DEV_T *udev);
extern HID_DEV_T *find_hid_deivce_by_handle(int handle);

extern hid_return hid_prepare_parser(HIDInterface* const hidif);
extern hid_return  usbh_hid_find_device(HIDInterface* hidif, HIDInterfaceMatcher *matcher);
extern void ResetParser(HIDParser* pParser);
extern int HIDParse(HIDParser* pParser, HIDData* pData);
extern hid_return hid_find_object(HIDInterface* hidif, int path[], uint32_t depth);
extern uint8_t* GetReportOffset(HIDParser* pParser, const uint8_t ReportID, const uint8_t ReportType);

/// @endcond HIDDEN_SYMBOLS

#endif /* _HID_DEF_H_ */
