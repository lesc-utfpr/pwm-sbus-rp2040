#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// Set USB port mode to device
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUD_ENDPOINT0_SIZE  64

// Enable the HID class driver
#define CFG_TUD_HID             1

// Size of the HID buffer
#define CFG_TUD_HID_EP_BUFSIZE  16

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */