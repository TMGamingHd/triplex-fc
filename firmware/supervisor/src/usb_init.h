/* SPDX-License-Identifier: MIT */
#ifndef TFC_SUP_USB_INIT_H
#define TFC_SUP_USB_INIT_H

#include <zephyr/usb/usbd.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Describe the device (one CDC ACM serial port, full speed, bus powered), register `msg_cb`, and initialise the USB device stack. Call usbd_enable() on the result
 * (or wait for VBUS if usbd_can_detect_vbus()). NULL if anything failed. */
struct usbd_context *tfc_usbd_init(usbd_msg_cb_t msg_cb);

#ifdef __cplusplus
}
#endif

#endif
