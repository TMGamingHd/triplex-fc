/* SPDX-License-Identifier: MIT
 *
 * The USB device description for the Pico: one CDC ACM serial port. Written against Zephyr's documented USB device (next) API; Zephyr's own sample helper is
 * not used because its options are limited to the samples in Zephyr's tree.
 */
#include "usb_init.h"

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(tfc_usb, LOG_LEVEL_ERR);

USBD_DEVICE_DEFINE(tfc_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_TFC_USB_VID, CONFIG_TFC_USB_PID);

USBD_DESC_LANG_DEFINE(tfc_lang);
USBD_DESC_MANUFACTURER_DEFINE(tfc_mfr, "Triplex FC bench");
USBD_DESC_PRODUCT_DEFINE(tfc_product, "Platform driver and fault injector");
USBD_DESC_SERIAL_NUMBER_DEFINE(tfc_sn);

USBD_DESC_CONFIG_DEFINE(tfc_fs_cfg_desc, "FS Configuration");
USBD_CONFIGURATION_DEFINE(tfc_fs_config, 0 /* bus powered, no remote wakeup */, 50 /* 100 mA, in 2 mA units */, &tfc_fs_cfg_desc);

struct usbd_context *tfc_usbd_init(usbd_msg_cb_t msg_cb)
{
	if (usbd_add_descriptor(&tfc_usbd, &tfc_lang) || usbd_add_descriptor(&tfc_usbd, &tfc_mfr) || usbd_add_descriptor(&tfc_usbd, &tfc_product) ||
	    usbd_add_descriptor(&tfc_usbd, &tfc_sn)) {
		LOG_ERR("cannot add the string descriptors");
		return NULL;
	}
	if (usbd_add_configuration(&tfc_usbd, USBD_SPEED_FS, &tfc_fs_config)) {
		LOG_ERR("cannot add the configuration");
		return NULL;
	}
	if (usbd_register_all_classes(&tfc_usbd, USBD_SPEED_FS, 1, NULL)) {
		LOG_ERR("cannot register the classes");
		return NULL;
	}
	/* a class with more than one interface uses an Interface Association Descriptor: miscellaneous device, common class */
	usbd_device_set_code_triple(&tfc_usbd, USBD_SPEED_FS, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	usbd_self_powered(&tfc_usbd, false);
	if (msg_cb != NULL && usbd_msg_register_cb(&tfc_usbd, msg_cb)) {
		LOG_ERR("cannot register the message callback");
		return NULL;
	}
	if (usbd_init(&tfc_usbd)) {
		LOG_ERR("cannot initialise the USB device");
		return NULL;
	}
	return &tfc_usbd;
}
