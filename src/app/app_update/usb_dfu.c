
#include "usb_common.h"

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/usbd.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cdc_acm_dfu, LOG_LEVEL_INF);


static struct usbd_context *sample_usbd;

 /*Create VBUS callback function . ctx (usb device data to avoid global variables or mange multiple usb device contexts)*/
 void sample_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
    LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

    if (usbd_can_detect_vbus(ctx)) {
        if (msg->type == USBD_MSG_VBUS_READY) {
            if (usbd_enable(ctx)) {
                LOG_ERR("Failed to enable device support");
            }
        }

        if (msg->type == USBD_MSG_VBUS_REMOVED) {
            if (usbd_disable(ctx)) {
                LOG_ERR("Failed to disable device support");
            }
        }
    }
}

/*enable usb stack in application level for dfu via mcumgr*/
 int enable_usb_device_next(void)
{
	int err;

	/*init usb device*/
	sample_usbd = sample_usbd_init_device(sample_msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	/*enablle usb device if vbus not detected*/
	if (!usbd_can_detect_vbus(sample_usbd)) {
		err = usbd_enable(sample_usbd);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");
	LOG_INF("firmware updated, new version upload");

	return 0;
}



