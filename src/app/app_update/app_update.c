#include "app_update.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(app_update, LOG_LEVEL_INF);

const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);


int app_update_init(void)
{
    int ret;
        if (!device_is_ready(uart_dev))
        {
            LOG_ERR("CDC ACM device not ready");
            return -ENODEV;
        }

        ret = enable_usb_device_next();
        if (ret != 0)
        {
            LOG_ERR("Failed to enable USB device support");
            return ret;
        }
    

    LOG_INF("USB init updated 2.0\n");

       ret =  ble_init();
        if (ret != 0)
        {
            LOG_ERR("Failed to initialize BLE support");
            return ret;
        }

    return 0;
}