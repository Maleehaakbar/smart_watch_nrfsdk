#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rtc_time, CONFIG_ZSW_APP_LOG_LEVEL);

const struct device *const rtc = DEVICE_DT_GET(DT_ALIAS(rtc));


int ds3231_rtc_init()
{
    /* Check if the RTC is ready */
	if (!device_is_ready(rtc)) {
		LOG_ERR("Device is not ready\n");
		return -ENODEV;;
	}

   int ret = 0;
	struct rtc_time tm = {
		.tm_year = 2026 - 1900,
		.tm_mon = 11 - 1,
		.tm_mday = 17,
		.tm_hour = 1,
		.tm_min = 57,
		.tm_sec = 0,
	};

	ret = rtc_set_time(rtc, &tm);
	if (ret < 0) {
		LOG_ERR("Cannot write date time: %d\n", ret);
		return ret;
	}

	return ret;
}

int rtc_get_date_time()
{
    int ret = 0;
	struct rtc_time tm;

	ret = rtc_get_time(rtc, &tm);
	if (ret < 0) {
		LOG_ERR("Cannot read date time: %d\n", ret);
		return ret;
	}
	
	LOG_INF("RTC date and time: %04d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900,
	       tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	return ret;
}


//TODO: check below warnings

/*home/mod/ncs/v3.0.2/zephyr/drivers/rtc/rtc_ds3231.c: In function 'rtc_ds3231_init':
/home/mod/ncs/v3.0.2/zephyr/drivers/rtc/rtc_ds3231.c:808:33: warning: unused variable 'data' [-Wunused-variable]
  808 |         struct rtc_ds3231_data *data = dev->data;
      |                                 ^~~~
/home/mod/ncs/v3.0.2/zephyr/drivers/rtc/rtc_ds3231.c: At top level:
/home/mod/ncs/v3.0.2/zephyr/drivers/rtc/rtc_ds3231.c:195:12: warning: 'rtc_ds3231_get_ctrl_sts' defined but not used [-Wunused-function]
  195 | static int rtc_ds3231_get_ctrl_sts(const struct device *dev, uint8_t *buf)*/