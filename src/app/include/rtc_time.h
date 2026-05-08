#ifndef RTC_TIME_H
#define RTC_TIME_H

#include <zephyr/drivers/rtc.h>

extern struct rtc_time tm;

int ds3231_rtc_init();
int rtc_get_date_time();

#endif