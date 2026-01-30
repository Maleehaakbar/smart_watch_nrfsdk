#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, CONFIG_ZSW_APP_LOG_LEVEL);

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM (hours:min:sec:millisecond), buff needs to return , it can't be local, declare it static */
	uint32_t now = k_uptime_get();     /*give time in ms*/
	unsigned int ms = now % MSEC_PER_SEC;  /*extract ms*/
	unsigned int s;
	unsigned int min;
	unsigned int h;
	/*timestamp is showing elapsed (uptime) time since the log source started running — not wall-clock time, conversion avoid floating-point math*/
	now /= MSEC_PER_SEC;   /*convert ms to s*/
	s = now % 60U;         /*extract seconds*/
	now /= 60U;            /*convert s to min*/
	min = now % 60U;       /*extract min*/
	now /= 60U;			   /*convert min to h*/
	h = now;               /*get hour*/

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}


int read_mpu6050(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP,
					&temperature);
	}
	if (rc == 0) {
		LOG_INF("[%s]:%g Cel\n"
		       "  accel %f %f %f m/s/s\n"
		       "  gyro  %f %f %f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		printf("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

int init_mpu6050(const struct device *mpu_sensor)
{
	if (!device_is_ready(mpu_sensor)) {
		printf("Device %s is not ready\n", mpu_sensor->name);
		return -ENODEV;
	}

	return 0;
}

