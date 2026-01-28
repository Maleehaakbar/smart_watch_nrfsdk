#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <zephyr/device.h>

int init_magnetometer(const struct device *sensor);
int read_sensor(const struct device *sensor);
int start_magn_calibration(const struct device *sensor);
void zsw_magnetometer_stop_calibration(void);
int read_magn_with_calibration(const struct device *sensor);



#endif 