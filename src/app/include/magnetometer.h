#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <zephyr/device.h>

int init_magnetometer();
int read_magnetometer_data(float *x, float *y, float *z);


int start_magn_calibration();
void zsw_magnetometer_stop_calibration(void);
int read_magn_with_calibration();




#endif 