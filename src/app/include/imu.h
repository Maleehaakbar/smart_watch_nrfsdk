#ifndef IMU_H
#define IMU_H

int init_mpu6050(const struct device *mpu_sensor);
int read_mpu6050(const struct device *dev);

#endif