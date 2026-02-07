#ifndef IMU_H
#define IMU_H

int init_mpu6050();
int read_attribute();
int read_mpu6050_gyro(float *x, float*y, float *z);
int read_mpu6050_accel(float *x, float*y, float *z);

#endif