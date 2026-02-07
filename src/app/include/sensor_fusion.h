#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

/*store data from AHRS algo for use in other apps/algorihms*/
typedef struct sensor_fusion {
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
} sensor_fusion_t;

typedef struct quat {
    float w;
    float x;
    float y;
    float z;
} quat_t;

void sensor_fusion_init();
void print_logs();

#endif