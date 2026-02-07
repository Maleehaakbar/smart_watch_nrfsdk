#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/sys/atomic.h>
#include <errno.h>

#include "../../../ext_libraries/fusion/Fusion/Fusion.h"  // src/app/src
#include "../../../ext_libraries/fusion/Fusion/FusionCompass.h"
#include "imu.h"
#include "magnetometer.h"
#include <string.h>
#include "sensor_fusion.h"

/*replace with actual sample rate ,100 in libraray,8000 for gyro,1000 for accel
loop execution rate in main.c currently is 10ms , so sample rate =100
*/
#define SAMPLE_RATE_HZ (100) 
//#define M_PI        3.14159265358979323846
#define SENSOR_GF       9.806650

LOG_MODULE_REGISTER(sensor_fusion, CONFIG_ZSW_APP_LOG_LEVEL);

/*prototype of work handler*/
static void sensor_fusion_timeout(struct k_work *item);

/*define a work queue to run prediocally(function name, function handler)*/
K_WORK_DELAYABLE_DEFINE(sensor_fusion_timer, sensor_fusion_timeout); 

static int32_t previousTimestamp;
static float last_delta_time_s = 0.0f;

sensor_fusion_t fusion_read;
quat_t quat_read;


/*1.0 on the diagonal + 0.0 elsewhere = identity matrix = no correction applied*/
// Define calibration (replace with actual calibration data if available)
static const FusionMatrix gyroscopeMisalignment = {.element.xx = 1.0f,
                                                   .element.xy = 0.0f,
                                                   .element.xz = 0.0f,
                                                   .element.yx = 0.0f,
                                                   .element.yy = 1.0f,
                                                   .element.yz = 0.0f,
                                                   .element.zx = 0.0f,
                                                   .element.zy = 0.0f,
                                                   .element.zz = 1.0f
                                                  };
static const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
static const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};
static const FusionMatrix accelerometerMisalignment = {.element.xx = 1.0f,
                                                       .element.xy = 0.0f,
                                                       .element.xz = 0.0f,
                                                       .element.yx = 0.0f,
                                                       .element.yy = 1.0f,
                                                       .element.yz = 0.0f,
                                                       .element.zx = 0.0f,
                                                       .element.zy = 0.0f,
                                                       .element.zz = 1.0f
                                                      };
static const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
static const FusionVector accelerometerOffset = {{0.0f, 0.0f, 0.0f}};

#ifdef CONFIG_ENABLE_QMC5883L_SENSOR
static const FusionMatrix softIronMatrix = {.element.xx = 1.0f,
                                            .element.xy = 0.0f,
                                            .element.xz = 0.0f,
                                            .element.yx = 0.0f,
                                            .element.yy = 1.0f,
                                            .element.yz = 0.0f,
                                            .element.zx = 0.0f,
                                            .element.zy = 0.0f,
                                            .element.zz = 1.0f
                                           };
static const FusionVector hardIronOffset = {{0.0f, 0.0f, 0.0f}};
#endif

// Initialise structures, used in sensor_fusion_init
FusionBias bias;
FusionAhrs ahrs;

/*work handler*/
static void sensor_fusion_timeout(struct k_work *item)
{
    int ret = 0;

    FusionVector gyroscope;
    FusionVector accelerometer;

    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    FusionVector magnetometer;
    #endif

    uint32_t start = k_uptime_get_32();
    ret = read_mpu6050_gyro(&gyroscope.axis.x, &gyroscope.axis.y, &gyroscope.axis.z);
    if (ret!=0)
    {
        LOG_ERR("imu_fetch_gyro err: %d", ret);
    }

      // Convert from rad/s to deg/s
    gyroscope.axis.x = gyroscope.axis.x * (180.0F / (float)M_PI);
    gyroscope.axis.y = gyroscope.axis.y * (180.0F / (float)M_PI);
    gyroscope.axis.z = gyroscope.axis.z * (180.0F / (float)M_PI);


    ret = read_mpu6050_accel(&accelerometer.axis.x, &accelerometer.axis.y, &accelerometer.axis.z);

    if (ret!=0)
    {
        LOG_ERR("imu_fetch_accel err: %d", ret);
    }

     // IMU driver converts to m/s2 by multiplying to 10, convert back to g-force
    accelerometer.axis.x /= (float)SENSOR_GF;
    accelerometer.axis.y /= (float)SENSOR_GF;
    accelerometer.axis.z /= (float)SENSOR_GF;

    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    ret = read_magnetometer_data(&magnetometer.axis.x, &magnetometer.axis.y, &magnetometer.axis.z);

    if (ret!=0)
    {
        LOG_ERR("fetch_mag err: %d", ret);
    }
    #endif

    gyroscope = FusionModelInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionModelInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity,
                                            accelerometerOffset);
    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);
    #endif

    // Update gyroscope offset correction algorithm
    gyroscope = FusionBiasUpdate(&bias, gyroscope);

    //calculates elapsed time between gyro samples, converts it to seconds, filters out invalid values, 
   // and keeps the last valid delta time to ensure stable orientation calculations
    float deltaTime = (float)(start - previousTimestamp) / 1000.0f;   //convert ms to s
    
    previousTimestamp = start;                                       //calculate time from one sample to another , not always from start
    last_delta_time_s = deltaTime > 0 ? deltaTime : last_delta_time_s;  //keep time valid for correct angle measurement
    
   
    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
    #else
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);
    #endif
     // Print algorithm outputs
    const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(q);
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
    
    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    //get heading to find direction via qmc
    float heading = FusionCompass(accelerometer, magnetometer,FusionConventionNwu);
    #endif

    fusion_read.roll = euler.angle.roll;
    fusion_read.pitch = euler.angle.pitch;
    fusion_read.yaw = euler.angle.yaw;
    fusion_read.x =  earth.axis.x;
    fusion_read.y = earth.axis.y;
    fusion_read.z = earth.axis.z;

    quat_read.w = q.element.w;
    quat_read.x = q.element.x;
    quat_read.y = q.element.y;
    quat_read.z = q.element.z;

    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    LOG_INF("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, Head: %01f, X %0.2f, Y %0.2f, Z %0.1f, X %0.1f, Y %0.1f, Z %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
            (double)euler.angle.roll,  (double)euler.angle.pitch,
            (double)euler.angle.yaw, (double)heading, (double)accelerometer.axis.x, (double)accelerometer.axis.y,
            (double)accelerometer.axis.z, (double)gyroscope.axis.x, (double)gyroscope.axis.y, (double)gyroscope.axis.z, (double)magnetometer.axis.x, (double)magnetometer.axis.y,
            (double)magnetometer.axis.z);
    #else
   static int i=0;
   if(++i>30)
   {
         LOG_INF("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.2f, Y %0.2f, Z %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
            (double)euler.angle.roll,  (double)euler.angle.pitch,
            (double)euler.angle.yaw, (double)accelerometer.axis.x, (double)accelerometer.axis.y,
            (double)accelerometer.axis.z, (double)gyroscope.axis.x, (double)gyroscope.axis.y, (double)gyroscope.axis.z);
    i=0;
   }
  
    
    #endif
    //reschedule to run preiodically
   // k_work_schedule(&sensor_fusion_timer, K_MSEC((1000 / SAMPLE_RATE_HZ) - (k_uptime_get_32() - start)));
    k_work_schedule(&sensor_fusion_timer, K_MSEC(1000 / SAMPLE_RATE_HZ));


    /*int ret = 0;

    FusionVector gyroscope = {{0.0f, 0.0f, 0.0f}};
    FusionVector accelerometer = {{0.0f, 0.0f, 0.0f}};

    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    FusionVector magnetometer = {{0.0f, 0.0f, 0.0f}};
    #endif

    uint32_t start = k_uptime_get_32();
    ret = read_mpu6050_gyro(&gyroscope.axis.x, &gyroscope.axis.y, &gyroscope.axis.z);
    if (ret!=0)
    {
        LOG_ERR("imu_fetch_gyro err: %d", ret);
    }

      // Convert from rad/s to deg/s
    gyroscope.axis.x = gyroscope.axis.x * (180.0F / (float)M_PI);
    gyroscope.axis.y = gyroscope.axis.y * (180.0F / (float)M_PI);
    gyroscope.axis.z = gyroscope.axis.z * (180.0F / (float)M_PI);

    ret = read_mpu6050_accel(&accelerometer.axis.x, &accelerometer.axis.y, &accelerometer.axis.z);

    if (ret!=0)
    {
        LOG_ERR("imu_fetch_accel err: %d", ret);
    }

      // IMU driver converts to m/s2 by multiplying to 10, convert back to g-force
    accelerometer.axis.x /= (float)SENSOR_GF;
    accelerometer.axis.y /= (float)SENSOR_GF;
    accelerometer.axis.z /= (float)SENSOR_GF;

    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    ret = read_magnetometer_data(&magnetometer.axis.x, &magnetometer.axis.y, &magnetometer.axis.z);

    if (ret!=0)
    {
        LOG_ERR("fetch_mag err: %d", ret);
    }
    #endif
   static int x=0;
   if(++x>20)
   {
   LOG_INF("Gyro before update : %f %f %f", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
    LOG_INF("Accel before update: %f %f %f", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);

    x=0;
   }

    gyroscope = FusionBiasUpdate(&bias, gyroscope);
    
    //calculates elapsed time between gyro samples, converts it to seconds, filters out invalid values, 
   // and keeps the last valid delta time to ensure stable orientation calculations
     float deltaTime = (start - previousTimestamp) / 1000.0f;   //convert ms to s

    if (deltaTime <= 0.0f || deltaTime > 0.05f) {
    deltaTime = 1.0f / SAMPLE_RATE_HZ;  // safe fallback
}
 //  LOG_INF("Delta time: %0.3f seconds", (double)deltaTime);
    previousTimestamp = start;                                       //calculate time from one sample to another , not always from start
   // last_delta_time_s = deltaTime > 0 ? deltaTime : last_delta_time_s;  //keep time valid for correct angle measurement
   
    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
    #else
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);
    #endif
     // Print algorithm outputs
    const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    const FusionEuler euler = FusionQuaternionToEuler(q);
    const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
    
    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    //get heading to find direction via qmc
    float heading = FusionCompass(accelerometer, magnetometer,FusionConventionNwu);
    #endif

    fusion_read.roll = euler.angle.roll;
    fusion_read.pitch = euler.angle.pitch;
    fusion_read.yaw = euler.angle.yaw;
    fusion_read.x =  earth.axis.x;
    fusion_read.y = earth.axis.y;
    fusion_read.z = earth.axis.z;

    quat_read.w = q.element.w;
    quat_read.x = q.element.x;
    quat_read.y = q.element.y;
    quat_read.z = q.element.z;

    #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
    LOG_INF("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, Head: %01f, X %0.2f, Y %0.2f, Z %0.1f, X %0.1f, Y %0.1f, Z %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
            (double)euler.angle.roll,  (double)euler.angle.pitch,
            (double)euler.angle.yaw, (double)heading, (double)accelerometer.axis.x, (double)accelerometer.axis.y,
            (double)accelerometer.axis.z, (double)gyroscope.axis.x, (double)gyroscope.axis.y, (double)gyroscope.axis.z, (double)magnetometer.axis.x, (double)magnetometer.axis.y,
            (double)magnetometer.axis.z);
    #else
   static int i=0;
   if(++i>20)
   {
         LOG_INF("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %f , Y %f, Z %f, X %f, Y %f, Z %f\n",
            (double)euler.angle.roll,  (double)euler.angle.pitch,
            (double)euler.angle.yaw, (double)accelerometer.axis.x, (double)accelerometer.axis.y,
            (double)accelerometer.axis.z, (double)gyroscope.axis.x, (double)gyroscope.axis.y, (double)gyroscope.axis.z);
    i=0;
   }
  
    
    #endif
    //reschedule to run preiodically
 //  k_work_schedule(&sensor_fusion_timer, K_MSEC((1000 / SAMPLE_RATE_HZ) - (k_uptime_get_32() - start)));
   k_work_schedule(&sensor_fusion_timer, K_MSEC(1000 / SAMPLE_RATE_HZ));*/
}

void sensor_fusion_init()
{
    FusionBiasInitialise(&bias, SAMPLE_RATE_HZ);
    FusionAhrsInitialise(&ahrs);

        const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f, 
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE_HZ, // 5 seconds TODO: Replace with actual sample rate
    };

    FusionAhrsSetSettings(&ahrs, &settings);
    /*schedule the work */
   k_work_schedule(&sensor_fusion_timer, K_MSEC(1000 / SAMPLE_RATE_HZ));  
 

}

void print_logs()
{
     LOG_INF("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",
            (double)fusion_read.roll,  (double)fusion_read.pitch,
            (double)fusion_read.yaw);
}

//debugging
/*
test fusion algorithm in seperate project and debug issues*/