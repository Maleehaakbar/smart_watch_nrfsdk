#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include "magnetometer.h"
#include "imu.h"
#include "sensor_fusion.h"


#define SLEEP_TIME_MS 100U
float mag_x, mag_y, mag_z;
float gyro_x, gyro_y, gyro_z;
float accel_x , accel_y, accel_z;

int main(void)
{       int ret;
        #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
        

        #ifdef CONFIG_ENABLE_QMC_CALIBRATION
        int64_t start = k_uptime_get();
        #endif

         /*wait for serial display*/
        k_sleep(K_MSEC(1000));
        
        init_magnetometer();

        #ifdef CONFIG_ENABLE_QMC_CALIBRATION
        /*test calibration of qmc sensor*/
        ret = start_magn_calibration();
        if(ret!=0){
                printk("calibration start fail\n");
                return 0;
        } 

        /*calibrate for 50s*/
        while (k_uptime_get() - start < 50000) {
        ret = read_magn_with_calibration(&mag_x, &mag_y, &mag_z); /*TODO: revist as no need of pass parameters here as during calibration , values of sensor are not updated , just max and min values are updated*/
        if(ret!=0){
                printk("unable to perform calibration\n");
                return 0;
        } 
        k_sleep(K_MSEC(100));
        }
        zsw_magnetometer_stop_calibration();
        #endif

        #endif
        #ifdef CONFIG_ENABLE_IMU
        
        k_msleep(150);  // wait 150ms after init

        init_mpu6050();
        read_attribute();
        #endif

       sensor_fusion_init();
        while(1)
        { 
                #ifdef CONFIG_ENABLE_QMC5883L_SENSOR
                /*read x,y,z axis from qmc5883l*/
                ret = read_magnetometer_data(&mag_x, &mag_y, &mag_z);
                if(ret!=0)
                {
                       // break;  //dont break, just to test with breadboard

                }
                k_sleep(K_MSEC(500));/*previous when testing without calibration: k_sleep(K_SECONDS(2)) */
                #endif

                #ifdef CONFIG_ENABLE_IMU
                /*TODO make qmc and mpu run side by side in while(1)*/
                ret = read_mpu6050_gyro(&gyro_x, &gyro_y, &gyro_x);
                if(ret!=0)
                {
                       // break;
                }

                ret = read_mpu6050_accel(&accel_x, &accel_y, &accel_x);
                if(ret!=0)
                {
                      //  break;
                }
                
                      //  print_logs();
        
               
                k_sleep(K_MSEC(10));
              
                #endif
        }
        
        return 0;

}


