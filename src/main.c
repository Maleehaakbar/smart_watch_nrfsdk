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
#include "rtc_time.h"
#include "watch_display.h"


#define SLEEP_TIME_MS 100U

int main(void)
{       int ret;
        char buff1[30] = {0};
        char buff2[20] = {0};

        const char* mag_direction = NULL;

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
       
        sensor_fusion_init();
        #endif

        #ifdef CONFIG_ENABLE_RTC_TIME
        /*init rtc and set initial time*/
        ds3231_rtc_init();   
        #endif
        
        watch_display_init();  /*init oled display*/
        
        /*create display screen via lvgl*/
        create_screen1();  
        
        #ifdef CONFIG_ENABLE_IMU
        create_screen2();
        #endif

        /* load screen1 initally*/
        lv_scr_load(screen1);

        lv_timer_handler();

       
        while(1)
        { 

                #ifdef CONFIG_ENABLE_IMU
                mag_direction = get_direction();
                k_sleep(K_MSEC(1000)); //k_sleep(K_MSEC(10)); //used to test sensor fusion
              
                #endif

                #ifdef CONFIG_ENABLE_RTC_TIME
                rtc_get_date_time();
                #endif

                /*display time on watch*/
                sprintf(buff1, "%02d:%02d:%02d",tm.tm_hour, tm.tm_min, tm.tm_sec);
                lv_label_set_text(rtc_label, buff1);

                k_sleep(K_MSEC(100));
                
                #ifdef CONFIG_ENABLE_IMU
                switch_screens();
                /*display direction*/
                sprintf(buff2,"%s", mag_direction);
                lv_label_set_text(direction_label, buff2);
                #endif
                lv_timer_handler();

                k_sleep(K_MSEC(1000));
        }
        
        return 0;

}

/*target : print RTC time on one screen and magnetic direction on screen2

 1. get RTC time and display
 2. get direction and display
 
 
 int main()
 {
 if enable qmc (qmc init)
 if qmc calibration enable(perform calibration)
 if enable imu (init mpu , init sensor fusion)
 if rtc enable : rtc init

 init zephyr display
 turn off display
 create screen1 and 2

 load screen1
 lv time handler

   while(1)
   {
     read rtc and get direction

     set lvgl text for rtc and direction
     
   }
 
 } 
   
 create screen1()
 { 
    create screen and dynamic label

 }
 
 */


