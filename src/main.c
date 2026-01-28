#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include "magnetometer.h"


#define SLEEP_TIME_MS 100U


int main(void)
{      int ret;
        int64_t start = k_uptime_get();

         /*wait for serial display*/
        k_sleep(K_MSEC(1000));

        static const struct device *qmc_sensor = DEVICE_DT_GET(DT_ALIAS(qmc5883l_dev));
        if (qmc_sensor == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return 0;
	}
        
        init_magnetometer(qmc_sensor);

        #ifdef CONFIG_ENABLE_QMC_CALIBRATION
        /*test calibration of qmc sensor*/
        ret = start_magn_calibration(qmc_sensor);
        if(ret!=0){
                printk("calibration start fail\n");
                return 0;
        } 

        /*calibrate for 50s*/
        while (k_uptime_get() - start < 50000) {
        ret = read_magn_with_calibration(qmc_sensor);
        if(ret!=0){
                printk("unable to perform calibration\n");
                return 0;
        } 
        k_sleep(K_MSEC(100));
        }

        zsw_magnetometer_stop_calibration();
        #endif

        while(1)
        {
                /*read x,y,z axis from qmc5883l*/
                ret = read_magn_with_calibration(qmc_sensor);
                if(ret!=0)
                {
                        break;
                }
                 k_sleep(K_MSEC(500));/*previous when testing without calibration: k_sleep(K_SECONDS(2)) */
               
        }
        
        return 0;

}


