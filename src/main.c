#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>


#ifdef CONFIG_ENABLE_TEST
#include "qmc5883l.h"
#endif
#define SLEEP_TIME_MS 100U


/*SDA->26,SCL->27*/ 
static int read_sensor(const struct device *sensor)
{
	struct sensor_value val[3];
	int ret = 0;

	ret = sensor_sample_fetch(sensor);
	if (ret == 0) {
                ret = sensor_channel_get(sensor, SENSOR_CHAN_MAGN_XYZ, val);
        }

	if (ret == 0) {
		printk("( x=%d.%06d  y=%d.%06d  z=%d.%06d)", val[0].val1,val[0].val2 , val[1].val1,val[1].val2 , val[2].val1,val[2].val2);
	
	}

        else{
                printk("sample fetch/get failed: %d\n", ret);

        }

	return ret;
}


int main(void)
{      
        static const struct device *dev = DEVICE_DT_GET(DT_ALIAS(qmc5883l_dev));
        if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return 0;
	}

        if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return 0;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);

        while(1)
        {
                
                int ret = read_sensor(dev);
                if(ret!=0)
                {
                        break;
                }
                k_sleep(K_SECONDS(2));
               

        }
        
        return 0;

}
