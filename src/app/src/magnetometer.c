#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_data_types.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

/*TODO: header file incude doesnt make diference , code is compiled without it, so need
to revisit how to exclude qmc sensor from build*/
#ifdef CONFIG_ENABLE_QMC5883L_SENSOR

LOG_MODULE_REGISTER(magnetometer, CONFIG_ZSW_APP_LOG_LEVEL);

#define SETTINGS_NAME_MAGN              "magn"
#define SETTINGS_KEY_CALIB              "calibr"
#define SETTINGS_MAGN_CALIB             SETTINGS_NAME_MAGN "/" SETTINGS_KEY_CALIB

typedef struct {
    double offset_x;
    double offset_y;
    double offset_z;
} magn_calib_data_t;

static double last_x;
static double last_y;
static double last_z;
static double max_x;
static double max_y;
static double max_z;
static double min_x;
static double min_y;
static double min_z;
bool is_calibrating;
static magn_calib_data_t calibration_data;


/* read qmc833l sensor data without calibration SDA->26,SCL->27*/ 
int read_sensor(const struct device *sensor)
{
	struct sensor_value val[3];
	int ret = 0;

	ret = sensor_sample_fetch(sensor);
	if (ret == 0) {
                ret = sensor_channel_get(sensor, SENSOR_CHAN_MAGN_XYZ, val);
        }

	if (ret == 0) {
		printk("( x=%d.%06d  y=%d.%06d  z=%d.%06d)\n", val[0].val1,val[0].val2 , val[1].val1,val[1].val2 , val[2].val1,val[2].val2);
	
	}

        else{
                printk("sample fetch/get failed: %d\n", ret);

        }

	return ret;
}

/*start qmc clibration*/
int start_magn_calibration(const struct device *sensor)
{
	if (!device_is_ready(sensor)) {
        return -ENODEV;
    }

	/*assign min and max values for x,y,z axis range */
 	max_x = -65000;
    max_y = -65000;
    max_z = -65000;
    min_x = 65000;
    min_y = 65000;
    min_z = 65000;
	is_calibrating = true;

    return 0;
}

int read_magn_with_calibration(const struct device *sensor)
{
	struct sensor_value val[3];
	int ret = 0;

	ret = sensor_sample_fetch(sensor);
	if (ret == 0) {
                ret = sensor_channel_get(sensor, SENSOR_CHAN_MAGN_XYZ, val);
        }

	if (ret == 0) {
		//printk("( x=%d.%06d  y=%d.%06d  z=%d.%06d)\n", val[0].val1,val[0].val2 , val[1].val1,val[1].val2 , val[2].val1,val[2].val2);
	
	}

    else{
            printk("sample fetch/get failed: %d\n", ret);
            return ret;

    }

	last_x = sensor_value_to_float(&val[0]);
	last_y = sensor_value_to_float(&val[1]);
	last_z = sensor_value_to_float(&val[2]);

	 if (is_calibrating) {
        if (last_x < min_x) {
            min_x = last_x;
        }
        if (last_x > max_x) {
            max_x = last_x;
        }

        if (last_y < min_y) {
            min_y = last_y;
        }
        if (last_y > max_y) {
            max_y = last_y;
        }

        if (last_z < min_z) {
            min_z = last_z;
        }
        if (last_z > max_z) {
            max_z = last_z;
        }

        return ret;
    }

    last_x = last_x - calibration_data.offset_x;
    last_y = last_y - calibration_data.offset_y;
    last_z = last_z - calibration_data.offset_z;

     printk("Calibrated: X=%f Y=%f Z=%f\n",
               last_x, last_y, last_z);
   

    return ret;

}


void zsw_magnetometer_stop_calibration(void)
{
    
    is_calibrating = false;
	/*Because of nearby magnets / metal, the center shifts
	(min + max) / 2 ≠ 0  so offset = min+max/2*/
    calibration_data.offset_x = (max_x + min_x) / 2;
    calibration_data.offset_y = (max_y + min_y) / 2;
    calibration_data.offset_z = (max_z + min_z) / 2;
    LOG_INF("max_x:%f max_y:%f max_z=%f min_x=%f, min_y=%f min_z=%f", max_x, max_y, max_z, min_x, min_y, min_z);
    LOG_INF("calibaration offset x= %f , y=%f , z=%f \n", calibration_data.offset_x, calibration_data.offset_y, calibration_data.offset_z);

    /*save calibration settings*/
    settings_save_one(SETTINGS_MAGN_CALIB, &calibration_data, sizeof(magn_calib_data_t));

}

/*save calibration offsets, so calibration not need to done at every reset/boot */
int load_magn_calibration(const char *p_key, size_t len,
                         settings_read_cb read_cb, void *p_cb_arg, void *p_param)
{
        ARG_UNUSED(p_key);

    if (len != sizeof(magn_calib_data_t)) {
        LOG_ERR("Invalid length of magn calibration data");
        return -EINVAL;
    }

    if (read_cb(p_cb_arg, &calibration_data, len) != sizeof(magn_calib_data_t)) {
        LOG_ERR("Error reading magn calibration data");
        return -EIO;
    }

    LOG_WRN("Calibration data loaded: x: %f, y: %f, z: %f",
            calibration_data.offset_x, calibration_data.offset_y, calibration_data.offset_z);

    return 0;
}

int init_magnetometer(const struct device *sensor)
{
    /*check if qmc sensor is ready*/
        if (!device_is_ready(sensor)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       sensor->name);
		return -ENODEV;
	}

	printk("Found device \"%s\", getting sensor data\n", sensor->name);

     if (settings_subsys_init()) {
        LOG_ERR("Error during settings_subsys_init!");
        return -EFAULT;
    }

    if (settings_load_subtree_direct(SETTINGS_MAGN_CALIB, load_magn_calibration , NULL)) {
        LOG_ERR("Error during settings_load_subtree!");
        return -EFAULT;
    }

    return 0;
}

#endif

//todo if x and y axis swap needed for mu and mpu
//TODO : https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/fetch_and_get.html
//polling and triggers in sensors.

/*calibration save flow
1.save calibation data
2. load calibration data
3. call, the load function in init, if calibration is not done, then it is zero at first call
4. include config settings in prj.conf */


/*init flow
1.check if device is ready
2. init subsystem
3. load calibration settings
TODO: can add power saving , attributes, triggers etc
*/

/*calibration procedure
1. Take the raw readings .
2. Find extremes(max and min values) by rotatiing sensor for 30-60 sec in x,y,z directions(up down, leftright, backforth)
3. find offset using (max+min)/2
4. Add offset calibration in all future readings.
TODO: Calibration results are not accurate, need to revisit*/

/*TODO: in compass app ui , if calibration data is stale, ask user to calibrate
for now we are just testing calibration functions in main.c with temporary config option*/
     


