/*compatible property , should be same for driver, binding and dts*/
#define DT_DRV_COMPAT   qmc5883l_sensor

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include "qmc5883l.h"


LOG_MODULE_REGISTER(QMC5883L, CONFIG_SENSOR_LOG_LEVEL);


/*convert raw sensor value to mG*/
static void qmc5883l_convert(struct sensor_value *val, int16_t raw_val,
			    qmc5883l_range_t rng)        
{   /*TODO: revisit the calculation values if it is correct or no, by comparing it with running on esp32*/
	/* value(in mG) = raw * (full scale range/32768),  
	nrf52840 have fpu but here use integer conversions to make zephyr driver generic */
	int16_t full_scale = (rng == QMC5883L_RNG_2) ? 2000 : 8000;
	
	/* take int32 to prevent overflow*/
	int32_t value_mg = (int32_t)raw_val * full_scale;

	/*integer part , 32768 ADC range int16_t, so divide by 32768 to have reading reference to it   */
	val->val1 = value_mg / 32768 ;  
	
	/* mod gives reminder(fractional part), take abs so if valuemg is negative then val2 and val1 both negative 
	combine both value in main app cause sign positve*/
	val->val2 = (int32_t)abs((((int64_t)value_mg % 32768) * 1000000L) / 32768);
}

static int qmc5883l_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{   /*fill up data from device structure TODO: how data in generic device fillup*/
	struct qmc5883l_data *drv_data = dev->data;

	if (chan == SENSOR_CHAN_MAGN_X) {
		qmc5883l_convert(val, drv_data->x_sample,
				 drv_data->range);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		qmc5883l_convert(val, drv_data->y_sample,
				 drv_data->range);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		qmc5883l_convert(val, drv_data->z_sample,
				 drv_data->range);
	} else if (chan == SENSOR_CHAN_MAGN_XYZ) {
		qmc5883l_convert(val, drv_data->x_sample,
				 drv_data->range);
		qmc5883l_convert(val + 1, drv_data->y_sample,
				 drv_data->range);
		qmc5883l_convert(val + 2, drv_data->z_sample,
				 drv_data->range);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int qmc5883l_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct qmc5883l_data *drv_data = dev->data;
	const struct qmc5883l_config *config = dev->config;
	int16_t buf[3];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	/* fetch magnetometer sample */
	if (i2c_burst_read_dt(&config->i2c, REG_XOUT_L,
			      (uint8_t *)buf, 6) < 0) {
		LOG_ERR("Failed to fetch magnetometer sample.");
		return -EIO;
	}

/*no need to convert endianess as both nrf52 cpu and qmc5883l are little endian */
	drv_data->x_sample = buf[0];
	drv_data->y_sample = buf[1];
	drv_data->z_sample = buf[2];

	return 0;
}


/*assign function address*/
static DEVICE_API(sensor, qmc5883l_driver_api) = {
	.sample_fetch = qmc5883l_sample_fetch,
	.channel_get = qmc5883l_channel_get,
};

/*#ifdefined for Kconfig only work for boolean options , for int , use switch/if-else*/
/*config full scale range*/
static uint8_t qmc5883l_range_cfg(void)
{
    switch (CONFIG_QMC5883L_RANGE) {
        case 2000: return QMC5883L_RNG_2;
        case 8000: return QMC5883L_RNG_8;
        default:   return QMC5883L_RNG_2;  // fallback (should never happen due to Kconfig default)
    }
}


/*config odr via Kconfig*/
static uint8_t qmc5883l_odr_cfg(void)
{
    switch (CONFIG_QMC5883L_ODR) {
        case 10:  return QMC5883L_ODR_10HZ;
        case 50:  return QMC5883L_ODR_50HZ;  
        case 100: return QMC5883L_ODR_100HZ;
        case 200: return QMC5883L_ODR_200HZ;
        default:  return QMC5883L_ODR_50HZ;  
    }
}


/*config osr via Kconfig*/
static uint8_t qmc5883l_osr_cfg(void)
{
    switch (CONFIG_QMC5883L_OSR) {
        case 512: return QMC5883L_OSR_512;
        case 256: return QMC5883L_OSR_256;
        case 128: return QMC5883L_OSR_128;  
        case 64:  return QMC5883L_OSR_64;
        default:  return QMC5883L_OSR_128;  
    }
}

int qmc5883l_init(const struct device *dev)
{
	struct qmc5883l_data *drv_data = dev->data;
	const struct qmc5883l_config *config = dev->config;
	uint8_t cfg[2], id;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}

	/* check chip ID */
	if (i2c_burst_read_dt(&config->i2c, REG_ID, &id, 1) < 0) {
		LOG_ERR("Failed to read chip ID.");
		return -EIO;
	}

	/* configure device */
	cfg[0] = REG_CTRL1;
	cfg[1] = QMC5883L_MODE_CONTINUOUS | qmc5883l_odr_cfg() << 2 |  qmc5883l_range_cfg() << 4 | qmc5883l_osr_cfg() << 6;

	/*get range for conversion*/
	drv_data->range = qmc5883l_range_cfg();
	

	if (i2c_write_dt(&config->i2c, cfg, sizeof(cfg)) < 0) {
		LOG_ERR("Failed to configure chip.");
		return -EIO;
	}

	return 0;
}

#define QMC5883L_DEFINE(inst)                                                  \
    /*declare qmc5883l_data_##inst and inst number is replaced by 0,1,2...*/   \
    static struct qmc5883l_data qmc5883l_data_##inst;                          \
                                                                               \
    /*declare and init config structure */                                     \
    static const struct qmc5883l_config qmc5883l_config_##inst = {			   \
    /*create i2c instance with same inst number and populate with DT values*/  \
		.i2c = I2C_DT_SPEC_INST_GET(inst),						               \
	};	                                                                       \
                                                                               \
    /*create a device instance from device tree and call init during boot*/    \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, qmc5883l_init, NULL,					   \
			      &qmc5883l_data_##inst, &qmc5883l_config_##inst, POST_KERNEL, \
			      CONFIG_SENSOR_INIT_PRIORITY, &qmc5883l_driver_api);		   \
// The Devicetree build process calls this to create an instance of structs for
// each device defined in the Devicetree
DT_INST_FOREACH_STATUS_OKAY(QMC5883L_DEFINE)
