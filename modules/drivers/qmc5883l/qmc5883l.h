#ifndef ZEPHYR_DRIVERS_SENSOR_QMC5883L_H_
#define ZEPHYR_DRIVERS_SENSOR_QMC5883L_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

/*register definations*/
#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define MASK_MODE  0xfe
#define MASK_ODR   0xf3


/*config macros*/
#define QMC5883L_ODR_10HZ      0x00
#define QMC5883L_ODR_50HZ      0x01
#define QMC5883L_ODR_100HZ     0x02
#define QMC5883L_ODR_200HZ     0x03

#define QMC5883L_OSR_512       0x00
#define QMC5883L_OSR_256       0x01
#define QMC5883L_OSR_128       0x02
#define QMC5883L_OSR_64        0x03

#define QMC5883L_MODE_STANDBY       0
#define QMC5883L_MODE_CONTINUOUS    1


typedef enum
{
    QMC5883L_RNG_2 = 0,//!< -2G..+2G
    QMC5883L_RNG_8     //!< -8G..+8G
} qmc5883l_range_t;

/*qmc data struct*/
struct qmc5883l_data {
	int16_t x_sample;
	int16_t y_sample;
	int16_t z_sample;
	qmc5883l_range_t range;   /*configure range by taking user input via Kconfig */
};

/*qmc config struct*/
struct qmc5883l_config {
	struct i2c_dt_spec i2c;
};

#endif
