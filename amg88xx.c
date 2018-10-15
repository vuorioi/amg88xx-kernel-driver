/*
 * Kernel driver for the Panasonic AMG88xx-series sensors
 *
 * Copyright (C) 2018  Iiro Vuorio
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define DRIVER_NAME "amg88xx"

/* i2c register addresses */
#define MODE_REG 0x00
#define RESET_REG 0x01
#define THERM_LOW_REG 0x0e
#define THERM_HIGH_REG 0x0f

#define SENSOR_FIRST_REG
#define SENSOR_LAST_REG

static inline int amg88xx_write_register(struct i2c_client *client, u8 reg_addr, u8 value)
{
	//TODO maybe check if register is valid
	return i2c_smbus_write_byte_data(client, reg_addr, value);
}

static inline int amg88xx_read_register(struct i2c_client *client, u8 reg_addr)
{
	//TODO maybe check if register is valid
	return i2c_smbus_read_byte_data(client, reg_addr);
}

static int amg88xx_read16b_register(struct i2c_client *client, u8 regl, u8 regh)
{
	int ret;
	int val;

	// First get the high bits and then the low bits
	ret = amg88xx_read_register(dev->client, regh);
	if (ret < 0)
		return ret;
	else
		val = ret << 8;

	ret = amg88xx_read_register(dev->client, regl);
	if (ret < 0)
		return ret;
	else
		val |= ret;

	return val;
}

enum amg88xx_device_mode {
        NORMAL_MODE = 0x0,
        SLEEP_MODE = 0x10,
        STANDBY60_MODE = 0x20,
        STANDBY10_MODE = 0x21 };

enum amg88xx_reset_mode {
	PARTIAL_RST = 0x30,
	FULL_RST = 0x3f };

enum amg88xx_fps {
        FPS10 = 0,
        FPS1 = 1 };

enum amg88xx_interrupt_mode {
        DIFFERENCE_MODE = 0,
        ABSOLUTE_VALUE_MODE = 1 };

enum amg88xx_interrupt_state {
        INT_DISABLED = 0,
        INT_ENABLED = 1 };

struct amg88xx {
        struct i2c_client *client;

        s16 temp_array[8*8];
        
        /*enum amg88xx_device_mode dev_mode;
        enum amg88xx_fps fps;

        enum amg88xx_interrupt_mode int_mode;
        enum amg88xx_interrupt_state int_sate;
        u8 interrupt_array[8];

        s16 int_trigger_level_high : 11;
        s16 int_trigger_level_low : 11;
        s16 int_trigger_hysteresis : 11;

        u8 thermistor_overflow : 1;
        u8 sensor_overflow : 1;
        u8 interrupt_active : 1;

        u8 twice_moving_average_mode : 1;*/

        s16 thermistor_value : 11;
};

static inline int amg88xx_set_dev_mode(struct amg88xx *dev, enum amg88xx_device_mode mode)
{
	return amg88xx_write_register(dev->client, MODE_REG, mode);
}

static inline int amg88xx_get_dev_mode(struct amg88xx *dev)
{
	return amg88xx_read_register(dev->client, MODE_REG);
}

static inline int amg88xx_reset(struct amg88xx *dev)
{
	return amg88xx_write_register(dev->client, RESET_REG, PARTIAL_RST);
}

static inline int amg88xx_read_thermistor(struct amg88xx *dev)
{
	int ret;

	ret = amg88xx_read16b_register(dev->client, THERM_LOW_REG, THERM_HIGH_REG);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read the thermistor registers\n");
		return ret;
	}

	dev->thermistor_value = (s16)ret;
	return 0;
}

static int amg88xx_read_sensor(struct amg88xx *dev)
{
	u8 reg_addr = SENSOR_FIRST_REG;

	// Loop through all the sensor registers
	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			ret = amg88xx_read16b_register(dev->client, reg_addr, reg_addr + 1);
			if (ret < 0) {
				printk(KERN_ERR "Failed to read sensor value at: %d.%d\n",
						col, row);
				return ret;
			}

			dev->temp_array[row*8 + col] = (s16)ret;
			reg_addr += 2;
		}
	}
}

/* sysfs entries */
static ssize_t show_sensor(struct device *dev, struct device_attribute *attr,
			   const char *buf)
{
	struct amg88xx *device;
	int row;
	int col;
	int nwrite;
	unsigned index = 0;

	device = dev_get_drvdata(dev);

	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			/* Write all the values on a row. Each value is sepparated by a comma
			   and there is newline character after the last value */
			nwrite = scprintf(&buf[index],
					  PAGE_SIZE - (index - 1),
					  col < 7 ? "%d, " : "%d\n",
					  (int)device->temp_array[row*8 + col]);
			index += nwrite + 1;
		}
	}

	return index - 1;
}
static DEVICE_ATTR(sensor, S_IRUGO, show_sensor, NULL);

static ssize_t show_thermistor(struct device *dev, struct device_attribute *attr,
			       const char *buf)
{
	struct amg88xx *device;

	device = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", (int)device->thermistor_value);
}
static DEVICE_ATTR(thermistor, S_IRUGO, show_thermistor, NULL);

static int amg88xx_probe_new(struct i2c_client *client)
{
        int ret;
	int temperature;
        struct amg88xx *device;

        device = devm_kzalloc(&client->dev, sizeof(*device), GFP_KERNEL);

        if (device == NULL)
                return -ENOMEM;
        else
                device->client = client;

	dev_set_drvdata(&client->dev, device);
        
        ret = amg88xx_get_dev_mode(device);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read device mode\n");
		return ret;
	} else if (!(ret == NORMAL_MODE ||
		   ret == SLEEP_MODE ||
		   ret == STANDBY60_MODE ||
		   ret == STANDBY11_MODE)) {
                return -EPROTO;
	}

        ret = amg88xx_reset(device);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset device\n");
		return ret;
	}
        
	// Start the device
        ret = amg88xx_set_dev_mode(device, NORMAL_MODE);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set device mode\n");
		return ret;
	}

        ret = amg88xx_read_thermistor(device);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read thermistor value\n");
		return ret;
	}

	printk(KERN_INFO "thermistor value: %d\n", device->thermistor_value);

        // TODO create sysfs entries
	device_create_file(&client->dev, &device_attr_sensor);
	device_create_file(&client->dev, &device_attr_thermistor);
}

static int amg88xx_remove(struct i2c_client *client)
{
        amg88xx_set_dev_mode(&client->dev, SLEEP_MODE);

        // TODO clear sysfs entries
	device_remove_file(&client->dev, &device_attr_sensor);
	device_remove_file(&client->dev, &device_attr_thermistor);
}


static const struct i2c_device_id amg88xx_id_table[] = {
        { "amg88xx", 0 },
        {},
};
MODULE_DEVICE_TABLE(i2c, amg88xx_id_table);

static const struct of_device_id amg88xx_of_match[] = {
         { .compatible = "panasonic,amg88xx" },
         {},
};
MODULE_DEVICE_TABLE(of, amg88xx_of_match);

static struct i2c_driver amg88xx_driver = {
        .driver = {
                .owner          = THIS_MODULE,
                .name           = DRIVER_NAME,
                .of_match_table = of_match_ptr(amg88xx_of_match),
        },
        .probe_new  = amg88xx_probe_new,
        .remove     = amg88xx_remove,
        .id_table   = amg88xx_id,
};

static int __init amg88xx_module_init(void)
{
	return i2c_add_driver(&amg88xx_driver);
}
module_init(amg88xx_module_init);

static void __exit amg88xx_module_exit(void)
{
	i2c_del_driver(&amg88xx_driver);
}
module_exit(amg88xx_module_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("Iiro Vuorio");
MODULE_LISENCE("GPL v2");
MODULE_DESCRIPTION("A kernel driver for the Panasonic AMG88xx-series sensors.");
