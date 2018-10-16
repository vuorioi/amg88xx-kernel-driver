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
#define DEVICE_MODE_REG 0x00
#define RESET_REG 0x01
#define FRAME_RATE_REG 0x02
#define INTERRUPT_MODE_REG 0x03
#define XXX_REG 0x04
#define MOVING_AVERAGE_REG 0x07
#define UPPER_INTERRUPT_LOW_REG 0x08
#define UPPER_INTERRUPT_HIGH_REG 0x09
#define LOWER_INTERRUPT_LOW_REG 0x0a
#define LOWER_INTERRUPT_HIGH_REG 0x0b
#define INTERRUPT_HYST_LOW_REG 0x0c
#define INTERRUPT_HYST_HIGH_REG 0x0d
#define THERM_LOW_REG 0x0e
#define THERM_HIGH_REG 0x0f
#define PIXEL_ROW1_REG 0x10
#define PIXEL_ROW2_REG 0x11
#define PIXEL_ROW3_REG 0x12
#define PIXEL_ROW4_REG 0x13
#define PIXEL_ROW5_REG 0x14
#define PIXEL_ROW6_REG 0x15
#define PIXEL_ROW7_REG 0x16
#define PIXEL_ROW8_REG 0x17
#define SENSOR_FIRST_REG 0x80 // Pixel 1 low bits register
#define SENSOR_LAST_REG 0xFF // Pixel 64 high bits register

/* Low level access helper functions */
static inline int amg88xx_write_register(struct i2c_client *client, u8 reg_addr, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg_addr, value);
}

static inline int amg88xx_read_register(struct i2c_client *client, u8 reg_addr)
{
	return i2c_smbus_read_byte_data(client, reg_addr);
}

static int amg88xx_read16b_register(struct i2c_client *client, u8 regl, u8 regh)
{
	int ret;
	int val;

	// First get the high bits and then the low bits
	ret = amg88xx_read_register(client, regh);
	if (ret < 0)
		return ret;
	else
		val = ret << 8;

	ret = amg88xx_read_register(client, regl);
	if (ret < 0)
		return ret;
	else
		val |= ret;

	return val;
}

/* Device configuration options that are mapped to register values */
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

/* Structure for holding device related data */
struct amg88xx {
	struct i2c_client *client;
	
	/*s16 temp_array[8*8];

	enum amg88xx_device_mode dev_mode;
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

	u8 twice_moving_average_mode : 1;

	s16 thermistor_value : 11;*/
};

/* Helper functions for device access */
static inline int amg88xx_set_dev_mode(struct amg88xx *dev, enum amg88xx_device_mode mode)
{
	return amg88xx_write_register(dev->client, DEVICE_MODE_REG, mode);
}

static int amg88xx_get_dev_mode(struct amg88xx *dev, int *result)
{
	int ret;

	ret = amg88xx_read_register(dev->client, DEVICE_MODE_REG);
	if (ret < 0)
		return ret;
	else
		*result = ret;

	return 0;
}

static inline int amg88xx_reset(struct amg88xx *dev)
{
	// Performe a "soft reset"
	return amg88xx_write_register(dev->client, RESET_REG, PARTIAL_RST);
}

static int amg88xx_read_thermistor(struct amg88xx *dev, int *result)
{
	int ret;

	ret = amg88xx_read16b_register(dev->client, THERM_LOW_REG, THERM_HIGH_REG);
	if (ret < 0) 
		return ret;
	else
		*result = ret;

	return 0;
}

static int amg88xx_read_sensor(struct amg88xx *dev, int *res_array)
{
	int index;
	int ret;
	u8 reg_addr = SENSOR_FIRST_REG;

	// Loop through all the sensor registers
	for (index = 0; index < 64; index++) {
		ret = amg88xx_read16b_register(dev->client, reg_addr, reg_addr + 1);
		if (ret < 0)
			return ret;

		res_array[index] = ret;
		reg_addr += 2;
	}

	return 0;
}

/* sysfs entries and the functions to implement them */
static ssize_t show_sensor(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct amg88xx *device;
	int ret;
	int row;
	int col;
	int nwrite;
	int sensor_array[64];
	unsigned index = 0;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_sensor(device, sensor_array);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read the sensor\n");
		return ret;
	}

	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			/* Write all the values on a row. Each value is sepparated by a comma
			   and there is newline character after the last value */
			nwrite = scnprintf(&buf[index],
					   PAGE_SIZE - (index - 1),
					   col < 7 ? "%x, " : "%x\n",
					   sensor_array[row*8 + col]);
			index += nwrite + 1;
		}
	}

	return index - 1;
}
static DEVICE_ATTR(sensor, S_IRUGO, show_sensor, NULL);

static ssize_t show_thermistor(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct amg88xx *device;
	int ret;
	int thermistor_value;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_thermistor(device, &thermistor_value);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read thermistor value\n");
		return ret;
	}

	// Convert to celcius scale
	return scnprintf(buf, PAGE_SIZE, "%x\n", thermistor_value);
}
static DEVICE_ATTR(thermistor, S_IRUGO, show_thermistor, NULL);

static ssize_t show_device_mode(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct amg88xx *device;
	int ret;
	int device_mode;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_dev_mode(device, &device_mode);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read device mode\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%x\n", device_mode);
}

static ssize_t store_device_mode(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct amg88xx *device;
	int ret;
	u8 mode;

	device = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 16, &mode);
	if (ret < 0) {
		printk(KERN_ERR "Failed to read value from input\n");
		return ret;
	}

	if (!(mode == NORMAL_MODE ||
	      mode == SLEEP_MODE ||
	      mode == STANDBY60_MODE ||
	      mode == STANDBY10_MODE)) {
		printk(KERN_ERR "Input is not a supported mode\n");
		return -EINVAL;
	}

	ret = amg88xx_set_dev_mode(device, mode);
	if (ret < 0) {
		printk(KERN_ERR "Failed to set device mode\n");
		return ret;
	}

	return count;
}

static DEVICE_ATTR(device_mode,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   show_device_mode,
		   store_device_mode);

// TODO all the rest of the sysfs stuff
// TODO group attributes

static int amg88xx_probe_new(struct i2c_client *client)
{
	int ret;
	struct amg88xx *device;

	device = devm_kzalloc(&client->dev, sizeof(*device), GFP_KERNEL);

	if (device == NULL)
		return -ENOMEM;
	else
		device->client = client;

	dev_set_drvdata(&client->dev, device);

        ret = amg88xx_reset(device);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset device\n");
		return ret;
	}

	// TODO create sysfs entries
	// TODO move sysfs entries to attribute group
	device_create_file(&client->dev, &dev_attr_sensor);
	device_create_file(&client->dev, &dev_attr_thermistor);
	device_create_file(&client->dev, &dev_attr_device_mode);

	return 0;
}

static int amg88xx_remove(struct i2c_client *client)
{
	int ret;
	struct amg88xx *device = dev_get_drvdata(&client->dev);

	// TODO clear sysfs entries
	device_remove_file(&client->dev, &dev_attr_sensor);
	device_remove_file(&client->dev, &dev_attr_thermistor);
	device_remove_file(&client->dev, &dev_attr_device_mode);

	ret = amg88xx_set_dev_mode(device, SLEEP_MODE);
	if (ret < 0) {
		printk(KERN_ERR "Failed to put the device to sleep\n");
		return ret;
	}

	return 0;
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
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= of_match_ptr(amg88xx_of_match),
	},
	.probe_new = amg88xx_probe_new,
	.remove	   = amg88xx_remove,
	.id_table  = amg88xx_id_table,
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
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("A kernel driver for the Panasonic AMG88xx-series sensors.");
