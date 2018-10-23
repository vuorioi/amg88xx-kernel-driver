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
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define DRIVER_NAME "amg88xx"

/*
 * Since we're dealing with 12-bit numbers the sign-bit needs to be extended
 * for the number to be represented correctly
 */
#define convert_to_s16(dst, src) \
	dst = src & 0x800 ? (src | (0xf << 12)) : src

/* i2c register addresses */
#define DEVICE_MODE_REG			0x00
#define RESET_REG			0x01
#define FRAMERATE_REG			0x02
#define INTERRUPT_CTRL_REG		0x03
#define STATUS_FLAG_REG			0x04 //TODO | one sysfs entry
#define STATUS_FLAG_CLR_REG		0x05 //TODO |
#define MOVING_AVERAGE_REG		0x07 //TODO
#define UPPER_INTERRUPT_LOW_REG		0x08
#define UPPER_INTERRUPT_HIGH_REG	0x09
#define LOWER_INTERRUPT_LOW_REG		0x0a
#define LOWER_INTERRUPT_HIGH_REG	0x0b
#define INTERRUPT_HYST_LOW_REG		0x0c
#define INTERRUPT_HYST_HIGH_REG		0x0d
#define THERM_LOW_REG			0x0e
#define THERM_HIGH_REG			0x0f
#define PIXEL_ROW1_REG			0x10
#define PIXEL_ROW2_REG			0x11
#define PIXEL_ROW3_REG			0x12
#define PIXEL_ROW4_REG			0x13
#define PIXEL_ROW5_REG			0x14
#define PIXEL_ROW6_REG			0x15
#define PIXEL_ROW7_REG			0x16
#define PIXEL_ROW8_REG			0x17
#define SENSOR_FIRST_REG		0x80 // Pixel 1 low bits register
#define SENSOR_LAST_REG			0xFF // Pixel 64 high bits register

/* 
 * Low level access helper functions
 */
static inline int amg88xx_write8(struct i2c_client *client, u8 reg_addr, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg_addr, value);
}

static inline int amg88xx_read8(struct i2c_client *client, u8 reg_addr)
{
	return i2c_smbus_read_byte_data(client, reg_addr);
}

static int amg88xx_read16(struct i2c_client *client, u8 regl, u8 regh)
{
	int ret;
	u16 val;

	// First get the high bits and then the low bits
	ret = amg88xx_read8(client, regh);
	if (ret < 0)
		return ret;
	else
		val = ret << 8;

	ret = amg88xx_read8(client, regl);
	if (ret < 0)
		return ret;
	else
		val |= ret;

	return val;
}

static int amg88xx_write16(struct i2c_client *client, u8 regl, u8 regh, u16 value)
{
	int ret;

	// Write the low register first and then the high register
	ret = amg88xx_write8(client, regl, (u8)value);
	if (ret < 0)
		return ret;

	return amg88xx_write8(client, regh, (u8)(value >> 8));
}

/* 
 * Device configuration options that are mapped to register values
 */
enum amg88xx_device_mode {
	NORMAL_MODE = 0x0,
	SLEEP_MODE = 0x10,
	STANDBY60_MODE = 0x20,
	STANDBY10_MODE = 0x21 };

static const char *mode_strs[4] = {
	"normal",
	"sleep",
	"standby_60",
	"standby_10" };

enum amg88xx_reset_mode {
	PARTIAL_RST = 0x30,
	FULL_RST = 0x3f };

enum amg88xx_fps {
	FPS10 = 0,
	FPS1 = 1 };

enum amg88xx_interrupt_mode {
	DIFFERENCE_MODE = 0, //FIXME
	ABSOLUTE_VALUE_MODE = 1 };

enum amg88xx_interrupt_state {
	INT_DISABLED = 0,
	INT_ENABLED = 1 };

/*
 * Structure for holding device related data
 */
struct amg88xx {
	struct i2c_client *client;
	struct gpio_desc *int_gpio;
};

/*
 * Handler for the threaded irq
 */
static irqreturn_t irq_handler(int irq, void *dev)
{
	struct amg88xx *device;

	device = dev;

	// Signal the userspace by notifying pollers on the 'interrupt' file
	sysfs_notify(&device->client->dev.kobj, NULL, "interrupt");

	return IRQ_HANDLED;
}

/*
 * Helper functions for device access
 */
static inline int amg88xx_set_dev_mode(struct amg88xx *dev, enum amg88xx_device_mode mode)
{
	return amg88xx_write8(dev->client, DEVICE_MODE_REG, mode);
}

static int amg88xx_get_dev_mode(struct amg88xx *dev, int *result)
{
	int ret;

	ret = amg88xx_read8(dev->client, DEVICE_MODE_REG);
	if (ret < 0)
		return ret;
	else
		*result = ret;

	return 0;
}

static inline int amg88xx_reset(struct amg88xx *dev, enum amg88xx_reset_mode mode)
{
	return amg88xx_write8(dev->client, RESET_REG, mode);
}

static int amg88xx_get_framerate(struct amg88xx *dev, unsigned *framerate)
{
	int ret;

	ret = amg88xx_read8(dev->client, FRAMERATE_REG);
	if (ret < 0)
		return ret;
	else if (ret == FPS10)
		*framerate = 10;
	else if (ret == FPS1)
		*framerate = 1;
	else
		*framerate = 0;

	return 0;
}

static inline int amg88xx_set_framerate(struct amg88xx *dev, enum amg88xx_fps framerate)
{
	return amg88xx_write8(dev->client, FRAMERATE_REG, framerate);
}

static int amg88xx_get_int_conf(struct amg88xx *dev, int *mode, int *enabled)
{
	int ret;

	ret = amg88xx_read8(dev->client, INTERRUPT_CTRL_REG);
	if (ret < 0) {
		return ret;
	} else {
		*mode = ret & 0x2;
		*enabled = ret & 0x1;
	}

	return 0;
}

static int amg88xx_set_int_mode(struct amg88xx *dev, enum amg88xx_interrupt_mode mode)
{
	int ret;
	u8 val;

	ret = amg88xx_read8(dev->client, INTERRUPT_CTRL_REG);
	if (ret < 0) {
		return ret;
	} else {
		if (mode)
			val = ret | 0x2;
		else
			val = ret & ~(0x2);
	}

	return amg88xx_write8(dev->client, INTERRUPT_CTRL_REG, val);
}

static int amg88xx_set_int_state(struct amg88xx *dev, enum amg88xx_interrupt_state state)
{
	int ret;
	u8 val;

	ret = amg88xx_read8(dev->client, INTERRUPT_CTRL_REG);
	if (ret < 0) {
		return ret;
	} else {
		if (state)
			val = ret | 0x1;
		else
			val = ret & ~(0x1);
	}

	return amg88xx_write8(dev->client, INTERRUPT_CTRL_REG, val);
}

static int amg88xx_get_int_upper_limit(struct amg88xx *dev, s16 *limit)
{
	int ret;

	ret = amg88xx_read16(dev->client, 
			     UPPER_INTERRUPT_LOW_REG,
			     UPPER_INTERRUPT_HIGH_REG);
	if (ret < 0)
		return ret;
	else
		convert_to_s16(*limit, ret);

	return 0;
}

static int amg88xx_get_int_lower_limit(struct amg88xx *dev, s16 *limit)
{
	int ret;

	ret = amg88xx_read16(dev->client,
			     LOWER_INTERRUPT_LOW_REG,
			     LOWER_INTERRUPT_HIGH_REG);
	if (ret < 0)
		return ret;
	else
		convert_to_s16(*limit, ret);

	return 0;
}

static int amg88xx_get_int_hysteresis(struct amg88xx *dev, s16 *hysteresis)
{
	int ret;

	ret = amg88xx_read16(dev->client,
			     INTERRUPT_HYST_LOW_REG,
			     INTERRUPT_HYST_HIGH_REG);
	if (ret < 0)
		return ret;
	else
		convert_to_s16(*hysteresis, ret);

	return 0;
}

static inline int amg88xx_set_int_upper_limit(struct amg88xx *dev, s16 limit)
{
	return amg88xx_write16(dev->client,
			       UPPER_INTERRUPT_LOW_REG,
			       UPPER_INTERRUPT_HIGH_REG,
			       (u16)limit);
}

static inline int amg88xx_set_int_lower_limit(struct amg88xx *dev, s16 limit)
{
	return amg88xx_write16(dev->client,
			       LOWER_INTERRUPT_LOW_REG,
			       LOWER_INTERRUPT_HIGH_REG,
			       (u16)limit);
}

static inline int amg88xx_set_int_hysteresis(struct amg88xx *dev, s16 hysteresis)
{
	return amg88xx_write16(dev->client,
			       INTERRUPT_HYST_LOW_REG,
			       INTERRUPT_HYST_HIGH_REG,
			       (u16)hysteresis);
}

static int amg88xx_read_thermistor(struct amg88xx *dev, s16 *result)
{
	int ret;

	ret = amg88xx_read16(dev->client, THERM_LOW_REG, THERM_HIGH_REG);
	if (ret < 0) 
		return ret;

	convert_to_s16(*result, ret);

	return 0;
}

static int amg88xx_read_sensor(struct amg88xx *dev, s16 *res_array)
{
	int i;
	int ret;
	u8 reg_addr = SENSOR_FIRST_REG;

	// Loop through all the sensor registers
	for (i = 0; i < 64; i++) {
		ret = amg88xx_read16(dev->client, reg_addr, reg_addr + 1);
		if (ret < 0)
			return ret;

		convert_to_s16(res_array[i], ret);
		reg_addr += 2;
	}

	return 0;
}

static int amg88xx_read_interrupt_map(struct amg88xx *dev, u8 *res_array)
{
	int i;
	int ret;
	u8 reg_addr = PIXEL_ROW1_REG;
	
	// Lopp trough all the interrupt map registers
	for (i = 0; i < 8; i++) {
		ret = amg88xx_read8(dev->client, reg_addr);
		if (ret < 0)
			return ret;

		res_array[i] = ret;
		reg_addr++;
	}

	return 0;
}

/* 
 * sysfs entries and the functions to implement them
 */
static ssize_t show_sensor(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct amg88xx *device;
	int ret;
	int row;
	int col;
	int nwrite;
	s16 sensor_array[64];
	unsigned index = 0;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_sensor(device, sensor_array);
	if (ret < 0) {
		dev_err(dev, "Failed to read the sensor\n");
		return ret;
	}

	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			/* 
			 * Write all the values on a row. Each value is sepparated by a comma
			 * and there is newline character after the last value
			 */
			nwrite = scnprintf(&buf[index],
					   PAGE_SIZE - index,
					   col < 7 ? "%d, " : "%d\n",
					   sensor_array[row*8 + col]);
			index += nwrite;
		}
	}

	return index;
}
static DEVICE_ATTR(sensor, S_IRUGO, show_sensor, NULL);

static ssize_t show_thermistor(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct amg88xx *device;
	int ret;
	s16 thermistor_value;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_thermistor(device, &thermistor_value);
	if (ret < 0) {
		dev_err(dev, "Failed to read thermistor value\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", thermistor_value);
}
static DEVICE_ATTR(thermistor, S_IRUGO, show_thermistor, NULL);

static ssize_t show_device_mode(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct amg88xx *device;
	int ret;
	int mode;
	const char* str;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_dev_mode(device, &mode);
	if (ret < 0) {
		dev_err(dev, "Failed to read device mode\n");
		return ret;
	}

	switch (mode) {
	case NORMAL_MODE:
		str = mode_strs[0];
		break;
	case SLEEP_MODE:
		str = mode_strs[1];
		break;
	case STANDBY60_MODE:
		str = mode_strs[2];
		break;
	case STANDBY10_MODE:
		str = mode_strs[3];
		break;
	default:
		dev_err(dev, "Unkown mode from hw\n");
		return -EREMOTEIO;
	}

	return scnprintf(buf, PAGE_SIZE, "%s\n", str);
}

static ssize_t store_device_mode(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct amg88xx *device;
	int ret;
	enum amg88xx_device_mode mode;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("normal", buf)) {
		mode = NORMAL_MODE;
	} else if (sysfs_streq("sleep", buf)) {
		mode = SLEEP_MODE;
	} else if (sysfs_streq("standby_60", buf)) {
		mode = STANDBY60_MODE;
	} else if (sysfs_streq("standby_10", buf)) {
		mode = STANDBY10_MODE;
	} else {
		dev_err(dev, "Input is not a supported mode\n");
		return -EINVAL;
	}

	ret = amg88xx_set_dev_mode(device, mode);
	if (ret < 0) {
		dev_err(dev, "Failed to set device mode\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(device_mode,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   show_device_mode,
		   store_device_mode);

static ssize_t show_interrupt_mode(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct amg88xx *device;
	int ret;
	int mode;
	int enabled;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_int_conf(device, &mode, &enabled);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupt mode\n");
		return ret;
	}

	return scnprintf(buf,
			 PAGE_SIZE,
			 "%s\n",
			 mode ? "absolute" : "differential");
}

static ssize_t store_interrupt_mode(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct amg88xx *device;
	int ret;
	enum amg88xx_interrupt_mode mode;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("absolute", buf)) {
		mode = ABSOLUTE_VALUE_MODE;
	} else if (sysfs_streq("differential", buf)) {
		mode = DIFFERENCE_MODE; //FIXME
	} else {
		dev_err(dev, "Invalid interrupt mode\n");
		return -EINVAL;
	}

	ret = amg88xx_set_int_mode(device, mode);
	if (ret < 0) {
		dev_err(dev, "Failed to set interrupt mode\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(interrupt_mode,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   show_interrupt_mode,
		   store_interrupt_mode);

static ssize_t show_interrupt_state(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct amg88xx *device;
	int ret;
	int mode;
	int enabled;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_int_conf(device, &mode, &enabled);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupt mode\n");
		return ret;
	}

	return scnprintf(buf,
			 PAGE_SIZE,
			 "%s\n",
			 enabled ? "enabled" : "disabled");
}

static ssize_t store_interrupt_state(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct amg88xx *device;
	int ret;
	enum amg88xx_interrupt_state state;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("enabled\n", buf)) {
		state = INT_ENABLED;
	} else if (sysfs_streq("disabled", buf)) {
		state = INT_DISABLED;
	} else {
		dev_err(dev, "Invalid interrupt state\n");
		return -EINVAL;
	}

	ret = amg88xx_set_int_state(device, state);
	if (ret < 0) {
		dev_err(dev, "Failed to set interrupt state\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(interrupt_state,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   show_interrupt_state,
		   store_interrupt_state);

static ssize_t show_interrupt_levels(struct device *dev, struct device_attribute *attr,
				     char *buf)
{
	struct amg88xx *device;
	s16 upper;
	s16 lower;
	s16 hysteresis;
	int ret;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_int_upper_limit(device, &upper);
	if (ret < 0)
		return ret;

	ret = amg88xx_get_int_lower_limit(device, &lower);
	if (ret < 0)
		return ret;

	ret = amg88xx_get_int_hysteresis(device, &hysteresis);
	if (ret < 0)
		return ret;
	
	return scnprintf(buf, PAGE_SIZE, "%d,%d,%d\n", upper, lower, hysteresis);
}

static ssize_t store_interrupt_levels(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct amg88xx *device;
	char *temp;
	s16 values[3];
	int ret;
	int i;
	int index = 0;

	// Allocate a temporary buffer for substring handling
	temp = kmalloc(count, GFP_KERNEL);
	if (temp == NULL)
		return -ENOMEM;

	device = dev_get_drvdata(dev);
	
	for (i = 0; i < 3; i++) {
		const char *substr_end;
		size_t strl;

		/* 
		 * Calculate the length of the substring and copy it adding a null
		 * terminator to the end
		 */
		substr_end = strchrnul(&buf[index], ',');
		strl = substr_end - &buf[index];

		strncpy(temp, &buf[index], strl);
		temp[strl] = '\0';

		// Convert the value to u16 number and check for upper limit
		ret = kstrtos16(temp, 10, &values[i]);
		if (ret < 0) {
			dev_err(dev, "Failed to read value for %s from input\n",
			       i == 0 ? "upper limit" : (i == 1 ? "lower limit" : "hysteresis"));
			goto exit;
		}

		if (values[i] < -2048|| values[i] > 2047) {
			dev_err(dev, "Illegal input value for %s\n",
			       i == 0 ? "upper limit" : (i == 1 ? "lower limit" : "hysteresis"));
			ret = -EINVAL;
			goto exit;
		}

		index += strl + 1;
	}

	ret = amg88xx_set_int_upper_limit(device, values[0]);
	if (ret < 0) {
		dev_err(dev, "Failed to set the interrupt upper limit\n");
		goto exit;
	}

	ret = amg88xx_set_int_lower_limit(device, values[1]);
	if (ret < 0) {
		dev_err(dev, "Failed to set the interrupt lower limit\n");
		goto exit;
	}

	ret = amg88xx_set_int_hysteresis(device, values[2]);
	if (ret < 0) {
		dev_err(dev, "Failed to set the interrupt hysteresis\n");
		goto exit;
	}

	ret = count;
	
exit:
	kfree(temp);

	return ret;
}
static DEVICE_ATTR(interrupt_levels,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   show_interrupt_levels,
		   store_interrupt_levels);

static ssize_t show_interrupt(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct amg88xx *device;
	int ret;

	device = dev_get_drvdata(dev);

	ret = gpiod_get_value(device->int_gpio);
	if (ret < 0) {
		dev_err(dev, "Failed to read interrupt gpio value\n");
		return ret;
	}
	
	return scnprintf(buf, PAGE_SIZE, "%s\n", !ret ? "active" : "not_active");
}
static DEVICE_ATTR(interrupt, S_IRUGO, show_interrupt, NULL);

static ssize_t show_interrupt_map(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	struct amg88xx *device;
	int row;
	int col;
	int nwrite;
	int ret;
	u8 int_array[8];
	int index = 0;

	device = dev_get_drvdata(dev);

	ret = amg88xx_read_interrupt_map(device, int_array);
	if (ret < 0) {
		dev_err(dev, "Failed to read the interrupt map\n");
		return ret;
	}

	for (row = 0; row < 8; row++) {
		for (col = 0; col < 8; col++) {
			nwrite = scnprintf(&buf[index],
					   PAGE_SIZE - index,
					   col < 7 ? "%s, " : "%s\n",
					   int_array[row] & 1 << col ? "1" : "0");
			index += nwrite;
		}
	}

	return index;
}
static DEVICE_ATTR(interrupt_map, S_IRUGO, show_interrupt_map, NULL);

static ssize_t store_reset(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct amg88xx *device;
	enum amg88xx_reset_mode reset_mode;
	int ret;

	device = dev_get_drvdata(dev);

	if (sysfs_streq("full", buf)) {
		reset_mode = FULL_RST;
	} else if (sysfs_streq("partial", buf)) {
		reset_mode = PARTIAL_RST;
	} else {
		dev_err(dev, "Invalid reset mode\n");
		return -EINVAL;
	}

	ret = amg88xx_reset(device, reset_mode);
	if (ret < 0) {
		dev_err(dev, "Failed to reset device\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, store_reset);

static ssize_t show_framerate(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct amg88xx *device;
	int framerate;
	int ret;

	device = dev_get_drvdata(dev);

	ret = amg88xx_get_framerate(device, &framerate);
	if (ret < 0) {
		dev_err(dev, "Failed to read framerate\n");
		return ret;
	}

	return scnprintf(buf, PAGE_SIZE, "%d\n", framerate);
}

static ssize_t store_framerate(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct amg88xx *device;
	int fps;
	int ret;

	device = dev_get_drvdata(dev);

	ret = kstrtoint(buf, 10, &fps);
	if (ret < 0) {
		dev_err(dev, "Failed to read framerate from input\n");
		return ret;
	}

	if (!(fps == 1 ||
	      fps == 10)) {
		dev_err(dev, "Invalid framerate value\n");
		return -EINVAL;
	}

	ret = amg88xx_set_framerate(device, fps == 1 ? FPS1 : FPS10);
	if (ret < 0) {
		dev_err(dev, "Failed to set framerate\n");
		return ret;
	}

	return count;
}
static DEVICE_ATTR(framerate,
		   S_IRUGO | S_IWUSR | S_IWGRP,
		   show_framerate,
		   store_framerate);

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

	device->int_gpio = gpiod_get(&client->dev, "interrupt", GPIOD_IN);
	if (IS_ERR(device->int_gpio)) {
		dev_err(&client->dev, "Failed to get a gpio line for interrupt\n");
		return PTR_ERR(device->int_gpio);
	}

	ret = devm_request_threaded_irq(&client->dev,
					client->irq,
					NULL,
					irq_handler,
					IRQF_SHARED | IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
					client->name,
					device);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request a threaded irq\n");
		return ret;
	}

	dev_set_drvdata(&client->dev, device);

	ret = amg88xx_reset(device, PARTIAL_RST);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to reset device\n");
		return ret;
	}

	// TODO create sysfs entries
	// TODO move sysfs entries to attribute group
	device_create_file(&client->dev, &dev_attr_sensor);
	device_create_file(&client->dev, &dev_attr_thermistor);
	device_create_file(&client->dev, &dev_attr_device_mode);
	device_create_file(&client->dev, &dev_attr_interrupt_mode);
	device_create_file(&client->dev, &dev_attr_interrupt_state);
	device_create_file(&client->dev, &dev_attr_interrupt_levels);
	device_create_file(&client->dev, &dev_attr_interrupt);
	device_create_file(&client->dev, &dev_attr_interrupt_map);
	device_create_file(&client->dev, &dev_attr_reset);
	device_create_file(&client->dev, &dev_attr_framerate);

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
	device_remove_file(&client->dev, &dev_attr_interrupt_mode);
	device_remove_file(&client->dev, &dev_attr_interrupt_state);
	device_remove_file(&client->dev, &dev_attr_interrupt_levels);
	device_remove_file(&client->dev, &dev_attr_interrupt);
	device_remove_file(&client->dev, &dev_attr_interrupt_map);
	device_remove_file(&client->dev, &dev_attr_reset);
	device_remove_file(&client->dev, &dev_attr_framerate);

	gpiod_put(device->int_gpio);

	ret = amg88xx_set_dev_mode(device, SLEEP_MODE);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to put the device to sleep\n");
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
MODULE_AUTHOR("Iiro Vuorio <iiro.vuorio@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("A kernel driver for the Panasonic AMG88xx-series sensors.");
