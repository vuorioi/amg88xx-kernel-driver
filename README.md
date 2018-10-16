# AMG88xx-series sensor device driver
This is a simple Linux device driver for the Panasonic amg88xx-series thermal cameras.
**This project is still in a very early stage!**

The amg88xx sensors use i2c-interface and a single interrupt line for communication.
This driver exposes the i2c registers to userspace via sysfs. Some extra features are
also exposed, please take a look at the _sysfs interface_ chapter for more info.

## TODO
- [ ] add support for the device interrupt
- [ ] complete the sysfs interface
- [ ] finish the device tree overlay

## Howto use
First clone this reposity:
```git clone https://github.com/vuorioi/amg88xx-kernel-driver.git```

Then use the supplied `Makefile` to build the driver. Make sure that you have the proper
kernel headers installed.
```cd amg88xx-kernel-driver && make```

In order for this driver to function properly the device tree needs to have an entry
for the amg88xx sensor. This repo provides a sample device tree overlay for Raspberry.

Compile the device tree overlay with `dtc`:
```dtc -W no-unit_address_vs_reg -O dtb -I dts -o amg88xx.dtbo amg88xx-overlay.dts```
And load it with `dtoverlay`:
```sudo dtoverlay amg88xx.dtbo```

Finally load the module:
```sudo insmod amg88xx.ko```

The sysfs entries are found in the `/sys/bus/i2c/device/<device_name>/` directory. You can find
the right `device_name` by running the following command:
```cd /sys/bus/i2c/devices/ && ls * | grep amg88xx */name```

Before the device can be used it must be set to _normal_ or one of the _stand-by_ modes:
```sudo sh -c "echo <mode> > device_mode"```
`mode` can be any on of the following:
 * 0x0 Normal mode
 * 0x10 Sleep mode
 * 0x20 Stand-by mode (60 s)
 * 0x21 Stand-by mode (10 s)

## sysfs interface
**The sysfs interface is not fully implemented!**
This driver exposes the following sysfs files:
 * `device_mode` i2c register: PCTL
   * reading this file returns the device mode in hex encoding
   * writing a correct hex encoded value to this file sets the device mode
 * `thermistor` i2c registers: TTHL and TTHH.
   * reading this file returns the thermistor output in hex encoding
 * `sensor` i2c registers: T01L to 0xFF
   * reading this file returns the 8x8 array containing the sensor values in hex encoding
