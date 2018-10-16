# AMG88xx-series sensor device driver
This is a simple Linux device driver for the Panasonic amg88xx-series thermal cameras.
**This project is still in a very early stage!**

The amg88xx sensors use i2c-interface for communication and a single interrupt line.
This driver exposes the i2c-registers to userspace via sysfs. Some extra features are
also exposed please take a look at the _sysfs interface_ chapter for more info.

## TODO
- [ ] add support for the device interrupt
- [ ] complete the sysfs interface

## Howto use
git clone
make, need to have headers
compile dtoverlay
load dtoverlay
load module
profit

## sysfs interface
**The sysfs interface is not fully finished!**
This driver exposes the following sysfs files:
 * `device_mode` RW, hex
  * reading this file returns the device mode in hex format
  * writing a correct hex value to this file sets the device mode
 * `thermistor` read-only, dec
  * reading this file returns the thermistor output in dec format
 * `sensor` read-only, dec
  * reading this file returns the 8x8 array containing the sensor values
