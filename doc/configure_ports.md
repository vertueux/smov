# Configure ports

Here are the steps to follow to configure your device to control a microcontroller with I²C and possibly a second microcontroller, as well as the LCD panel.

For the configuration of the first microcontroller, we assume that you have connected the cables enabling configuration between your device and the microcontroller.

## Configure Ubuntu

Make sure that I²C is enabled by default. Check the file */boot/firmware/syscfg.txt* and see if you have the following line:

```txt
dtparam=i2c_arm=on
```

If not, maybe you can append them on */boot/firmware/usercfg.txt* and reboot, and hopefully, that works. If that doesn't work, maybe do `sudo apt update && sudo full-upgrade -y`  and see if there are any distro updates needed.

* Refer to [this post](https://askubuntu.com/questions/1273700/enable-spi-and-i2c-on-ubuntu-20-04-raspberry-pi/1273900#1273900).

You can also add the following line to */etc/modules*:

```bash
i2c-dev
```

* Refer to [this post](https://raspberrypi.stackexchange.com/questions/61905/enable-i2c-on-ubuntu-mate-raspberry-pi-3).

---

On a RPi, with `raspi-config`, you can enable I²C by navigating to *Interface Options->Advanced->I²C* and then enable it.

If you're not using a second microcontroller and you don't have an LCD panel, you can go straight to [the next step](calibrate_servos.md).

## Testing I²C

Now when you log in you can type the following command to see all the connected devices:

```bash
sudo i2cdetect -y 1 # Or 0, depends on the device you use.
```

## 1st Method: Configuring the Second I²C Port (Optional)

If your robot uses two microcontrollers, then you can figure the second I²C by following the instructions below.

We first have to edit */boot/config.txt*:

```bash
sudo nano /boot/config.txt
```

If we scroll to the bottom we may find the remnants of the precious peripheral (if it was added here previously):

```bash
dtoverlay=i2c-rtc,pcf8523,wakeup-source # RTC
```

We now need to add an additional device under a new bus — e.g. bus 2 or 4 (I personally used bus 4). We do this by adding an additional line with the content below:
You can modify the values according to the ports & bus you have chosen for your Raspberry Pi.

```bash
dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=23,i2c_gpio_scl=24 # 23 & 24 can be changed to the port you want.
```

## 2nd Method: Configuring the Second I²C Port (Optional)

Here's a second method for configuring a second port, if you don't want to set this port as the default when you start your device.
When using a Raspberry Pi, you must first have dtoverlay installed for the manual configuration. Install it by copying this command:

```bash
sudo apt install libraspberrypi-bin
```

If we do not want to change any files we can tell our Raspberry Pi to load the port through a shell script with the following command. This method is easier for optional loads or prototyping. The dtoverlay the command is similar to that of the config.txt file.

```bash
sudo dtoverlay i2c-gpio bus=2 i2c_gpio_sda=23 i2c_gpio_scl=24 
```

* Refer to [this post](https://medium.com/cemac/creating-multiple-i2c-ports-on-a-raspberry-pi-e31ce72a3eb2)

> Note that there are [other ways to acccomplish this](https://www.quora.com/How-do-I-connect-two-PCA9685-servo-controllers-to-Raspberry-Pi).

## Configuring the LCD I²C Port (Optional)

To configure the LCD, you'll need to set up a new I²C port, in the same way as for configuring the second microcontroller input.
After selecting the SDA, SCL port & bus to be configured, enter this command, modifying it as required, Here's an example (I personally chosed bus 3):

```bash
sudo dtoverlay i2c-gpio bus=3 i2c_gpio_sda=10 i2c_gpio_scl=9 
```

## Surpress Python setup.py deprecated warnings

ROS2 Humble still uses the old `setup.py` method of installing packages within its enviornment. This has been deprecated in python 3.10.
If you are getting these warnings you can safely ignore them but if you want to surpress them you can downgrade your version of setup tools
doing the following.

```bash
pip install setuptools==58.2.0
```

**Next step**: [Calibrate servos](calibrate_servos.md)
