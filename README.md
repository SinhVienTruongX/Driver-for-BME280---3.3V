BME280 Sensor Driver and User Application

Overview

This project is a Linux kernel driver and user-space application for the BME280 sensor, which measures temperature, pressure, and humidity. The project was developed as part of an embedded systems course.

Authors

Lâm Gia Hào (21146222)

Bùi Phước Huy (21146233)

Phan Nhật Tân (21146310)

Features

Custom Linux kernel driver for BME280 with IOCTL commands

User-space application to read sensor data

Supports I2C communication

Provides temperature, pressure, and humidity readings

Project Structure

├── Huy_Tan_Hao_bme280_kernel.c   # Kernel driver source code
├── Huy_Tan_Hao_bme280_test.c     # User-space application
├── Final_Manual.pdf              # Project documentation

Installation and Usage

1. Building and Installing the Kernel Module

make
sudo insmod bme280_driver.ko

Check if the driver is loaded:

dmesg | grep BME280

2. Running the User Application

gcc -o bme280_test Huy_Tan_Hao_bme280_test.c
sudo ./bme280_test

IOCTL Commands

The following IOCTL commands are supported:

BME280_IOCTL_READ_T - Read temperature

BME280_IOCTL_READ_P - Read pressure

BME280_IOCTL_READ_H - Read humidity

BME280_IOCTL_INIT_FORCED_MODE - Initialize sensor in forced mode

BME280_IOCTL_INIT_NORMAL_MODE - Initialize sensor in normal mode

BME280_IOCTL_CHECK_ADDR - Check device address

BME280_IOCTL_CHECK_STATUS - Check sensor status

License

This project is released under the GPL License.

Documentation

Refer to Final_Manual.pdf for detailed information about the sensor, driver implementation, and usage instructions.

Contact

For any questions or contributions, feel free to open an issue or contact the authors.
