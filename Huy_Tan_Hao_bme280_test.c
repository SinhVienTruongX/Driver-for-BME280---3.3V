#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h> // Include errno header
#include <i2c/smbus.h>
#include <math.h>
#include <stdint.h>
#include <linux/i2c-dev.h>

#define DEVICE_PATH "/dev/bme280"
#define DEV_ID 0x76
// IOCTL commands
#define BME280_IOCTL_MAGIC 'm'
#define BME280_IOCTL_READ_T _IOR(BME280_IOCTL_MAGIC, 1, uint32_t)
#define BME280_IOCTL_READ_P _IOR(BME280_IOCTL_MAGIC, 2, uint32_t)
#define BME280_IOCTL_READ_H _IOR(BME280_IOCTL_MAGIC, 3, uint32_t)
#define BME280_IOCTL_INIT_FORCED_MODE _IO(BME280_IOCTL_MAGIC, 4)
#define BME280_IOCTL_INIT_NORMAL_MODE _IO(BME280_IOCTL_MAGIC, 5)
#define BME280_IOCTL_CHECK_ADDR _IO(BME280_IOCTL_MAGIC, 6)
#define BME280_IOCTL_CHECK_STATUS _IO(BME280_IOCTL_MAGIC, 7)

int main() {
    int fd;
    uint8_t id;
    int32_t temp_int = 0;
    int32_t press_int = 0;
    uint32_t hum_int = 0;
    // Open the device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open the device");
        return errno;
    }
    printf("Successfully open device\n");
    //Check ID
    id = ioctl(fd, BME280_IOCTL_CHECK_ADDR);
    if (id < 0) {
        perror("Unable to configure i2c slave device");
        close(fd);
        return 2;
    }
    
    //Init sensor
    if (ioctl(fd, BME280_IOCTL_INIT_FORCED_MODE) < 0) {
        perror("Failed to initialize the sensor");
        close(fd);
        return errno;
    }
    printf("Sensor initialized successfully\n");
    //-----------------------------------------------------------------------------
    while(1){
        sleep(1);
        // Kiểm tra STATUS. Neu dat gia tri 0x00 thi OK
        if (ioctl(fd, BME280_IOCTL_CHECK_STATUS) < 0) {
            perror("Data is not ready");
            close(fd);
        }
        // Read Temperature
        if (ioctl(fd, BME280_IOCTL_READ_T, &temp_int) < 0) {
            perror("Failed to read temparature data");
            close(fd);
            return errno;
        }
        printf("Temparature: %.2f°C\n", temp_int/100.0);
        // Read Pressure
        if (ioctl(fd, BME280_IOCTL_READ_P, &press_int) < 0) {
            perror("Failed to read pressure data");
            close(fd);
            return errno;
        }
        printf("Pressure: %.2f hPa\n", press_int/100.0);
        // Read Humidity
        if (ioctl(fd, BME280_IOCTL_READ_H, &hum_int) < 0) {
            perror("Failed to read humidity data");
            close(fd);
            return errno;
        }
        printf("Humidity: %.2f %%rH\n", hum_int/1024.0);
        //printf("Humidity: %d %%rH\n", hum_int);
        
    }
    // Close the device
    close(fd);
    return 0;
}