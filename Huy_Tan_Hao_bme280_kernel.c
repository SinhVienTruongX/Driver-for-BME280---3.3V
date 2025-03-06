#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>

#define DRIVER_NAME "bme280_driver"
#define CLASS_NAME "bme280"
#define DEVICE_NAME "bme280"

// IOCTL commands
#define BME280_IOCTL_MAGIC 'm'
#define BME280_IOCTL_READ_T _IOR(BME280_IOCTL_MAGIC, 1, s32)
#define BME280_IOCTL_READ_P _IOR(BME280_IOCTL_MAGIC, 2, s32)
#define BME280_IOCTL_READ_H _IOR(BME280_IOCTL_MAGIC, 3, s32)
#define BME280_IOCTL_INIT_FORCED_MODE _IO(BME280_IOCTL_MAGIC, 4)
#define BME280_IOCTL_INIT_NORMAL_MODE _IO(BME280_IOCTL_MAGIC, 5)
#define BME280_IOCTL_CHECK_ADDR _IO(BME280_IOCTL_MAGIC, 6)
#define BME280_IOCTL_CHECK_STATUS _IO(BME280_IOCTL_MAGIC, 7)

static struct i2c_client *bme280_client;
static struct class* bme280_class = NULL;
static struct device* bme280_device = NULL;
static int major_number;

// Định nghĩa các địa chỉ thanh ghi của BME280
#define BME280_REG_TEMP_MSB   0xFA
#define BME280_REG_PRESS_MSB  0xF7
#define BME280_REG_HUM_MSB    0xFD

#define DATA_START_ADDR 0xF7
#define DATA_LENGTH 8

//Calibration register
#define CAL_DATA0_START_ADDR 0x88
#define CAL_DATA0_LENGTH 25
#define CAL_DATA1_START_ADDR 0xE1
#define CAL_DATA1_LENGTH 7

#define IDENT 0xD0
#define SOFT_RESET 0xE0
#define CTRL_HUM 0xF2
#define STATUS 0xF3
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5

s32 t_fine = 0;
static s32 BME280_compensate_T_int32(struct i2c_client *client, s32 adc_T)
{
    //s32 t_fine = 0;
    u16 dig_T1 = 0;
    s16 dig_T2 = 0;
    s16 dig_T3 = 0;
    u8 calData0[25];
    u8 calData1[7];
    s32 var1, var2, T;
    // read calibration data 
    i2c_smbus_read_i2c_block_data(client, CAL_DATA0_START_ADDR, CAL_DATA0_LENGTH, calData0);
    i2c_smbus_read_i2c_block_data(client, CAL_DATA1_START_ADDR, CAL_DATA1_LENGTH, calData1);

    // trimming parameters/
    dig_T1 = (calData0[1] << 8) | calData0[0];
    dig_T2 = (calData0[3] << 8) | calData0[2];
    dig_T3 = (calData0[5] << 8) | calData0[4];

    var1 = ((((adc_T>>3) - ((s32)dig_T1<<1))) * ((s32)dig_T2)) >> 11; 
    var2  = (((((adc_T>>4) - ((s32)dig_T1)) * ((adc_T>>4) - ((s32)dig_T1))) >> 12) *  ((s32)dig_T3)) >> 14;
    t_fine = var1 + var2; 
    T  = (t_fine * 5 + 128) >> 8; 
    return T; 
}
static u32 BME280_compensate_P_int32(struct i2c_client *client, s32 adc_P) 
{   
    u16 dig_P1 = 0;
    s16 dig_P2 = 0;
    s16 dig_P3 = 0;
    s16 dig_P4 = 0;
    s16 dig_P5 = 0;
    s16 dig_P6 = 0;
    s16 dig_P7 = 0;
    s16 dig_P8 = 0;
    s16 dig_P9 = 0;
    s32 var1, var2; 
    u32 p; 

    u8 calData0[25];
    u8 calData1[7];
    // read calibration data 
    i2c_smbus_read_i2c_block_data(client, CAL_DATA0_START_ADDR, CAL_DATA0_LENGTH, calData0);
    i2c_smbus_read_i2c_block_data(client, CAL_DATA1_START_ADDR, CAL_DATA1_LENGTH, calData1);
    // trimming parameters/
    dig_P1 = (calData0[7] << 8) | calData0[6];
    dig_P2 = (calData0[9] << 8) | calData0[8];
    dig_P3 = (calData0[11] << 8) | calData0[10];
    dig_P4 = (calData0[13] << 8) | calData0[12];
    dig_P5 = (calData0[15] << 8) | calData0[14];
    dig_P6 = (calData0[17] << 8) | calData0[16];
    dig_P7 = (calData0[19] << 8) | calData0[18];
    dig_P8 = (calData0[21] << 8) | calData0[20];
    dig_P9 = (calData0[23] << 8) | calData0[22];

    var1 = (((s32)t_fine)>>1) - (s32)64000; 
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((s32)dig_P6); 
    var2 = var2 + ((var1*((s32)dig_P5))<<1); 
    var2 = (var2>>2)+(((s32)dig_P4)<<16); 
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((s32)dig_P2) * 
    var1)>>1))>>18; 
    var1 =((((32768+var1))*((s32)dig_P1))>>15); 
    if (var1 == 0) 
    { 
        return 0; // avoid exception caused by division by zero 
    } 
    p = (((u32)(((s32)1048576)-adc_P)-(var2>>12)))*3125; 
    if (p < 0x80000000)  
    { 
        p = (p << 1) / ((u32)var1); 
    }  
    else  
    { 
        p = (p / (u32)var1) * 2; 
    } 
    var1 = (((s32)dig_P9) * ((s32)(((p>>3) * (p>>3))>>13)))>>12; 
    var2 = (((s32)(p>>2)) * ((s32)dig_P8))>>13; 
    p = (u32)((s32)p + ((var1 + var2 + dig_P7) >> 4)); 
    return p; 
}
static u32 bme280_compensate_H_int32(struct i2c_client *client, s32 adc_H) 
{ 
    s32 v_x1_u32r;

    u8 dig_H1 = 0;
    s16 dig_H2 = 0;
    u8 dig_H3 = 0;
    s16 dig_H4 = 0;
    s16 dig_H5 = 0;
    s8 dig_H6 = 0; 

    u8 calData0[25];
    u8 calData1[7];
    // read calibration data 
    i2c_smbus_read_i2c_block_data(client, CAL_DATA0_START_ADDR, CAL_DATA0_LENGTH, calData0);
    i2c_smbus_read_i2c_block_data(client, CAL_DATA1_START_ADDR, CAL_DATA1_LENGTH, calData1);

    dig_H1 = calData0[24];
    dig_H2 = (calData1[1] << 8) | calData1[0];
    dig_H3 = calData1[2];
    dig_H4 = (calData1[3] << 4) | (calData1[4] & 0xF);
    dig_H5 = (calData1[5] << 4) | (calData1[4] >> 4);
    dig_H6 = calData1[6];

    v_x1_u32r = (t_fine - ((s32)76800));
    v_x1_u32r = (((((adc_H << 14) - (((s32)dig_H4) << 20) - (((s32)dig_H5) * v_x1_u32r)) + ((s32)16384)) >> 15) * (((((((v_x1_u32r * 
    ((s32)dig_H6)) >> 10) * (((v_x1_u32r * ((s32)dig_H3)) >> 11) + ((s32)32768))) >> 10) + ((s32)2097152)) * ((s32)dig_H2) + 8192) >> 14)); 
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((s32)dig_H1)) >> 4)); 
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);  
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);  
    return (u32)(v_x1_u32r>>12); 
}
static int bme280_init(struct i2c_client *client, int mode)
{
    int ret;
    ret = i2c_smbus_write_byte_data(client, SOFT_RESET, 0xB6);
    if (ret < 0) {
        printk(KERN_ERR "Failed to reset the BME280 sensor\n");
        return ret;
    }
    msleep(50);

    // Humid config
    ret = i2c_smbus_write_byte_data(client, CTRL_HUM, 0x1);
    if (ret < 0) {
        printk(KERN_ERR "Failed to set humidity oversampling\n");
        return ret;
    }
    // Pressure and Humidity config depend on mode
    if (mode == 0) { // Forced mode
        ret = i2c_smbus_write_byte_data(client, CTRL_MEAS, 0x25);
        if (ret < 0) {
            printk(KERN_ERR "Failed to set temperature and pressure oversampling and forced mode\n");
            return ret;
        }
    } else if (mode == 1) { // Normal mode
        ret = i2c_smbus_write_byte_data(client, CTRL_MEAS, 0x27);
        if (ret < 0) {
            printk(KERN_ERR "Failed to set temperature and pressure oversampling and normal mode\n");
            return ret;
        }
    } else {
        printk(KERN_ERR "Invalid mode selected\n");
        return -EINVAL; // Invalid argument
    }
    // Thiết lập các tham số cấu hình khác (nếu cần)
    ret = i2c_smbus_write_byte_data(client, CONFIG, 0x0);
    if (ret < 0) {
        printk(KERN_ERR "Failed to set configuration register\n");
        return ret;
    }
    
    return 0;
}

static s32 bme280_read_raw_value(struct i2c_client *client, int axis)
{
    u8 dataBlock[8];
    s32 data[3];

    if (i2c_smbus_read_i2c_block_data(client, DATA_START_ADDR, DATA_LENGTH, dataBlock) < 0) {
        printk(KERN_ERR "Failed to read data\n");
        return -EIO;
    }

    i2c_smbus_write_byte_data(client, CTRL_MEAS, 0x25);
    /* get raw temp */
    data[0] = (dataBlock[3] << 16 | dataBlock[4] << 8 | dataBlock[5]) >> 4;
    /* get raw pressure */
    data[1] = (dataBlock[0] << 16 | dataBlock[1] << 8 | dataBlock[2]) >> 4;
    /* get raw humidity */
    data[2] = dataBlock[6] << 8 | dataBlock[7];
    return data[axis];
}
static int bme280_check_i2c_address(struct i2c_client *client)
{
    int ret;
    u8 id;

    ret = i2c_smbus_read_byte_data(client, IDENT);
    id = (u8)ret;
    // Kiểm tra mã nhận diện, giá trị này tùy thuộc vào thiết bị cụ thể
    if (id != 0x60) {
        printk(KERN_ERR "I2C device not found or incorrect ID\n");
        return -ENODEV;
    }

    return 0;
}
static int bme280_check_status(struct i2c_client *client) {
    u8 ret;
    // Đọc giá trị từ thanh ghi STATUS của BME280
    ret = i2c_smbus_read_byte_data(client, STATUS);
    if ((ret & 0x9) != 0) { //Check bit 3 and bit 0 of STATUS register
        printk(KERN_ERR "Error to read data\n");
        return -ENODEV;
    }
    return 0; 
}
static long bme280_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    s32 raw_value;
    s32 temp;
    u32 press;
    u32 humid;
    switch (cmd) {
        case BME280_IOCTL_READ_T:
            // Đọc nhiệt độ từ cảm biến và lưu vào biến value
            raw_value = bme280_read_raw_value(bme280_client, 0);
            temp = BME280_compensate_T_int32(bme280_client, raw_value);
            if (copy_to_user((int __user *)arg, &temp, sizeof(temp))) {
                return -EFAULT;
            }
            break;
        case BME280_IOCTL_READ_P:
            // Đọc áp suất từ cảm biến và lưu vào biến value
            raw_value = bme280_read_raw_value(bme280_client, 1);
            press = BME280_compensate_P_int32(bme280_client, raw_value);
            if (copy_to_user((int __user *)arg, &press, sizeof(press))) {
                return -EFAULT;
            }
            break;
        case BME280_IOCTL_READ_H:
            // Đọc độ ẩm từ cảm biến và lưu vào biến value
            raw_value = bme280_read_raw_value(bme280_client, 2);
            humid = bme280_compensate_H_int32(bme280_client, raw_value);
            if (copy_to_user((int __user *)arg, &humid, sizeof(humid))) {
                return -EFAULT;
            }
            break;
        case BME280_IOCTL_INIT_FORCED_MODE:
            return bme280_init(bme280_client, 0);
        case BME280_IOCTL_INIT_NORMAL_MODE:
            return bme280_init(bme280_client, 1);
        case BME280_IOCTL_CHECK_ADDR:
            return bme280_check_i2c_address(bme280_client);
        case BME280_IOCTL_CHECK_STATUS:
            return  bme280_check_status(bme280_client);
        default:
            return -EINVAL;
    }
    return 0;
}
static int bme280_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BME280 device opened\n");
    return 0;
}

static int bme280_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BME280 device closed\n");
    return 0;
}
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = bme280_open,
    .unlocked_ioctl = bme280_ioctl,
    .release = bme280_release,
};
static int bme280_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    //s32 temp_raw, press_raw, hum_raw;
    //int ret;
    bme280_client = client;
    // Create a char device
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    bme280_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bme280_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to register device class\n");
        return PTR_ERR(bme280_class);
    }

    bme280_device = device_create(bme280_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bme280_device)) {
        class_destroy(bme280_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to create the device\n");
        return PTR_ERR(bme280_device);
    }

    printk(KERN_INFO "BME280 driver installed\n");
    return 0;
}

static void bme280_remove(struct i2c_client *client)
{
    device_destroy(bme280_class, MKDEV(major_number, 0));
    class_unregister(bme280_class);
    class_destroy(bme280_class);
    unregister_chrdev(major_number, DEVICE_NAME);

    printk(KERN_INFO "BME280 driver removed\n");
}
static const struct of_device_id bme280_of_match[] = {
    { .compatible = "bosch,bme280", },
    { },
};
MODULE_DEVICE_TABLE(of,bme280_of_match);

static struct i2c_driver bme280_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(bme280_of_match),
    },
    .probe      = bme280_probe,
    .remove     = bme280_remove,
};
static int __init bme280_driver_init(void)
{
    printk(KERN_INFO "Initializing BME280 driver\n");
    return i2c_add_driver(&bme280_driver);
}

static void __exit bme280_driver_exit(void)
{
    printk(KERN_INFO "Exiting BME280 driver\n");
    i2c_del_driver(&bme280_driver);
}

module_init(bme280_driver_init);
module_exit(bme280_driver_exit);

MODULE_AUTHOR("Its me");
MODULE_DESCRIPTION("BME280 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("GPL");
