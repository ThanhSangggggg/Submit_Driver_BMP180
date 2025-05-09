#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/math64.h>  

#define DRIVER_NAME "bmp180_driver"
#define CLASS_NAME "bmp180_class"
#define DEVICE_NAME "bmp180"

// Bien Đọc hệ số hiệu chuẩn
 struct bmp180_calibration_data {
    int16_t AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t B1, B2;
    int16_t MB, MC, MD;
};
static struct bmp180_calibration_data calib_data;

// IOCTL commands
#define BMP180_IOCTL_MAGIC 'b'
#define BMP180_IOCTL_READ_TEMP     _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS    _IOR(BMP180_IOCTL_MAGIC, 2, int)
#define BMP180_IOCTL_READ_ALTITUDE _IOR(BMP180_IOCTL_MAGIC, 3, int)

static struct i2c_client *bmp180_client;
static struct class* bmp180_class = NULL;
static struct device* bmp180_device = NULL;
static int major_number;

// Kiểm tra ID cảm biến 
static int bmp180_init_sensor(struct i2c_client *client) {
    uint8_t reg = 0xD0;  
    uint8_t id;
    int ret;

    ret = i2c_master_send(client, &reg, 1);
    if (ret < 0) {
        printk(KERN_ERR "Failed to send register address to sensor\n");
        return -EIO;
    }

    ret = i2c_master_recv(client, &id, 1);
    if (ret < 0) {
        printk(KERN_ERR "Failed to read sensor ID\n");
        return -EIO;
    }

    if (id != 0x55) {
        printk(KERN_ERR "Incorrect sensor ID: %x\n", id);
        return -ENODEV;
    }

    printk(KERN_INFO "BMP180 sensor initialized, ID: %x\n", id);
    return 0;
}

int bmp180_read_calibration_data(struct i2c_client *client) {
    uint8_t buf[22];
    int ret;

    // Đọc 22 byte từ các thanh ghi hiệu chuẩn (0xAA đến 0xBF)
    // Địa chỉ thanh ghi bắt đầu laf 0xAA
    ret = i2c_smbus_read_i2c_block_data(client, 0xAA, 22, buf);
    if (ret != 22) {
        printk(KERN_ERR "Failed to read calibration data\n");
        return -EIO;
    }

    // Lưu các hệ số vào cấu trúc calib_data
    calib_data.AC1 = (int16_t)((buf[0] << 8) | buf[1]);
    calib_data.AC2 = (int16_t)((buf[2] << 8) | buf[3]);
    calib_data.AC3 = (int16_t)((buf[4] << 8) | buf[5]);
    calib_data.AC4 = (uint16_t)((buf[6] << 8) | buf[7]);
    calib_data.AC5 = (uint16_t)((buf[8] << 8) | buf[9]);
    calib_data.AC6 = (uint16_t)((buf[10] << 8) | buf[11]);
    calib_data.B1  = (int16_t)((buf[12] << 8) | buf[13]);
    calib_data.B2  = (int16_t)((buf[14] << 8) | buf[15]);
    calib_data.MB  = (int16_t)((buf[16] << 8) | buf[17]);
    calib_data.MC  = (int16_t)((buf[18] << 8) | buf[19]);
    calib_data.MD  = (int16_t)((buf[20] << 8) | buf[21]);

    printk(KERN_INFO "Calibration data read successfully\n");

    return 0;  
}

// Đọc nhiệt độ thô
static int bmp180_read_raw_temperature(struct i2c_client *client) {
    uint8_t reg;
    uint8_t buf[2];
    int16_t raw_temp;

    // Gửi lệnh bắt đầu đo nhiệt độ (ghi 0x2E vào thanh ghi 0xF4)
    uint8_t cmd[2] = {0xF4, 0x2E};
    if (i2c_master_send(client, cmd, 2) < 0) {
        printk(KERN_ERR "Failed to write temperature command\n");
        return -EIO;
    }

    // Chờ ít nhất 4.5ms (BMP180 cần thời gian đo)
    msleep(5);

    // Gửi địa chỉ thanh ghi bắt đầu đọc dữ liệu (0xF6)
    reg = 0xF6;
    if (i2c_master_send(client, &reg, 1) < 0) {
        printk(KERN_ERR "Failed to write register address 0xF6\n");
        return -EIO;
    }

    // Đọc 2 byte dữ liệu nhiệt độ
    if (i2c_master_recv(client, buf, 2) < 0) {
        printk(KERN_ERR "Failed to read temperature data\n");
        return -EIO;
    }

    raw_temp = (int16_t)((buf[0] << 8) | buf[1]);

    printk(KERN_INFO "Raw temperature: %d\n", raw_temp);
    return raw_temp;
}

// Doc ap suat tho
static int bmp180_read_raw_pressure(struct i2c_client *client) {
    uint8_t cmd[2] = {0xF4, 0x34};
    uint8_t reg = 0xF6;
    uint8_t buf[3];
    int32_t raw_press;
    int delay_ms = 5;

    if (i2c_master_send(client, cmd, 2) < 0) {
        printk(KERN_ERR "Failed to send pressure command\n");
        return -EIO;
    }

    msleep(delay_ms);

    if (i2c_master_send(client, &reg, 1) < 0) {
        printk(KERN_ERR "Failed to send register address\n");
        return -EIO;
    }

    if (i2c_master_recv(client, buf, 3) < 0) {
        printk(KERN_ERR "Failed to read raw pressure\n");
        return -EIO;
    }

    raw_press = (((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2]) >> 8 ;
    printk(KERN_INFO "Raw pressure : %d\n", raw_press);
    return raw_press;
}

// Tinh B5
static int B5;  // thêm biến toàn cục

static int bmp180_get_B5(int raw_temp) {
    int X1, X2;

    X1 = ((raw_temp - calib_data.AC6) * calib_data.AC5) >> 15;
    X2 = (calib_data.MC << 11) / (X1 + calib_data.MD);
    B5 = X1 + X2;

    return B5;
}
// Tính nhiệt độ thực tế
static int bmp180_calculate_temperature(int raw_temp) {

    int B5 = bmp180_get_B5(raw_temp);
    int temperature = (B5 + 8) >> 4;  // Tính T trong đơn vị 0.1°C
    
    printk(KERN_INFO "Temperature: %d\n", temperature);
    return temperature;  // Trả về nhiệt độ thực tế
}

// Tính áp suất thực tế
static int bmp180_calculate_pressure(int raw_press, int B5) {
    int B6, X1, X2, X3, B3, B4, B7;
    uint32_t pressure;
    
    // Tính B6 và các giá trị trung gian
    B6 = B5 - 4000;
    X1 = (calib_data.B2 * (B6 * B6) >> 12) >> 11;
    X2 = calib_data.AC2 * B6 >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)calib_data.AC1) * 4 + X3) + 2) >> 2;

    X1 = calib_data.AC3 * B6 >> 13;
    X2 = (calib_data.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = calib_data.AC4 * (unsigned int)(X3 + 32768) >> 15;

    B7 = (unsigned int)(raw_press - B3) * 50000;
    
    // Tính áp suất thực tế
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    } else {
        pressure = (B7 / B4) * 2;
    }

    X1 = (pressure >> 8) * (pressure >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * pressure) >> 16;

    // Tính kết quả cuối cùng
    pressure = pressure + ((X1 + X2 + 3791) >> 4);
    
    printk(KERN_INFO "Pressure: %d\n", pressure);
    return pressure;  // Trả về áp suất thực tế
}

// Đọc nhiệt độ hoàn chỉnh
static int bmp180_read_temperature(struct i2c_client *client) {
    int raw_temp;
    raw_temp = bmp180_read_raw_temperature(client);  // Đọc nhiệt độ thô
    return bmp180_calculate_temperature(raw_temp);   // Tính và trả về nhiệt độ thực tế
}

// Đọc áp suất hoàn chỉnh
static int bmp180_read_pressure(struct i2c_client *client) {
    int32_t raw_temp = bmp180_read_raw_temperature(client);
    int32_t B5 = bmp180_get_B5(raw_temp);
    int32_t raw_press = bmp180_read_raw_pressure(client);
    
    if (raw_press < 0) return raw_press;  

    return bmp180_calculate_pressure(raw_press, B5);
}

// Tính độ cao từ áp suất
static int bmp180_calculate_altitude(int pressure) {
    int altitude = (44330 * (101325 - pressure)) / 101325;
    printk(KERN_INFO "Altitude: %d meters\n", altitude);
    return altitude;
}

// IOCTL
static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int data =0;
    switch (cmd) {
        case BMP180_IOCTL_READ_TEMP: {
            int temp = bmp180_read_temperature(bmp180_client);
            data = temp;
            break;
        }
        case BMP180_IOCTL_READ_PRESS: {
            int press = bmp180_read_pressure(bmp180_client);
            data = press;
            break;
        }
        case BMP180_IOCTL_READ_ALTITUDE: {
            int raw_temp = bmp180_read_raw_temperature(bmp180_client);
            int B5 = bmp180_get_B5(raw_temp);
            int raw_press = bmp180_read_raw_pressure(bmp180_client);
            int pressure = bmp180_calculate_pressure(raw_press, B5);
            data = bmp180_calculate_altitude(pressure);
            break;
        }
        default:
            return -EINVAL;
        }

    if (copy_to_user((int __user *)arg, &data, sizeof(data))) {
        return -EFAULT;
    }

    return 0;
}

static int bmp180_open(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inodep, struct file *filep)
{
    printk(KERN_INFO "BMP180 device closed\n");
    return 0;
}

static struct file_operations fops = {
    .owner            = THIS_MODULE,
    .open             = bmp180_open,
    .unlocked_ioctl   = bmp180_ioctl,
    .release          = bmp180_release,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    bmp180_client = client;

    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    bmp180_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp180_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to register device class\n");
        return PTR_ERR(bmp180_class);
    }

    bmp180_device = device_create(bmp180_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) {
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to create the device\n");
        return PTR_ERR(bmp180_device);
    }

    if (bmp180_init_sensor(client) < 0) {
        printk(KERN_ERR "Sensor init failed\n");
        return -ENODEV;
    }
    
    if (bmp180_read_calibration_data(client) < 0) {
        printk(KERN_ERR "Calibration read failed\n");
        return -EIO;
    }
    

    printk(KERN_INFO "BMP180 driver installed\n");
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);

    printk(KERN_INFO "BMP180 driver removed\n");
}

// ====  ID table và driver struct ====
static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "bosch,bmp180"},
    { },
};
MODULE_DEVICE_TABLE(of, bmp180_of_match);

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(bmp180_of_match),
    },
    .probe = bmp180_probe,
    .remove = bmp180_remove,
};

// ==== init/exit ====
static int __init bmp180_init(void) {
    printk(KERN_INFO "Initializing BMP180 driver\n");
    return i2c_add_driver(&bmp180_driver);
}

static void __exit bmp180_exit(void) {
    printk(KERN_INFO "Exiting BMP180 driver\n");
    i2c_del_driver(&bmp180_driver);
}

module_init(bmp180_init);
module_exit(bmp180_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("SPQA");
MODULE_DESCRIPTION("BMP180 I2C Client Driver with IOCTL Interface");