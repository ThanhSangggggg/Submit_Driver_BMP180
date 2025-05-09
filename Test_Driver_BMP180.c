#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h> 

#define DEVICE_PATH "/dev/bmp180"
#define BMP180_IOCTL_MAGIC 'b'
#define BMP180_IOCTL_READ_TEMP     _IOR(BMP180_IOCTL_MAGIC, 1, int)
#define BMP180_IOCTL_READ_PRESS    _IOR(BMP180_IOCTL_MAGIC, 2, int)
#define BMP180_IOCTL_READ_ALTITUDE _IOR(BMP180_IOCTL_MAGIC, 3, int)
int main() {
    int fd;
    int data;

    // Open the device
    fd = open(DEVICE_PATH, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open the device");
        return errno;
    }

    // Read Temperature data
    if (ioctl(fd, BMP180_IOCTL_READ_TEMP, &data) < 0) {
        perror("Failed to read Temperature data");
        close(fd);
        return errno;
    }
    printf("Temperature:  %d\n", data);

    // Read Pressure data
    if (ioctl(fd, BMP180_IOCTL_READ_PRESS, &data) < 0) {
        perror("Failed to read Pressure data");
        close(fd);
        return errno;
    }
    printf("Pressure:  %d\n", data);

    // Read Altitude data
    if (ioctl(fd, BMP180_IOCTL_READ_ALTITUDE, &data) < 0) {
        perror("Failed to read Altitude data");
        close(fd);
        return errno;
    }
    printf("Altitude:  %d\n\n", data);

    // Close the device
    close(fd);
    return 0;
}
