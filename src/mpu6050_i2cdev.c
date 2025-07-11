#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <endian.h>
#include <linux/i2c-dev.h>

#include "mpu6050_i2cdev.h"
#include <mpu6050.h>

static int mpu6050_i2cdev_read_raw(mpu6050_t *mpu6050)
{
    int16_t *acc_raw = mpu6050->acc.raw;
    int16_t *gyro_raw = mpu6050->gyro.raw;
    
	char data[14];
	char reg[1] = {ACCEL_XOUT_H};

	int ret;		
	ret = write(mpu6050->fd, reg, 1);
	if (unlikely(ret != 1)) {
		perror("i2cdev_read_raw, write err");
        return -1;
	}
		
	ret = read(mpu6050->fd, data, 14);
	if (unlikely(ret != 14)) {
		perror("i2cdev_read_raw, read err");
        return -1;
	}

    for (int i = 0; i < 3; i++) {
        int idx = 2 * i;

        acc_raw[i] = (data[idx] << 8) | data[idx + 1];
        gyro_raw[i] = (data[idx + 8] << 8) | data[idx + 9];
    }

    return 0;
}

int mpu6050_i2cdev_init(mpu6050_t *mpu6050, unsigned int dev)
{
    int ret;
	int fd;
	char path[20];
    
	memset(mpu6050, 0, sizeof(mpu6050_t));

	sprintf(path, "/dev/i2c-%d", dev);
	ret = open_fd(fd, path, O_RDWR);
    if (unlikely(ret)) {
        return ret;
    }

	mpu6050->fd = fd;

    // Set the I2C address for the MPU6050
    if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        perror("Failed to connect to MPU6050 sensor");
        return -1;
    }

    // Reset device
    char config[2] = {PWR_MGMT_1, 0x40};
    if (write(fd, config, 2) != 2) {
        perror("Failed to reset MPU6050");
        return -1;
    }

    // Initialize MPU6050: Set Power Management 1 register to 0 to wake up the sensor
    config[1] = 0;
    if (write(fd, config, 2) != 2) {
        perror("Failed to initialize MPU6050");
        return -1;
    }

    // Set DLPF: 21Hz bandwidth (Configuration 2)
    char dlpf_config[2] = {CONFIG, 0x04}; // 0x03 sets DLPF to Configuration 2
    if (write(fd, dlpf_config, 2) != 2) {
        perror("Failed to set DLPF");
        return -1;
    }

    // Set Gyroscope sensitivity: ±500°/s (Configuration 0)
    char gyro_config[2] = {GYRO_CONFIG, 0x08}; 
    if (write(fd, gyro_config, 2) != 2) {
        perror("Failed to set gyroscope configuration");
        return -1;
    }

    // Set Accelerometer sensitivity: ±8g (Configuration 0)
    char accel_config[2] = {ACCEL_CONFIG, 0x10}; 
    if (write(fd, accel_config, 2) != 2) {
        perror("Failed to set accelerometer configuration");
        return -1;
    }

    // Set sample rate divider for 1 kHz sample rate
    char smplrt_div_config[2] = {SMPLRT_DIV, 0}; // 0 sets sample rate to 1 kHz
    if (write(fd, smplrt_div_config, 2) != 2) {
        perror("Failed to set sample rate divider");
        return -1;
	}
	
    mpu6050->read_raw = mpu6050_i2cdev_read_raw;

    mpu6050->acc.scale = (1.0f / ACC_FS_SENSITIVITY);
    mpu6050->gyro.scale = (1.0f / GYRO_FS_SENSITIVITY);

	return 0;
}