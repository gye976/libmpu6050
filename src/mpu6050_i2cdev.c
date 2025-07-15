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

static int mpu6050_i2cdev_read_hw(mpu6050_t *mpu6050, int16_t *dest)
{
 	char data[14];
	char reg[1] = {ACCEL_XOUT_H};
	int ret;		
	int i = 0;

	ret = write(mpu6050->fd, reg, 1);
	if (unlikely(ret != 1)) {
		perror("i2cdev_read_hw, write err");
		return -1;
	}
		
	ret = read(mpu6050->fd, data, 14);
	if (unlikely(ret != 14)) {
		perror("i2cdev_read_raw, read err");
		return -1;
	}

	for (i = 0; i < 3; i++) {
		int idx = i * 2;

		dest[i] = (data[idx] << 8) | data[idx + 1];
		dest[i + 3] = (data[idx + 8] << 8) | data[idx + 9];
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

	if (ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
		perror("Failed to connect to MPU6050 sensor");
		return -1;
	}
	char config[2] = {PWR_MGMT_1, 0x40};
	if (write(fd, config, 2) != 2) {
		perror("Failed to reset MPU6050");
		return -1;
	}
	config[1] = 0;
	if (write(fd, config, 2) != 2) {
		perror("Failed to initialize MPU6050");
		return -1;
	}
	char dlpf_config[2] = {CONFIG, 0x04}; // 0x03 sets DLPF to Configuration 2
	if (write(fd, dlpf_config, 2) != 2) {
		perror("Failed to set DLPF");
		return -1;
	}
	char gyro_config[2] = {GYRO_CONFIG, 0x08}; 
	if (write(fd, gyro_config, 2) != 2) {
		perror("Failed to set gyroscope configuration");
		return -1;
	}
	char accel_config[2] = {ACCEL_CONFIG, 0x10}; 
	if (write(fd, accel_config, 2) != 2) {
		perror("Failed to set accelerometer configuration");
		return -1;
	}
	char smplrt_div_config[2] = {SMPLRT_DIV, 0}; // 0 sets sample rate to 1 kHz
	if (write(fd, smplrt_div_config, 2) != 2) {
		perror("Failed to set sample rate divider");
		return -1;
	}

	mpu6050->read_hw = mpu6050_i2cdev_read_hw;

	mpu6050->acc.scale = (1.0f / ACC_FS_SENSITIVITY);
	mpu6050->gyro.scale = (1.0f / GYRO_FS_SENSITIVITY);

	mpu6050_init(mpu6050);

	return 0;
}
