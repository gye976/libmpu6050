#ifndef MPU6050_I2CDEV_H
#define MPU6050_I2CDEV_H

#include <mpu6050_core.h>

#define INT_ENABLE	0x38
#define INT_STATUS	0x3A
#define FIFO_OVERFLOW_BIT	0x10

#define MPU6050_ADDR 0x68 // MPU6050 I2C address
#define PWR_MGMT_1 0x6B // Power Management 1 register
#define CONFIG 0x1A // DLPF Configuration register
#define SMPLRT_DIV 0x19

#define USER_CTRL      0x6A

#define GYRO_CONFIG 0x1B // Gyroscope Configuration register
#define ACCEL_CONFIG 0x1C // Accelerometer Configuration register
#define ACCEL_XOUT_H 0x3B // Accelerometer X-axis high byte
#define ACCEL_XOUT_L 0x3C // Accelerometer X-axis low byte
#define GYRO_XOUT_H 0x43 // Gyroscope X-axis high byte
#define GYRO_XOUT_L 0x44 // Gyroscope X-axis low byte

#define ACC_FS_SENSITIVITY		4096.0f 
#define GYRO_FS_SENSITIVITY		65.536f

typedef struct mpu6050_i2cdev_data {
	int fd;
	int dev; // /dev/i2c-%d
} mpu6050_i2cdev_data_t;

int mpu6050_i2cdev_init(mpu6050_t *mpu6050, int dev);

#endif 

