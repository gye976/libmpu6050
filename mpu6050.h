#ifndef MPU6050_H
#define MPU6050_H

#include <endian.h>
#include <math.h>
#include <stdint.h>

#define RADIANS_TO_DEGREES 		((float)(180/3.14159))

#define CF_ALPHA                   0.96f
#define A_ACC	0.3f
#define A_GYRO	0.5f

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define OPEN_FD(fd, path, args...) \
do \
{ \
    fd = open(path, args); \
\
    if (unlikely(fd == -1)) \
    { \
        perror("open"); \
        fprintf(stderr, "path:%s\n", path); \
    } \
	return -1; \
} while (0)

typedef struct mpu6050 {
	/* current value */
	float angle[3];
	float gyro[3];

	float gyro_bias[3];
	float alpha;

	int fd;
	float dt;
	
	int (*read_raw)(struct mpu6050 *mpu6050, float acc[], float gyro[]);
} mpu6050_t;

enum enum_axis {
	X, Y, Z
};

enum enum_angle {
	PITCH, ROLL, YAW
};

int mpu6050_calibrate(mpu6050_t *mpu6050, unsigned int num, float alpha);
int mpu6050_calc_angle(mpu6050_t *mpu6050);

#include "mpu6050_i2cdev.h"

#endif 

