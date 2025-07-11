#ifndef MPU6050_H
#define MPU6050_H

#include <endian.h>
#include <math.h>
#include <stdint.h>

#include "mpu6050_i2cdev.h"
#include "mpu6050_iio.h"

#define mpu_dbg(format, ...) \
       mpu_print(format, ##__VA_ARGS__)

#define mpu_print(format, ...) \
       fprintf(stderr, "%s:%d: "format, __func__, __LINE__, ##__VA_ARGS__)


#define RADIANS_TO_DEGREES 		((float)(180/3.14159))

#define CF_ALPHA                   0.96f
#define A_ACC	0.3f
#define A_GYRO	0.5f

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define open_fd(fd, path, args...) \
({ \
	fd = open(path, args); \
	unlikely(fd == -1) ? ({ perror("open"); fprintf(stderr, "path:%s\n", path); -1; }) : 0; \
})

typedef struct mpu6050 {
	/* current value */
	float angle[3];
	float gyro[3];

	float gyro_bias[3];
	float alpha;

	float sampling_dt;
	
	int (*read_raw)(struct mpu6050 *mpu6050, float acc[], float gyro[]);

	int fd;
	mpu6050_iio_t iio;
} mpu6050_t;

enum enum_axis {
	X, Y, Z
};

enum enum_angle {
	PITCH, ROLL, YAW
};

int mpu6050_calibrate(mpu6050_t *mpu6050, unsigned int num, float alpha);
int mpu6050_calc_angle(mpu6050_t *mpu6050);

#endif 

