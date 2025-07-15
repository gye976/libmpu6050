#ifndef MPU6050_H
#define MPU6050_H

#include <endian.h>
#include <math.h>
#include <stdint.h>

#include "mpu6050_i2cdev.h"
#include "mpu6050_iio.h"

#define SMA_N	5

#define mpu_dbg(f) \
       f

#define mpu_err(format, ...) \
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

typedef struct acc {
	int16_t buf[SMA_N][3];
	int16_t raw[3];

	float m_s2[3];

	float scale;
	/* Angle calculated acc */
	float angle[2];
} acc_t;

typedef struct gyro {
	int16_t buf[SMA_N][3];
	int16_t raw[3];

	float rad_s[3];
	float bias[3];

	float scale;
	/* Angle calculated gyro */
	float angle[3];

	uint8_t sampling_ms;
} gyro_t;

typedef struct mpu6050 mpu6050_t;
typedef struct mpu6050 {
	acc_t acc;
	gyro_t gyro;
	int buf_i;

	/* Final output angle. */
	float angle[3];

	/* Complementary Filter Ratio between acc and gyro */
	float cf_ratio;
	
	int (*read_hw)(mpu6050_t *mpu6050, int16_t *dest);
	pthread_t raw_loop, angle_loop;

	int fd;
	mpu6050_iio_t iio;
} mpu6050_t;

enum enum_axis {
	X, Y, Z
};

enum enum_angle {
	PITCH, ROLL, YAW
};

int mpu6050_init(mpu6050_t *mpu6050);

int mpu6050_calibrate(mpu6050_t *mpu6050, unsigned int num);
int mpu6050_calc_angle(mpu6050_t *mpu6050);

void mpu6050_print_raw(mpu6050_t *mpu6050);
void mpu6050_print_val(mpu6050_t *mpu6050);

#endif 

