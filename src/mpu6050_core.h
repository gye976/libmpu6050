#ifndef MPU6050_CORE_H
#define MPU6050_CORE_H

#include <endian.h>
#include <math.h>
#include <stdint.h>
#include <pthread.h>

#define SMA_N	5

#define mpu_dbg(f) \
       f

#define mpu_err(format, ...) \
       fprintf(stderr, "%s:%d: "format, __func__, __LINE__, ##__VA_ARGS__)

#define RADIANS_TO_DEGREES 		((float)(180/3.14159))

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

	double scale;
	/* Angle calculated acc */
	float angle[2];
} acc_t;

typedef struct gyro {
	int16_t buf[SMA_N][3];
	int16_t raw[3];

	float rad_s[3];
	float bias[3];

	double scale;
	/* Angle calculated gyro */
	float angle[3];

	uint8_t sampling_ms;
} gyro_t;

typedef struct mpu6050 mpu6050_t;

typedef struct mpu6050_iface {
	int (*init)(mpu6050_t *mpu6050);
	int (*read)(mpu6050_t *mpu6050, int16_t *dest);
} mpu6050_iface_t;

typedef struct mpu6050 {
	mpu6050_iface_t *iface;
	void *data;

	pthread_t raw_loop, angle_loop;

	/* Final output angle. */
	float angle[3];

	/* Complementary Filter Ratio between acc and gyro */
	float cf_ratio;

	acc_t acc;
	gyro_t gyro;
	int buf_i;
} mpu6050_t;

enum enum_axis {
	X, Y, Z
};

enum enum_angle {
	PITCH, ROLL, YAW
};

int mpu6050_core_alloc(mpu6050_t *mpu6050, unsigned int size);
int mpu6050_core_init(mpu6050_t *mpu6050, mpu6050_iface_t *iface);

int mpu6050_calibrate(mpu6050_t *mpu6050, unsigned int num);

void mpu6050_print_raw(mpu6050_t *mpu6050);
void mpu6050_print_val(mpu6050_t *mpu6050);

#endif 

