#ifndef MPU6050_IIO_H
#define MPU6050_IIO_H

#include <iio.h>

typedef struct mpu6050 mpu6050_t;

typedef struct mpu6050_iio {
//     struct iio_context *ctx;
//     struct iio_device *dev;
    struct iio_buffer *buf;
    struct iio_channel *chans[6];
} mpu6050_iio_t;

#define SAMPLE_SIZE 1

int mpu6050_iio_init(mpu6050_t *mpu6050);

#endif 

