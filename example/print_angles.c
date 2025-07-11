#include <stdio.h>
#include <unistd.h>

#include "mpu6050.h"

#define NUM 1000
#define ALPHA 0.8

int main()
{
    int ret;
    mpu6050_t mpu;

    ret = mpu6050_iio_init(&mpu); 
    if (unlikely(ret)) {
        fprintf(stderr, "mpu6050_i2cdev_init err\n");
        return -1;
    }

    mpu.sampling_dt = 0.001;
    ret = mpu6050_calibrate(&mpu, NUM, ALPHA);
    if (unlikely(ret)) {
        fprintf(stderr, "mpu6050_calibrate err\n");
        return -1;
    }

    while (1) {
        ret = mpu6050_calc_angle(&mpu);
        if (unlikely(ret)) {
            fprintf(stderr, "mpu6050_calc_angle err\n");
            return -1;
        }

        usleep(1000);

        printf("X:%f, Y:%f, Z:%f\n", mpu.angle[X], mpu.angle[Y], mpu.angle[Z]);
    }
}
