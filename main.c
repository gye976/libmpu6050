#include <stdio.h>

#include "mpu6050.h"

#define NUM 1000
#define ALPHA 0.8

int main(int argc, char *argv[])
{
    int ret;
    mpu6050_t mpu;

    ret = mpu6050_i2cdev_init(&mpu, 1); //i2c-1
    if (unlikely(ret != 0)) {
        fprintf(stderr, "mpu6050_i2cdev_init err");
        return -1;
    }

    ret = mpu6050_calibrate(&mpu, NUM, ALPHA);
    if (unlikely(ret != 0)) {
        fprintf(stderr, "mpu6050_calibrate err");
        return -1;
    }

    while (1) {
        ret = mpu6050_calc_angle(&mpu);
        if (unlikely(ret != 0)) {
            fprintf(stderr, "mpu6050_calc_angle err");
            return -1;
        }

        printf("X:%f, Y:%f, Z:%f\n", mpu.angle[X], mpu.angle[Y], mpu.angle[Z]);
    }
}
