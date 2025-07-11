#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "mpu6050.h"

uint8_t backend_is_libiio = 0;

static void parse_options(int argc, char *argv[])
{
	int c;

	while ((c = getopt(argc, argv, "b:")) != -1) {
		switch (c) {
		case 'b':
			if (!strcmp(optarg, "iio")) {
                backend_is_libiio = 1;
            } else if(!strcmp(optarg, "i2cdev")) {
                backend_is_libiio = 0;
            } else {
                exit(1);
            }
			break;
		default:
			exit(1);
		}
	}
}

int main(int argc, char *argv[])
{
    int ret;
    mpu6050_t mpu;

    parse_options(argc, argv);

    if (backend_is_libiio) {
        ret = mpu6050_iio_init(&mpu); 
    } else {
        ret = mpu6050_i2cdev_init(&mpu, 1); 
    }
    if (unlikely(ret)) {
        fprintf(stderr, "mpu6050_i2cdev_init err\n");
        return -1;
    }

    mpu.cf_ratio = 0.97f; 
    mpu.alpha = 0.8f;
    mpu.gyro.sampling_ms = 10;

    ret = mpu6050_calibrate(&mpu, 100);
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

        usleep(10000);

        printf("X:%f, Y:%f, Z:%f\n", mpu.angle[X], mpu.angle[Y], mpu.angle[Z]);
    }
}
