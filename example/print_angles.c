#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "mpu6050.h"

int backend = 0;
int print_ms = 1;
int sampling_ms = 1;

static void parse_options(int argc, char *argv[])
{
	int c;

	while ((c = getopt(argc, argv, "b:c:p:s:")) != -1) {
		switch (c) {
		case 'b':
			if (!strcmp(optarg, "iio")) {
				backend = 1;
			} else if(!strcmp(optarg, "i2cdev")) {
				backend = 2;
			} else {
				fprintf(stderr, "-b err\n");
				exit(1);
			}
			break;
		case 'p':
			print_ms = atoi(optarg);
			if (print_ms < 0) {
				fprintf(stderr, "invalide ms\n");
				exit(1);
			}
			break;
		case 's':
			sampling_ms = atoi(optarg);
			if (sampling_ms < 0) {
				fprintf(stderr, "invalide ms\n");
				exit(1);
			}
			break;
		default:
			fprintf(stderr, "option err\n");
			exit(1);
		}
	}
}

int main(int argc, char *argv[])
{
	int ret;
	mpu6050_t mpu;

	parse_options(argc, argv);

	switch (backend) {
	case 1:
		ret = mpu6050_iio_init(&mpu); 
		if (unlikely(ret)) {
			fprintf(stderr, "mpu6050_iio_init err\n");
			return -1;
		}
		break;
	case 2:
		ret = mpu6050_i2cdev_init(&mpu, 1); 
		if (unlikely(ret)) {
			fprintf(stderr, "mpu6050_i2cdev_init err\n");
			return -1;
		}
		break;
	default:
		fprintf(stderr, "No backend\n");
		exit(1);
		break;
	}

	mpu.cf_ratio = 0.97f; 
	mpu.gyro.sampling_ms = sampling_ms;

	// ret = mpu6050_calibrate(&mpu, 100);
	// if (unlikely(ret)) {
	//     fprintf(stderr, "mpu6050_calibrate err\n");
	//     return -1;
	// }

	while (1) {
		printf("X:%f, Y:%f, Z:%f\n", mpu.angle[X], mpu.angle[Y], mpu.angle[Z]);
		usleep(1000 * print_ms);
	}
}
