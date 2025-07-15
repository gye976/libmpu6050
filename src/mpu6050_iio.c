#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <endian.h>
#include <iio.h>

#include "mpu6050_iio.h"
#include <mpu6050.h>

static int mpu6050_iio_read_hw(mpu6050_t *mpu6050, int16_t *dest)
{
    struct iio_buffer *buf = mpu6050->iio.buf;
    struct iio_channel **chans = mpu6050->iio.chans;

    ssize_t nbytes;
    
    nbytes = iio_buffer_refill(buf);
    if (nbytes < 0) {
        fprintf(stderr, "Failed to refill buffer\n");
        return -1;
    }

    for (int i = 0; i < 6; i++) {
        iio_channel_convert(chans[i], &dest[i], iio_buffer_first(buf, chans[i]));
    }

    return 0;
}

void a()
{
    // iio_buffer_destroy(buf);
    // iio_context_destroy(ctx);
}

int mpu6050_iio_init(mpu6050_t *mpu6050)
{
	int ret;
	mpu6050_iio_t *mpu_iio = &mpu6050->iio;
	struct iio_context *ctx;
	struct iio_device *dev;
	struct iio_buffer *buf;
	struct iio_channel *chans[6];
	const char *chans_name[] = {
		"accel_x", "accel_y", "accel_z", 
		"anglvel_x", "anglvel_y", "anglvel_z" 
	};

	memset(mpu6050, 0, sizeof(mpu6050_t));

	ctx = iio_create_local_context();
	if (!ctx) {
		fprintf(stderr, "Failed to create context\n");
		return -1;
	}

	dev = iio_context_find_device(ctx, "mpu6050");
	if (!dev) {
		fprintf(stderr, "MPU6050 not found\n");
		return -1;
	}

	for (int i = 0; i < 6; i++) {
		chans[i] = iio_device_find_channel(dev, chans_name[i], false);
		if (!chans[i]) {
			    fprintf(stderr, "Channel %s not found\n", chans_name[i]);
			    return -1;
		}
		iio_channel_enable(chans[i]);
	}

	buf = iio_device_create_buffer(dev, SAMPLE_SIZE, false);
	if (!buf) {
		fprintf(stderr, "Failed to create buffer\n");
		return -1;
	}

	mpu_iio->buf = buf;
	for (int i = 0; i < 6; i++) {
		mpu_iio->chans[i] = chans[i];
	}

	mpu6050->read_hw = mpu6050_iio_read_hw;

	ret = iio_channel_attr_read_double(chans[0], "scale", &mpu6050->acc.scale);
	if (ret) {
		return -1;
	}
	ret = iio_channel_attr_read_double(chans[3], "scale", &mpu6050->gyro.scale);
	if (ret) {
		return -1;
	}

	ret = mpu6050_init(mpu6050);
	if (ret) {
		return ret;
	}

	return 0;
}
