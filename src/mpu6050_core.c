#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <endian.h>

#include "mpu6050.h"

//'be16toh' only considers the bit, does not guarantee the sign.
static inline int16_t be16toh_s(int16_t be_val)
{
    return (int16_t)be16toh(be_val);
}

static inline int mpu6050_read_raw(mpu6050_t *mpu6050, float acc[], float gyro[])
{
	if (unlikely(mpu6050->read_raw == NULL)) {
		fprintf(stderr, "read_raw is NULL");
		return -1;
	}

	return mpu6050->read_raw(mpu6050, acc, gyro);
}

static void mpu6050_gyro_bias(mpu6050_t *mpu6050, float *gyro)
{
	float *bias = mpu6050->gyro_bias;

	for (int i = 0; i < 3; i++)
		gyro[i] -= bias[i];
}		

static void mpu6050_gyro_to_angle(mpu6050_t *mpu6050, float gyro[], float gyro_angle[])
{
	for (int i = 0; i < 3; i++)
		gyro_angle[i] += gyro[i] * mpu6050->dt;
}

static int mpu6050_acc_to_angle(float acc[], float new_acc_angle[])
{
	if (unlikely(sqrt(pow(acc[X],2) + pow(acc[Z],2))) == 0) {
		fprintf(stderr, "nan\n");
		return -1;
	}
	if (unlikely(sqrt(pow(acc[Y],2) + pow(acc[Z],2))) == 0) {
		fprintf(stderr, "nan\n");
		return -1;
	}

	new_acc_angle[PITCH] = atan(acc[Y] / sqrt(pow(acc[X],2) + pow(acc[Z],2))) \
		* RADIANS_TO_DEGREES;

	new_acc_angle[ROLL] = atan(-1 * acc[X] / sqrt(pow(acc[Y], 2) + pow(acc[Z],2))) \
		* RADIANS_TO_DEGREES;
	
	return 0;
}

static void do_EMA(float alpha, float *data, float new_data)
{
	*data = alpha * new_data + (1.0f - alpha) * (*data);
}

static void do_complementary_filter(float acc_angle[], float gyro_angle[], float angle[])
{
	for (int i = 0; i < 2; i++)
		angle[i] = (1.0 - CF_ALPHA) * acc_angle[i] + CF_ALPHA * gyro_angle[i];
}

int mpu6050_calibrate(mpu6050_t *mpu6050, uint32_t num, float alpha)
{
	int ret;
	float new_gyro[3] = { 0, };
	float gyro[3] = { 0, };

	float new_acc[3] = { 0, };
	float new_acc_angle[3] = { 0, };
	float acc_angle[3] = { 0, };

	for (uint32_t i = 0; i < num; i++) {
		ret = mpu6050_read_raw(mpu6050, new_acc, new_gyro);
		if (unlikely(ret != 0)) {
			return ret;
		}
		ret = mpu6050_acc_to_angle(new_acc, new_acc_angle);
		if (unlikely(ret != 0)) {
			return ret;
		}

		for (int j = 0; j < 3; j++)
			gyro[j] = (1.0f - alpha) * gyro[j] + alpha * new_gyro[j];

		acc_angle[0] = (1.0f - alpha) * acc_angle[0] + alpha * new_acc_angle[0];
		acc_angle[1] = (1.0f - alpha) * acc_angle[1] + alpha * new_acc_angle[1];
	}

	for (int i = 0; i < 3; i++)
		mpu6050->gyro_bias[i] = gyro[i];

	mpu6050->angle[0] = acc_angle[0];
	mpu6050->angle[1] = acc_angle[1];

	return 0;
}

int mpu6050_calc_angle(mpu6050_t *mpu6050)
{
	int ret;
	float *angle = mpu6050->angle;
	float *gyro = mpu6050->gyro;

	float acc_angle[3] = {
		angle[0], angle[1], angle[2],
	};
	float gyro_angle[3] = {
		angle[0], angle[1], angle[2],
	};

	float new_acc[3] = { 0, };
	float new_acc_angle[3] = { 0, };

	float new_gyro[3] = { 0, };

	ret = mpu6050_read_raw(mpu6050, new_acc, new_gyro);
	if (unlikely(ret != 0))
		return ret;

	mpu6050_gyro_bias(mpu6050, new_gyro);

	for (int i = 0; i < 3; i++) {
		do_EMA(A_GYRO, &gyro[i], new_gyro[i]);
	}
	
	mpu6050_gyro_to_angle(mpu6050, gyro, gyro_angle);

	/* acc */
	ret = mpu6050_acc_to_angle(new_acc, new_acc_angle);
	if (unlikely(ret != 0))
		return ret;

	for (int i = 0; i < 2; i++) {
		do_EMA(A_ACC, &acc_angle[i], new_acc_angle[i]);
	}

	do_complementary_filter(acc_angle, gyro_angle, angle);

	mpu6050->angle[YAW] = gyro_angle[YAW];

	return 0;
}
