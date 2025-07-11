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

void mpu6050_print_raw(mpu6050_t *mpu6050)
{
	int16_t *acc_raw = mpu6050->acc.raw;
	int16_t *gyro_raw = mpu6050->gyro.raw;
	
        printf("acc raw: %d, %d, %d\n", acc_raw[X], acc_raw[Y], acc_raw[Z]);
        printf("gyro raw:%d, %d, %d\n\n", gyro_raw[X], gyro_raw[Y], gyro_raw[Z]);
}

void mpu6050_print_val(mpu6050_t *mpu6050)
{
	float *m_s2 = mpu6050->acc.m_s2;
	float *rad_s = mpu6050->gyro.rad_s;
	
        printf("acc m_s2: %f, %f, %f\n", m_s2[X], m_s2[Y], m_s2[Z]);
        printf("gyro rad_s:%f, %f, %f\n\n", rad_s[X], rad_s[Y], rad_s[Z]);
}

static int mpu6050_read_raw(mpu6050_t *mpu6050)
{
	int ret;

	if (unlikely(mpu6050->read_raw == NULL)) {
		fprintf(stderr, "read_raw is NULL");
		return -1;
	}

	ret = mpu6050->read_raw(mpu6050);

        mpu_dbg(mpu6050_print_raw(mpu6050));

	return ret;
}

static void mpu6050_apply_scale(mpu6050_t *mpu6050)
{
	int16_t *acc_raw = mpu6050->acc.raw;
	int16_t *gyro_raw = mpu6050->gyro.raw;
	
	float *m_s2 = mpu6050->acc.m_s2;
	float *rad_s = mpu6050->gyro.rad_s;
	
	float acc_scale = mpu6050->acc.scale;
	float gyro_scale =  mpu6050->gyro.scale;

	for (int i = 0; i < 3; i++) {
		m_s2[i] = acc_raw[i] * acc_scale;
		rad_s[i] = gyro_raw[i] * gyro_scale;
	}
}

static void mpu6050_gyro_apply_bias(gyro_t *gyro)
{
	float *rad_s = gyro->rad_s;
	float *bias = gyro->bias;

	for (int i = 0; i < 3; i++)
		rad_s[i] -= bias[i];
}		

static int mpu6050_gyro_get_angle(gyro_t *gyro, float cur_angle[])
{
	float *rad_s = gyro->rad_s;
	float *angle = gyro->angle;
	uint8_t sampling_ms = gyro->sampling_ms;

	if (unlikely(sampling_ms == 0)) {
		fprintf(stderr, "sampling_ms initial err\n");
		return -1;
	}

	for (int i = 0; i < 3; i++)
		angle[i] = cur_angle[i] + (rad_s[i] * gyro->sampling_ms / 1000.0f);

	return 0;
}

static int mpu6050_acc_get_angle(acc_t *acc)
{
	float *m_s2 = acc->m_s2;
	float *angle = acc->angle;
	
	if (unlikely(sqrt(pow(m_s2[X],2) + pow(m_s2[Z],2))) == 0) {
		fprintf(stderr, "nan\n");
		return -1;
	}
	if (unlikely(sqrt(pow(m_s2[Y],2) + pow(m_s2[Z],2))) == 0) {
		fprintf(stderr, "nan\n");
		return -1;
	}

	angle[PITCH] = atan(m_s2[Y] / sqrt(pow(m_s2[X],2) + pow(m_s2[Z],2))) \
		* RADIANS_TO_DEGREES;

	angle[ROLL] = atan(-1 * m_s2[X] / sqrt(pow(m_s2[Y], 2) + pow(m_s2[Z],2))) \
		* RADIANS_TO_DEGREES;
	
	return 0;
}

static int mpu6050_apply_angle(mpu6050_t *mpu6050)
{
	float *acc_angle = mpu6050->acc.angle;
	float *gyro_angle = mpu6050->gyro.angle;
	float *cur_angle = mpu6050->angle;
	float cf_ratio = mpu6050->cf_ratio;
	float alpha = mpu6050->alpha;
	
	float sampling_angle[3];

	if (unlikely(cf_ratio <= 0.0f || cf_ratio > 1.0f)) {
		return -1;
	}
	if (unlikely(alpha <= 0.0f || alpha > 1.0f)) {
		return -1;
	}

	for (int i = 0; i < 2; i++)
		sampling_angle[i] = (1.0 - cf_ratio) * acc_angle[i] + cf_ratio * gyro_angle[i];
	sampling_angle[YAW] =  gyro_angle[YAW];

	for (int i = 0; i < 3; i++) 
		cur_angle[i] = alpha * cur_angle[i] + (1.0f - alpha) * sampling_angle[i];

	return 0;
}

/* 
 * 1: To measure gyro error 
 * (Always gyroscope value is defined as zero  when stationary, 
 * but the acc varies depending on its orientation and position, 
 * so it cannot be precisely defined. Therefore, only measured the gyroscope’s bias.)
 *
 * 2: To set angle by acc
 * The gyro only provides the incremental changes of the Euler angles, 
 * but it cannot measure the absolute Euler angles relative to the real world.
 */
int mpu6050_calibrate(mpu6050_t *mpu6050, uint32_t num)
{
	int ret;
	acc_t *acc = &mpu6050->acc;
	float *acc_angle = mpu6050->acc.angle;
	float *rad_s = mpu6050->gyro.rad_s;

	float *cur_angle = mpu6050->angle;
	float *gyro_bias = mpu6050->gyro.bias;

	float acc_angle_total[3] = { 0, };  
	float rad_s_total[3] = { 0, };  

	for (uint32_t i = 0; i < num; i++) {
		ret = mpu6050_read_raw(mpu6050);
		if (unlikely(ret)) {
			fprintf(stderr, "mpu6050_read_raw err\n");
			return ret;
		}

		mpu6050_apply_scale(mpu6050);
		
		ret = mpu6050_acc_get_angle(acc);
		if (ret) {
			fprintf(stderr, "mpu6050_acc_get_angle err\n");
			return ret;
		}

		for (int j = 0; j < 2; j++) {
			acc_angle_total[j] += acc_angle[j];
			rad_s_total[j] += rad_s[j];
		}
		rad_s_total[2] += rad_s[2];
	}

	for (int i = 0; i < 2; i++) {
		gyro_bias[i] = rad_s_total[i] / num;
		cur_angle[i] = acc_angle_total[i] / num;
	}
	gyro_bias[2] = rad_s_total[2] / num;

        // printf("gyro bias: %f, %f, %f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
        // printf("initial angle by acc: %f, %f\n", cur_angle[0], cur_angle[1]);
        // printf("\n");

	return 0;
}

int mpu6050_calc_angle(mpu6050_t *mpu6050)
{
	int ret;
	acc_t *acc = &mpu6050->acc;
	gyro_t *gyro = &mpu6050->gyro;
	float *cur_angle = mpu6050->angle;

	ret = mpu6050_read_raw(mpu6050);
	if (unlikely(ret)) {
		fprintf(stderr, "mpu6050_read_raw err\n");
		return ret;
	}

	mpu6050_apply_scale(mpu6050);

	mpu6050_gyro_apply_bias(gyro);
	
	ret = mpu6050_acc_get_angle(acc);
	if (ret) {
		fprintf(stderr, "mpu6050_acc_get_angle err\n");
		return ret;
	}
	ret = mpu6050_gyro_get_angle(gyro, cur_angle);
	if (ret) {
		fprintf(stderr, "mpu6050_gyro_get_angle err\n");
		return ret;
	}

	ret = mpu6050_apply_angle(mpu6050);
	if (ret) {
		fprintf(stderr, "mpu6050_apply_angle err\n");
		return ret;
	}

	return 0;
}
