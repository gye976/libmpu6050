#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <endian.h>
#include <pthread.h>
#include <time.h>

#include "mpu6050.h"

//'be16toh' only considers the bit, does not guarantee the sign.
static inline int16_t be16toh_s(int16_t be_val)
{
    return (int16_t)be16toh(be_val);
}

static void mpu6050_convert(mpu6050_t *mpu6050, const int16_t *src)
{
	acc_t *acc = &mpu6050->acc;
	gyro_t *gyro = &mpu6050->gyro;
	int buf_i = mpu6050->buf_i;

	for (int i = 0; i < 3; i++) {
		acc->raw[i] -= acc->buf[buf_i][i] / SMA_N;
		gyro->raw[i] -= gyro->buf[buf_i][i] / SMA_N;
        
		acc->buf[buf_i][i] = src[i] / SMA_N;
		gyro->buf[buf_i][i] = src[i + 3] / SMA_N;

		acc->raw[i] += acc->buf[buf_i][i] / SMA_N;
		gyro->raw[i] += gyro->buf[buf_i][i] / SMA_N;
	}
    
	mpu6050->buf_i = (buf_i + 1) % SMA_N;
}

static void timespec_get_diff(struct timespec *start, struct timespec *end, struct timespec *diff) 
{
    if ((end->tv_nsec - start->tv_nsec) < 0) {
        diff->tv_sec  = end->tv_sec - start->tv_sec - 1;
        diff->tv_nsec = end->tv_nsec - start->tv_nsec + 1000000000ULL;
    } else {
        diff->tv_sec  = end->tv_sec - start->tv_sec;
        diff->tv_nsec = end->tv_nsec - start->tv_nsec;
    }
}

static int mpu6050_read_raw(mpu6050_t *mpu6050)
{
	(void)mpu6050;

	return 0;
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
	
	for (int i = 0; i < 2; i++)
		cur_angle[i] = (1.0 - cf_ratio) * acc_angle[i] + cf_ratio * gyro_angle[i];
	cur_angle[YAW] =  gyro_angle[YAW];

	return 0;
}

static void mpu6050_apply_scale(mpu6050_t *mpu6050)
{
	acc_t *acc = &mpu6050->acc;
	gyro_t *gyro = &mpu6050->gyro;
	
	float acc_scale = mpu6050->acc.scale;
	float gyro_scale =  mpu6050->gyro.scale;

	for (int i = 0; i < 3; i++) {
		acc->m_s2[i] = acc->raw[i] * acc_scale;
		gyro->rad_s[i] = gyro->raw[i] * gyro_scale;
	}
}

static int mpu6050_calc_angle(mpu6050_t *mpu6050)
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

	//mpu6050_gyro_apply_bias(gyro);
	
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

// static void mpu6050_gyro_apply_bias(gyro_t *gyro)
// {
// 	float *rad_s = gyro->rad_s;
// 	float *bias = gyro->bias;
//
// 	for (int i = 0; i < 3; i++)
// 		rad_s[i] -= bias[i];
// }		

static void* mpu6050_raw_loop(void* arg) 
{
	mpu6050_t *mpu6050 = arg;
	int16_t buf[6];
	int ret;

	if (unlikely(mpu6050->read_hw == NULL)) {
		mpu_err("read_hw is NULL");
		return (void*)(-1);
	}

	while (1) {
	 	ret = mpu6050->read_hw(mpu6050, buf);
		if (unlikely(ret)) {
			return (void*)(-1);
		}

		mpu6050_convert(mpu6050, buf);
	}
	
	return (void*)(-1);
}

static void* mpu6050_angle_loop(void* arg) 
{
	mpu6050_t *mpu6050 = arg;
	int ret;
	struct timespec ts[2];
	struct timespec ts_sleep, ts_diff;
	struct timespec ts_sampling = {
		.tv_sec = 0,
		.tv_nsec = mpu6050->gyro.sampling_ms * (1000) * (1000),
	};

	while (1) {
		ret = clock_gettime(CLOCK_MONOTONIC, &ts[0]);
		if (unlikely(ret)) {
			return (void*)(-1);
		}

	 	ret = mpu6050_calc_angle(mpu6050);
		if (unlikely(ret)) {
			return (void*)(-1);
		}

		ret = clock_gettime(CLOCK_MONOTONIC, &ts[1]);
		if (unlikely(ret)) {
			return (void*)(-1);
		}

		timespec_get_diff(&ts[0], &ts[1], &ts_diff);	
		timespec_get_diff(&ts_diff, &ts_sampling, &ts_sleep);	
		ret = nanosleep(&ts_sleep, NULL);
		if (unlikely(ret)) {
			return (void*)(-1);
		}
	}
	
	return (void*)(-1);
}

int mpu6050_init(mpu6050_t *mpu6050)
{
	int ret;
	int16_t buf[6];
	acc_t *acc = &mpu6050->acc;
	gyro_t *gyro = &mpu6050->gyro;

	ret = mpu6050->read_hw(mpu6050, buf);
	if (unlikely(ret)) {
		return -1;
	}
	
	for (int i = 0; i < SMA_N; i++) {
		for (int j = 0; j < 3; j++) {
			acc->buf[i][j] = buf[j];
			gyro->buf[i][j] = buf[j + 3];
		}
	}
	for (int i = 0; i < 3; i++) {
               acc->raw[i] = buf[i];
               gyro->raw[i] = buf[i + 3];
	}

	ret = pthread_create(&mpu6050->raw_loop, NULL, mpu6050_raw_loop, mpu6050);
        if (unlikely(ret)) {
		perror("pthread_create");
		return -1;	
        }

	ret = pthread_create(&mpu6050->angle_loop, NULL, mpu6050_angle_loop, mpu6050);
        if (unlikely(ret)) {
		perror("pthread_create");
		return -1;	
        }

	return 0;	
}

// TODO: calib
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

// void mpu6050_print_raw(mpu6050_t *mpu6050)
// {
// 	int16_t *acc_raw = mpu6050->acc.raw;
// 	int16_t *gyro_raw = mpu6050->gyro.raw;
// 	
//         printf("acc raw: %d, %d, %d\n", acc_raw[X], acc_raw[Y], acc_raw[Z]);
//         printf("gyro raw:%d, %d, %d\n\n", gyro_raw[X], gyro_raw[Y], gyro_raw[Z]);
// }

void mpu6050_print_val(mpu6050_t *mpu6050)
{
	float *m_s2 = mpu6050->acc.m_s2;
	float *rad_s = mpu6050->gyro.rad_s;
	
        printf("acc m_s2: %f, %f, %f\n", m_s2[X], m_s2[Y], m_s2[Z]);
        printf("gyro rad_s:%f, %f, %f\n\n", rad_s[X], rad_s[Y], rad_s[Z]);
}

