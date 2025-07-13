#ifndef MPU6050_UTIL_H
#define MPU6050_UTIL_H

#ifdef PRINT_DEBUG
#define mpu_dbg(f) \
       f
#else 
#define mpu_dbg(f) \
	(void)f
#endif

#define mpu_err(format, ...) \
       fprintf(stderr, "%s:%d: "format"\n", __func__, __LINE__, ##__VA_ARGS__)

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#define open_fd(fd, path, args...) \
({ \
	fd = open(path, args); \
	unlikely(fd == -1) ? ({ perror("open"); fprintf(stderr, "path:%s\n", path); -1; }) : 0; \
})

#define FIFO_SIZE 1024

typedef struct fifo fifo_t;

typedef struct fifo {
    int packet_num;
    int packet_size; // unit
    int fifo_size;

	int consume_idx; /* 0 ~ (fifo_size - 1)*/
	int produce_idx;

    int (*produce)(fifo_t *fifo, char *buf, int size);
    int (*consume)(fifo_t *fifo, char *buf, int size);

	char buf[]; // packet size * (packet num + 1) 
} fifo_t;

void fifo_init(fifo_t *fifo);
int fifo_produce(fifo_t *fifo, int size);
int fifo_consume(fifo_t *fifo, int size);

#endif 

