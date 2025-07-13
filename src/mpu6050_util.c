#include <mpu6050_util.h>

void fifo_init(fifo_t *fifo)
{

}

int fifo_produce(fifo_t *fifo, int size)
{
    int ret;
    int fifo_size = fifo->fifo_size;
    int consume_idx = fifo->consume_idx;
    int produce_idx = fifo->produce_idx;
    int packet_num = fifo->packet_num;
    int packet_size = fifo->packet_size;
    char *buf = fifo->buf;
    int next_produce_idx;
    int delta_idx;

    delta_idx = produce_idx - consume_idx;
	if (delta_idx < 0) {
		delta_idx += fifo_size;
	}
    if (delta_idx > packet_size * (packet_num - 1)) {
        mpu_err("consumer is slow, bug");
        return -1; 
    }

    if (produce_idx + size >= packet_size * packet_num) {
        next_produce_idx = 0;
        fifo->fifo_size = produce_idx + size + 1;
    } else { 
        next_produce_idx = produce_idx + size;
    }

    ret = fifo->produce(p, buf, size);
    if (ret != size) {
        mpu_err("produce size err");
        return -1;
    }

    fifo->produce_idx = next_produce_idx;

    return size;
}

int fifo_consume(fifo_t *fifo, int size)
{
    int ret;
    int fifo_size = fifo->fifo_size;
    int consume_idx = fifo->consume_idx;
    int produce_idx = fifo->produce_idx;
    int packet_num = fifo->packet_num;
    int packet_size = fifo->packet_size;
    char *buf = fifo->buf;
    int delta_idx;

    delta_idx = produce_idx - consume_idx;
	if (delta_idx < 0) {
		delta_idx += fifo_size;
	}

	if (delta_idx == 0) {
        // producer is slow, sleep
        return 0;
	} else if (delta_idx >= 16) {
		// todo
        //consume_idx = (consume_idx + (delta_idx / 2)) & (fifo_size - 1);
    }

    if (size > delta_idx)
        size = delta_idx;
    
    int nbytes = fifo_size - consume_idx;
    int num = size / packet_size;
    for (int i = 0; i < num; i++) {
        ret = fifo->consume(fifo, buf, packet_size);
        if (ret != size) {
            mpu_err("consumer size err");
            return -1;
        }

        buf += packet_size;
    }

    fifo->consume_idx = (consume_idx + size) & (fifo_size - 1);

    return size;
}