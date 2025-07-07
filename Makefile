all:
	gcc *.c *.h -o mpu6050 -lm

clean:
	rm mpu6050