all: sample-i2c

sample-i2c: sample_arduinoI2C.c
	gcc sample_arduinoI2C.c -o sample-i2c