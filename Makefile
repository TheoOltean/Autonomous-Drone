FLAGS = -O3 -Wall

build: objects/i2c.o main.c
	gcc $(FLAGS) -o build objects/i2c.o main.c

objects/i2c.o: i2c/i2c.c i2c/i2c.h
	gcc $(FLAGS) -o objects/i2c.o -O3 -c i2c/i2c.c

