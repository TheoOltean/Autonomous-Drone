FLAGS = -O3 -Wall

build: i2c.o main.c
	gcc $(FLAGS) -o build i2c.o main.c

i2c.o: i2c/i2c.c i2c/i2c.h
	gcc $(FLAGS) -o i2c.o -O3 -c i2c/i2c.c

