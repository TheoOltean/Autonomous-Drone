build: i2c.o main.c
	gcc -o build i2c.o -O3 main.c

i2c.o:
	gcc -o i2c.o -O3 -c i2c/i2c.c

