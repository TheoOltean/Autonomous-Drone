FLAGS = -O3 -Wall

build: objects/i2c.o objects/IMU-ICM-20948.o main.c
	gcc $(FLAGS) -o $@ $^ -lfusion -lm

objects/i2c.o: i2c/i2c.c 
	gcc $(FLAGS) -o $@ -c $^ 

objects/IMU-ICM-20948.o: drivers/IMU-ICM-20948.c
	gcc $(FLAGS) -o $@ -c $^
