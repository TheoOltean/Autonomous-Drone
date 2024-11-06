FLAGS = -O3 -Wall

build: objects/i2c.o objects/IMU-ICM-20948.o objects/PWM-GPIO.o main.c
	gcc $(FLAGS) -pthread -o $@ $^ -lfusion -lm -llgpio -lrt

objects/i2c.o: i2c/i2c.c 
	gcc $(FLAGS) -o $@ -c $^ 

objects/IMU-ICM-20948.o: drivers/IMU-ICM-20948.c
	gcc $(FLAGS) -o $@ -c $^

objects/PWM-GPIO.o: drivers/PWM-GPIO.c
	gcc $(FLAGS) -o $@ -c $^
