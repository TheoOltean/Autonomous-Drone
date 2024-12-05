#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "../drivers/PWM-GPIO.h"

// Motor Pins
#define MOTOR_FRONT_LEFT 26
#define MOTOR_FRONT_RIGHT 17
#define MOTOR_BACK_LEFT 23
#define MOTOR_BACK_RIGHT 16

#define HOVER_POWER 5

#define HOVER_SP_ROLL 0
#define HOVER_SP_PITCH 0
#define HOVER_SP_YAW 0

typedef struct {
    double kp;
    double ki;
    double kd;
    double setpoint;
    double integral;
    double previous_error;
} PIDController;

typedef struct {
    double roll;
    double pitch;
    double yaw;
} Orientation;

void init_pid(PIDController *pid, double kp, double ki, double kd, double setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
}

double calculate_pid(PIDController *pid, double measurement, double dt) {
    double error = pid->setpoint - measurement;
    pid->integral += error * dt;
    double derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

double get_time_sec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)(ts.tv_sec) + (double)(ts.tv_nsec) / 1e9;
}

void *controller() {
    PIDController pid_roll, pid_pitch, pid_yaw;
    pid_roll = (PIDController){.kp = 0.1, .ki = 0, .kd = 0, .setpoint = HOVER_SP_ROLL, .integral = 0, .previous_error = 0};
    pid_pitch = (PIDController){.kp = 0.1, .ki = 0, .kd = 0, .setpoint = HOVER_SP_PITCH, .integral = 0, .previous_error = 0};
    pid_yaw = (PIDController){.kp = 0.1, .ki = 0, .kd = 0, .setpoint = HOVER_SP_YAW, .integral = 0, .previous_error = 0};

    set_motor_power(MOTOR_FRONT_LEFT, HOVER_POWER);
    set_motor_power(MOTOR_FRONT_RIGHT, HOVER_POWER);
    set_motor_power(MOTOR_BACK_LEFT, HOVER_POWER);
    set_motor_power(MOTOR_BACK_RIGHT, HOVER_POWER);

    double previous_time = get_time_sec();

    while (1) {

        double current_time = get_time_sec();
        double dt = current_time - previous_time;
        if (dt <= 0.0) {
            dt = 0.01; 
        }
        previous_time = current_time;

        Orientation orientation = read_orientation();

        double roll_output = calculate_pid(&pid_roll, orientation.roll, dt);
        double pitch_output = calculate_pid(&pid_pitch, orientation.pitch, dt);
        double yaw_output = calculate_pid(&pid_yaw, orientation.yaw, dt);

        int motor_fl = HOVER_POWER - (int)(roll_output) + (int)(pitch_output) + (int)(yaw_output);
        int motor_fr = HOVER_POWER + (int)(roll_output) + (int)(pitch_output) - (int)(yaw_output);
        int motor_bl = HOVER_POWER - (int)(roll_output) - (int)(pitch_output) - (int)(yaw_output);
        int motor_br = HOVER_POWER + (int)(roll_output) - (int)(pitch_output) + (int)(yaw_output);

        if(motor_fl > PWM_MAX_POWER || motor_fr > PWM_MAX_POWER || motor_bl > PWM_MAX_POWER || motor_br > PWM_MAX_POWER){
            
            int max_value = motor_fl;

            if (motor_fr > max_value) {
                max_value = motor_fr;
            }
            if (motor_bl > max_value) {
                max_value = motor_bl;
            }
            if (motor_br > max_value) {
                max_value = motor_br;
            }

            motor_fl -= (maxPower - PWM_MAX_POWER);
            motor_fr -= (maxPower - PWM_MAX_POWER);
            motor_bl -= (maxPower - PWM_MAX_POWER);
            motor_br -= (maxPower - PWM_MAX_POWER);
        }

        PWM_Setpower(MOTOR_FRONT_LEFT, motor_fl);
        pwm_setPower(MOTOR_FRONT_RIGHT, motor_fr);
        pwm_setPower(MOTOR_BACK_LEFT, motor_bl);
        pwm_setPower(MOTOR_BACK_RIGHT, motor_br);

        usleep(10000);
    }
    return 0;
}
