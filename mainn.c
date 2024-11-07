#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Motor Pins
#define MOTOR_FRONT_LEFT 26
#define MOTOR_FRONT_RIGHT 17
#define MOTOR_BACK_LEFT 23
#define MOTOR_BACK_RIGHT 16

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

int main() {
    PIDController pid_roll, pid_pitch, pid_yaw;
    init_pid(&pid_roll, 0, 0.0, 0.1, 0.0);
    init_pid(&pid_pitch, 0, 0.0, 0.1, 0.0);
    init_pid(&pid_yaw, 0, 0.0, 0.1, 0.0);

    int base_power = 50;

    set_motor_power(MOTOR_FRONT_LEFT, base_power);
    set_motor_power(MOTOR_FRONT_RIGHT, base_power);
    set_motor_power(MOTOR_BACK_LEFT, base_power);
    set_motor_power(MOTOR_BACK_RIGHT, base_power);

    double previous_time = get_time_sec();

    while (1) {

        double current_time = get_time_sec();
        double dt = current_time - previous_time;
        if (dt <= 0.0) {
            dt = 0.01; 
        }
        previous_time = current_time;


        Orientation ori = read_orientation();


        double roll_output = calculate_pid(&pid_roll, ori.roll, dt);
        double pitch_output = calculate_pid(&pid_pitch, ori.pitch, dt);
        double yaw_output = calculate_pid(&pid_yaw, ori.yaw, dt);


        int motor_fl = base_power - (int)(roll_output) + (int)(pitch_output);
        int motor_fr = base_power + (int)(roll_output) + (int)(pitch_output);
        int motor_bl = base_power - (int)(roll_output) - (int)(pitch_output);
        int motor_br = base_power + (int)(roll_output) - (int)(pitch_output);


        motor_fl += (int)(yaw_output);
        motor_fr -= (int)(yaw_output);
        motor_bl -= (int)(yaw_output);
        motor_br += (int)(yaw_output);


        PWM_Setpower(MOTOR_FRONT_LEFT, motor_fl);
        set_motor_power(MOTOR_FRONT_RIGHT, motor_fr);
        set_motor_power(MOTOR_BACK_LEFT, motor_bl);
        set_motor_power(MOTOR_BACK_RIGHT, motor_br);

        usleep(10000);
    }
    return 0;
}
