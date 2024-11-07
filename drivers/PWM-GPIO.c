#include <unistd.h>
#include <lgpio.h>
#include <stdatomic.h>
#include "PWM-GPIO.h"

atomic_int power;

atomic_int power0;
atomic_int power1;
atomic_int power2;
atomic_int power3;

void pwm_setPower(int pin, int power) {
    if (power > 100 || power < 0) return;
    switch (pin) {
        case (PWM_PIN_0):
            atomic_store(&power0,power);
            break;
        case (PWM_PIN_1):
            atomic_store(&power1,power);
            break;
        case (PWM_PIN_2):
            atomic_store(&power2,power);
            break;
        case (PWM_PIN_3):
            atomic_store(&power3,power);
            break;
    }
}

atomic_int handle;

void *pwm_run(int pin) {
    lgGpioClaimOutput(atomic_load(&handle),0,pin,0);
    
    lgTxPwm(handle,pin,50.0,5.0,0,0);
    usleep(100000);
    lgTxPwm(handle,pin,50.0,10.0,0,0);
    usleep(100000);

    for (;;) {
        lgTxPwm(handle,pin,50.0,5.0+((float)atomic_load(&power)/(float)100)*5.0,PWM_FLAGS,0);
        usleep(20000);
    }

    return NULL;
}

void pwm_init() {
    pwm_setPower(PWM_PIN_0,0);
    pwm_setPower(PWM_PIN_1,0);
    pwm_setPower(PWM_PIN_2,0);
    pwm_setPower(PWM_PIN_3,0);

    atomic_store(&handle,lgGpiochipOpen(0));
    pthread_t pwmthread0, pwmthread1, pwmthread2, pwmthread4;
    struct sched_param param;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    param.sched_priority = 99;

    pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
    pthread_attr_setschedparam(&attr,&param);

    int data0 = PWM_PIN_0;
    pthread_create(&pwmthread0,&attr,pwm_run,(void*)&data0);
    int data1 = PWM_PIN_1;
    pthread_create(&pwmthread1,&attr,pwm_run,(void*)&data1);
    int data2 = PWM_PIN_2;
    pthread_create(&pwmthread2,&attr,pwm_run,(void*)&data2);
    int data3 = PWM_PIN_3;
    pthread_create(&pwmthread3,&attr,pwm_run,(void*)&data3);
}
