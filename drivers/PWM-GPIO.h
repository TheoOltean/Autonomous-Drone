#ifndef PWM_GPIO_H
#define PWM_GPIO_H

#define PWM_PIN_0 26
#define PWM_PIN_1 17
#define PWM_PIN_2 23
#define PWM_PIN_3 16

void pwm_init();

void pwm_setPower(int pin, int power);

#endif
