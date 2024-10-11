#ifndef PWM_H
#define PWM_H

#include <stdint.h>

void pwm_init();
void pwm_setFrequency(float frequency);
int pwm_writeChannel(int channel, uint16_t phaseBegin, uint16_t phaseEnd);

#endif
