#ifndef pwm16_h
#define pwm16_h

#include <Arduino.h>

void setupPWM16(int resolution);
void analogWrite16(uint8_t pin, uint16_t val);

#endif
