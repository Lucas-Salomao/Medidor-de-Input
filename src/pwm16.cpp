#include "pwm16.h"

void setupPWM16(int resolution)
{
    uint16_t icr = 0xffff;
    OCR1A = 0;
    OCR1B = 0;
    switch (resolution)
    {
    case 16:
        icr = 0xffff;
        break;
    case 15:
        icr = 0x7fff;
        break;
    case 14:
        icr = 0x3fff;
        break;
    case 13:
        icr = 0x1fff;
        break;
    case 12:
        icr = 0x0fff;
        break;
    case 11:
        icr = 0x07ff;
        break;
    case 10:
        icr = 0x03ff;
        break;
    case 9:
        icr = 0x01ff;
        break;
    case 8:
        icr = 0x00ff;
        break;
    default:
        icr = 0x00ff;
        break;
    }
    DDRB |= _BV(PB1) | _BV(PB2);                  /* set pins as outputs */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)            /* non-inverting PWM */
             | _BV(WGM11);                        /* mode 14: fast PWM, TOP=ICR1 */
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); /* prescaler 1 */
    ICR1 = icr;                                   /* TOP counter value (freeing OCR1A*/
    //Serial.print("ICR1:");
    //Serial.println(icr);
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(uint8_t pin, uint16_t val)
{
    switch (pin)
    {
    case 9:
        OCR1A = val;
        break;
    case 10:
        OCR1B = val;
        break;
    }
}