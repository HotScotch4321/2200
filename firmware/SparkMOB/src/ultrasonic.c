#include "ultrasonic.h"
#include "pins.h"
#include <avr/io.h>
#include <util/delay.h>

// Timer1 prescaler /8  2 MHz tick, 0.5 µs per tic
// distance_mm = ticks * 343 / 4000
//
// Rising-edge wait: TCNT1 reset to 0, 60 000 tick timeout (30 ms).
// Falling-edge wait: unsigned 16-bit subtraction handles one overflow; 50 000 tick ceiling.

#define T1_RISING  ((1 << ICNC1) | (1 << ICES1) | (1 << CS11))
#define T1_FALLING ((1 << ICNC1) |                (1 << CS11))

void ultrasonic_init(void) {
    ULTRASONIC_TRIG_DDR  |=  (1 << ULTRASONIC_TRIG_BIT);
    ULTRASONIC_TRIG_PORT &= ~(1 << ULTRASONIC_TRIG_BIT);

    ULTRASONIC_ECHO_DDR  &= ~(1 << ULTRASONIC_ECHO_BIT);
    ULTRASONIC_ECHO_PORT &= ~(1 << ULTRASONIC_ECHO_BIT);
    TCCR1A = 0;
    TCCR1B = T1_RISING;
    TCCR1C = 0;
    TIMSK1 = 0;
}

uint32_t ultrasonic_measure_mm(void) {
    ULTRASONIC_TRIG_PORT |=  (1 << ULTRASONIC_TRIG_BIT);     // 10 µs trigger pulse
    _delay_us(10);
    ULTRASONIC_TRIG_PORT &= ~(1 << ULTRASONIC_TRIG_BIT);

    TCCR1B = T1_RISING;
    TIFR1  = (1 << ICF1);
    TCNT1  = 0;

    while (!(TIFR1 & (1 << ICF1))) {
        if (TCNT1 >= 60000) return 0;   // 30 ms – no echo
    }
    uint16_t t_rise = ICR1;

    TCCR1B = T1_FALLING;
    TIFR1  = (1 << ICF1);

    while (!(TIFR1 & (1 << ICF1))) {
        if ((uint16_t)(TCNT1 - t_rise) >= 50000) return 0;  // 25 ms ceiling
    }
    uint16_t t_fall = ICR1;

    uint16_t ticks = t_fall - t_rise;
    return ((uint32_t)ticks * 343UL) / 4000UL;
}
