#include "switch.h"

void bump_init(void) {
    DDRB  &= ~(1 << PB4);   // input
    PORTB &= ~(1 << PB4);   // no pull-up (active high)
    DDRD  &= ~(1 << PD7);   // input
    PORTD &= ~(1 << PD7);   // no pull-up (active high)
}


uint8_t bump_read(uint8_t idx) {
    // switch drives pin HIGH when pressed
    switch (idx) {
        case 0:
             return (PINB >> PB4) & 1;
        case 1:
             return (PIND >> PD7) & 1;
        default:
             return 0;
    }
}
