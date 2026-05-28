#include "switch.h"

void bump_init(void) {
    DDRD  &= ~(1 << PD5);   // input
    PORTD |=  (1 << PD5);   // enable pull-up
    DDRD  &= ~(1 << PD7);   // input
    PORTD |=  (1 << PD7);   // enable pull-up
}


uint8_t bump_read(uint8_t idx) { 
    // switch pulls pin LOW when pressed
    switch (idx) {
        case 0: // bump switch 1
             return !((PIND >> PD5) & 1);
        case 1: // bump switch 2 (if we had one)
             return !((PIND >> PD7) & 1);
        default:
             return 0; // invalid index
    }
} 
