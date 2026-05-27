#include "switch.h"

void bump_init(void) {
    DDRD  &= ~(1 << PD5);   // input
    PORTD |=  (1 << PD5);   // enable pull-up
}

uint8_t bump_read(void) {
    // switch pulls pin LOW when pressed
    return !((PIND >> PD5) & 1);
} 
