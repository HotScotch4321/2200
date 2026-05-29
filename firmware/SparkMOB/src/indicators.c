#include "indicators.h"
#include "pins.h"
#include <avr/io.h>

void indicators_init(void) {
    TCCR4A &= ~((1 << COM4A1) | (1 << COM4A0));  // release PC7 from Timer4
    TCCR3A &= ~((1 << COM3A1) | (1 << COM3A0));

    IND0_DDR  |= (1 << IND0_BIT);
    IND1_DDR |= (1 << IND1_BIT);
    IND2_DDR |= (1 << IND2_BIT);
    IND0_PORT  &= ~(1 << IND0_BIT);
    IND1_PORT &= ~(1 << IND1_BIT);
    IND2_PORT &= ~(1 << IND2_BIT);
}

void indicators_set(uint8_t idx, uint8_t on) {
    switch (idx) {
        case 0: // RED
            if (on) IND0_PORT  |=  (1 << IND0_BIT);
            else    IND0_PORT  &= ~(1 << IND0_BIT);
            break;
        case 1: // GREEN
            if (on) IND1_PORT |=  (1 << IND1_BIT);
            else    IND1_PORT &= ~(1 << IND1_BIT);
            break;
        case 2: // BLUE
            if (on) IND2_PORT |=  (1 << IND2_BIT);
            else    IND2_PORT &= ~(1 << IND2_BIT);
            break;
    }
}

void indicators_toggle(uint8_t idx) {
    switch (idx) {
        case 0: // RED
            IND0_PIN  = (1 << IND0_BIT);
            break;
        case 1: // GREEN
            IND1_PIN = (1 << IND1_BIT);
            break;
        case 2: // BLUE
            IND2_PIN = (1 << IND2_BIT);
            break;
    }
}

void indicators_all(uint8_t on) {
    indicators_set(0, on);
    indicators_set(1, on);
    indicators_set(2, on);
}
