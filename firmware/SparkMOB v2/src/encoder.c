#include "encoder.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

static volatile int32_t enc1_count = 0;
static volatile int32_t enc2_count = 0;
static volatile uint8_t enc2_last  = 0;

// Speed: ticks since last encoder_speed_update() call (call at fixed rate e.g. 100 Hz)
static volatile int32_t enc1_speed = 0;
static volatile int32_t enc2_speed = 0;
static volatile int32_t enc1_prev  = 0;
static volatile int32_t enc2_prev  = 0;

void encoder_init(void) {
    DDRD  &= ~((1 << ENC1_A_BIT) | (1 << ENC1_B_BIT));
    PORTD |=  ((1 << ENC1_A_BIT) | (1 << ENC1_B_BIT));

    DDRB  &= ~((1 << ENC2_A_BIT) | (1 << ENC2_B_BIT));
    PORTB |=  ((1 << ENC2_A_BIT) | (1 << ENC2_B_BIT));

    EICRA |=  (1 << ISC20) | (1 << ISC30);
    EICRA &= ~((1 << ISC21) | (1 << ISC31));
    EIMSK |=  (1 << INT2) | (1 << INT3);

    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT2) | (1 << PCINT3);

    enc2_last = PINB & ((1 << ENC2_A_BIT) | (1 << ENC2_B_BIT));
}

ISR(INT2_vect) { 
    uint8_t a = (ENC1_PINREG >> ENC1_A_BIT) & 1;
    uint8_t b = (ENC1_PINREG >> ENC1_B_BIT) & 1;
    enc1_count += (a ^ b) ? -1 : 1;
}

ISR(INT3_vect) { 
    uint8_t a = (ENC1_PINREG >> ENC1_A_BIT) & 1;
    uint8_t b = (ENC1_PINREG >> ENC1_B_BIT) & 1;
    enc1_count += (a ^ b) ? 1 : -1;
}

ISR(PCINT0_vect) {
    uint8_t curr    = PINB & ((1 << ENC2_A_BIT) | (1 << ENC2_B_BIT));
    uint8_t changed = curr ^ enc2_last;
    uint8_t a       = (curr >> ENC2_A_BIT) & 1;
    uint8_t b       = (curr >> ENC2_B_BIT) & 1;

    if (changed & (1 << ENC2_A_BIT))
        enc2_count += (a ^ b) ? 1 : -1;

    if (changed & (1 << ENC2_B_BIT))
        enc2_count += (a ^ b) ? -1 : 1;

    enc2_last = curr;
}

int32_t encoder1_get(void) {
    int32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = enc1_count; }
    return v;
}

int32_t encoder2_get(void) {
    int32_t v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = enc2_count; }
    return v;
}

void encoder1_reset(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { enc1_count = 0; }
}

void encoder2_reset(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { enc2_count = 0; }
}

// Call once per fixed time interval (e.g. in your 100 Hz PID tick).
// After calling, use encoder1_speed() / encoder2_speed() to read ticks/interval.
void encoder_speed_update(void) {
    int32_t c1, c2;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { c1 = enc1_count; c2 = enc2_count; }
    enc1_speed = c1 - enc1_prev;
    enc2_speed = c2 - enc2_prev;
    enc1_prev  = c1;
    enc2_prev  = c2;
}

// get speed in ticks/interval since last encoder_speed_update() call
int32_t encoder1_speed(void) { return enc1_speed; }
int32_t encoder2_speed(void) { return enc2_speed; }
