#pragma once
#include <stdint.h>

// HC-SR04 
// TRIG = PC7  – simple GPIO pulse (10 µs).
// ECHO = PD4  – Timer1 Input Capture (ICP1), prescaler /8 → 0.5 µs/tick.
// Do not use OC1A (PB5) or OC1B (PB6) as PWM outputs while this driver is active.

void     ultrasonic_init(void);
uint32_t ultrasonic_measure_mm(void);   
