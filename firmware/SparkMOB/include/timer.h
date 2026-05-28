#pragma once

#include <avr/io.h>
#include <stdint.h>

// Initialize Timer1 for 100Hz
void init_timer();

// Flag set by the ISR every 10ms
extern volatile uint8_t pid_run_flag;   