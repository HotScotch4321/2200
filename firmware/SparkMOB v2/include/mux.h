#pragma once
#include <stdint.h>

void     mux_init(void);
uint16_t mux_read(uint8_t channel);   // channel 0–15, returns 10-bit ADC value
