#pragma once
#include <stdint.h>

// LED index: 0 = RED, 1 = GREEN, 2 = BLUE

void led_init(void);
void led_set(uint8_t idx, uint8_t on);
void led_toggle(uint8_t idx);
void led_all(uint8_t on);
