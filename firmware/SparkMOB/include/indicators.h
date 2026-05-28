#pragma once
#include <stdint.h>

// indicator index: 0 = RED, 1 = GREEN/buzzer, 2 = BLUE

void indicators_init(void);
void indicators_set(uint8_t idx, uint8_t on);
void indicators_toggle(uint8_t idx);
void indicators_all(uint8_t on);
