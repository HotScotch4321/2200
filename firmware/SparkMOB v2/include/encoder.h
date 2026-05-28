#pragma once
#include <stdint.h>

void    encoder_init(void);
int32_t encoder1_get(void);
int32_t encoder2_get(void);
void    encoder1_reset(void);
void    encoder2_reset(void);

// Call at a fixed rate; then read speed as ticks per that interval
void    encoder_speed_update(void);
int32_t encoder1_speed(void);
int32_t encoder2_speed(void);
