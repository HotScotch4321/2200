#pragma once
#include <stdint.h>

void    twi_init(void);
uint8_t twi_start_write(uint8_t addr);
uint8_t twi_start_read(uint8_t addr);
uint8_t twi_rep_start_write(uint8_t addr);
uint8_t twi_write(uint8_t data);
uint8_t twi_read_ack(void);
uint8_t twi_read_nack(void);
void    twi_stop(void);
