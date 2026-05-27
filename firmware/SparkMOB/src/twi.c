#include "twi.h"
#include <avr/io.h>
#include <util/twi.h>

// 400 kHz @ 16 MHz: TWBR = (16 000 000 / 400 000 - 16) / 2 = 12
#ifndef TWI_FREQ
#define TWI_FREQ 400000UL
#endif

void twi_init(void) {
    TWSR = 0x00;
    TWBR = (uint8_t)((F_CPU / TWI_FREQ - 16) / 2);
    TWCR = (1 << TWEN);
}

static uint8_t _start(uint8_t addr_rw) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    uint8_t s = TWSR & 0xF8;
    if (s != TW_START && s != TW_REP_START) return s;

    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWSR & 0xF8;
}

uint8_t twi_start_write(uint8_t addr) {
    uint8_t s = _start((uint8_t)(addr << 1) | TW_WRITE);
    return (s == TW_MT_SLA_ACK) ? 0 : s;
}

uint8_t twi_start_read(uint8_t addr) {
    uint8_t s = _start((uint8_t)(addr << 1) | TW_READ);
    return (s == TW_MR_SLA_ACK) ? 0 : s;
}

uint8_t twi_rep_start_write(uint8_t addr) {
    return twi_start_write(addr);
}

uint8_t twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    uint8_t s = TWSR & 0xF8;
    return (s == TW_MT_DATA_ACK) ? 0 : s;
}

uint8_t twi_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t twi_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}
