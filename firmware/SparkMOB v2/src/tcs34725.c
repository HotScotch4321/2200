#include "tcs34725.h"
#include "twi.h"
#include <util/delay.h>
#include <stdbool.h>

#define TCS_ADDR     0x29
#define TCS_CMD      0x80
#define TCS_CMD_AUTO 0x20

#define REG_ENABLE  0x00
#define REG_ATIME   0x01
#define REG_CONTROL 0x0F
#define REG_ID      0x12
#define REG_STATUS  0x13
#define REG_CDATAL  0x14
#define REG_RDATAL  0x16
#define REG_GDATAL  0x18
#define REG_BDATAL  0x1A

#define ENABLE_PON    0x01
#define ENABLE_AEN    0x02
#define STATUS_AVALID 0x01



static void write_reg(uint8_t reg, uint8_t val) {
    if (twi_start_write(TCS_ADDR)) { twi_stop(); return; }
    twi_write(TCS_CMD | reg);
    twi_write(val);
    twi_stop();
}

static uint8_t read_reg(uint8_t reg) {
    if (twi_start_write(TCS_ADDR)) { twi_stop(); return 0xFF; }
    twi_write(TCS_CMD | reg);
    if (twi_start_read(TCS_ADDR)) { twi_stop(); return 0xFF; }
    uint8_t val = twi_read_nack();
    twi_stop();
    return val;
}

static uint16_t read16_reg(uint8_t reg) {
    if (twi_start_write(TCS_ADDR)) {
        twi_stop();
        return 0;
    }

    twi_write(TCS_CMD | TCS_CMD_AUTO | reg);

    if (twi_start_read(TCS_ADDR)) {
        twi_stop();
        return 0;
    }

    uint8_t low = twi_read_ack();
    uint8_t high = twi_read_nack();

    twi_stop();

    return ((uint16_t)high << 8) | low;
}

bool tcs34725_read(RGBCData *out) {
    if (out == 0) {
        return false;
    }

    if (!(read_reg(REG_STATUS) & STATUS_AVALID)) {
        return false;
    }

    out->c = read16_reg(REG_CDATAL);
    out->r = read16_reg(REG_RDATAL);
    out->g = read16_reg(REG_GDATAL);
    out->b = read16_reg(REG_BDATAL);

    return true;
}

bool tcs34725_init(void) {
    twi_init();
    _delay_ms(10);

    uint8_t id = read_reg(REG_ID);
    if (id != 0x44) 
        return false;

    write_reg(REG_ATIME,   0xEB);   // 50 ms integration
    write_reg(REG_CONTROL, 0x01);   // 4x gain
    write_reg(REG_ENABLE,  ENABLE_PON);
    _delay_ms(3);
    write_reg(REG_ENABLE,  ENABLE_PON | ENABLE_AEN);
    _delay_ms(60); // wait the chip spends 50ms counting photons before it has a valid reading for integration time
    return true;
}

TCSColour tcs34725_classify(const RGBCData *d) {
    if (d->c < 100) return TCS_COLOUR_UNKNOWN;   // too dark, no reading

    // normalise to scale to 0-255
    uint8_t r = (uint32_t)d->r * 255 / d->c;
    uint8_t g = (uint32_t)d->g * 255 / d->c;
    uint8_t b = (uint32_t)d->b * 255 / d->c;

    if (r > 100 && r > g * 2 && r > b * 2) return TCS_COLOUR_RED;
    if (g > 100 && g > r * 2 && g > b * 2) return TCS_COLOUR_GREEN;
    if (r > 80  && g > 80   && b > 80)     return TCS_COLOUR_WHITE;

    return TCS_COLOUR_UNKNOWN;
}


