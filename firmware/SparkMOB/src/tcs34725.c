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

bool tcs34725_init(void) {
    twi_init();
    _delay_ms(10);

    uint8_t id = read_reg(REG_ID);
    if (id != 0x44) 
        return false;

    write_reg(REG_ATIME,   0xF6);   // 24 ms 0xF6 integration OR 12ms 0xFB OR 2.4ms 0xFE
    write_reg(REG_CONTROL, 0x01);   // 4x gain
    write_reg(REG_ENABLE,  ENABLE_PON);
    _delay_ms(3);
    write_reg(REG_ENABLE,  ENABLE_PON | ENABLE_AEN);
    _delay_ms(60); // wait the chip spends 50ms counting photons before it has a valid reading for integration time
    return true;
}

bool tcs34725_read(RGBCData *out) {
    if (!(read_reg(REG_STATUS) & STATUS_AVALID)) return false;

    out->c = read_reg(REG_CDATAL) | (read_reg(REG_CDATAL + 1) << 8);
    out->r = read_reg(REG_CDATAL + 2) | (read_reg(REG_CDATAL + 3) << 8);
    out->g = read_reg(REG_CDATAL + 4) | (read_reg(REG_CDATAL + 5) << 8);
    out->b = read_reg(REG_CDATAL + 6) | (read_reg(REG_CDATAL + 7) << 8);
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
