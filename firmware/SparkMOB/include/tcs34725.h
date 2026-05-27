#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t c, r, g, b;
} RGBCData;

typedef enum {
    TCS_COLOR_UNKNOWN = 0,
    TCS_COLOR_RED,
    TCS_COLOR_GREEN,
    TCS_COLOR_WHITE
} TCSColor;

bool     tcs34725_init(void);
bool     tcs34725_read(RGBCData *out);
TCSColor tcs34725_classify(const RGBCData *d);