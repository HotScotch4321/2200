#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t c, r, g, b;
} RGBCData;

typedef enum {
    TCS_COLOUR_UNKNOWN = 0,
    TCS_COLOUR_RED,
    TCS_COLOUR_GREEN,
    TCS_COLOUR_WHITE
} TCSColour;

bool     tcs34725_init(void);
bool     tcs34725_read(RGBCData *out);
TCSColour tcs34725_classify(const RGBCData *d);