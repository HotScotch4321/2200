#pragma once
#include <stdint.h>

// Battery voltage sense on PF1 (ADC1).
// Adjust the divider constants below to match the actual resistor values
// on your board: V_bat = V_adc * (R_TOP + R_BOT) / R_BOT

#define BATT_R_TOP_KOHM  10
#define BATT_R_BOT_KOHM  10
#define BATT_AVCC_MV     5000UL

void     battery_init(void);
uint16_t battery_read_raw(void);       // raw 10-bit ADC count
uint16_t battery_read_mv(void);        // millivolts at battery terminal
