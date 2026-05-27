#include "battery.h"
#include "pins.h"
#include <avr/io.h>

void battery_init(void) {
    DDRF  &= ~(1 << PF1);
    PORTF &= ~(1 << PF1);

    // ADC: AVCC ref, prescaler /128 (same config as mux)
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // Warm-up conversion
    ADMUX  = (1 << REFS0) | BATT_ADC_CH;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
}

uint16_t battery_read_raw(void) {
    ADMUX  = (1 << REFS0) | BATT_ADC_CH;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint16_t battery_read_mv(void) {
    uint32_t raw    = battery_read_raw();
    uint32_t adc_mv = (raw * BATT_AVCC_MV) / 1023UL;
    uint32_t div    = (BATT_R_TOP_KOHM + BATT_R_BOT_KOHM);
    return (uint16_t)((adc_mv * div) / BATT_R_BOT_KOHM);
}
