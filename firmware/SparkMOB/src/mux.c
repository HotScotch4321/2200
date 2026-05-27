#include "mux.h"
#include "pins.h"
#include <avr/io.h>

static uint16_t adc_read_ch(uint8_t ch) {
    ADMUX  = (1 << REFS0) | (ch & 0x07);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void mux_init(void) {
    // S0–S3 as outputs
    MUX_DDR |= (1 << MUX_S0_BIT) | (1 << MUX_S1_BIT) |
               (1 << MUX_S2_BIT) | (1 << MUX_S3_BIT);

    // PF0 (ADC0) input, no pull-up
    DDRF  &= ~(1 << PF0);
    PORTF &= ~(1 << PF0);

    // ADC: AVCC ref, prescaler /128 → 125 kHz sample clock
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    adc_read_ch(MUX_ADC_CH);   // warm-up conversion
}

uint16_t mux_read(uint8_t channel) {
    // Set S3–S0 in upper nibble of PORTF, preserve lower nibble
    MUX_PORT = (MUX_PORT & 0x0F) | ((channel & 0x0F) << 4);

    // ~1 µs settling time for mux switch
    __asm__ __volatile__("nop\nnop\nnop\nnop\n");

    adc_read_ch(MUX_ADC_CH);           // discard – channel just switched
    return adc_read_ch(MUX_ADC_CH);
}
