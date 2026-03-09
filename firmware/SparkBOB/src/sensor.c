#include "sensor.h"
// enable pins for sensors
// PF4, PF5, PF6, PF7, PB4, PD7, PD6, and PD4

void init_ADC() 
{
    // set all pins to ADC input
    ADMUX |= (1 << REFS0); // AVcc reference
    ADCSRA |= (1 << ADEN) | (1 << ADATE) | (7 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // enable auto trigger, adc, prescaler division factor 128

    // high speed mode?
    ADCSRB |= (1 << ADHSM);
}

uint16_t read_ADC(uint8_t channel)
{
    // clear ad mux from register ADCSRB admux 5 and the admux 4:0
    ADCSRB &= ~(1 << MUX5);
    ADMUX &= 0xE0; 

    if (channel >= 8) 
    {
        ADCSRB |= (1 << MUX5);
        channel -= 8;
    }

    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)); // wait for conversion to complete

    return ADC; // 10-bit ADC value (0-1023)
}

 bool read_sensor(uint8_t sensor_num) 
{
    uint16_t adc_value = read_ADC(sensor_num);
    return (adc_value < SENSOR_THRESHOLD) ? true : false; // return true (line detected) if below threshold, else false (no line detected)
}

// TODO: should test the threshold value of sensor reading to ensure appropriate sensitivity
