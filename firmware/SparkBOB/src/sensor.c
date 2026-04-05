#include "sensor.h"

// Array mapping sequential sensors S1-S8 to their physical ADC channels
// S1=ADC4, S2=ADC5, S3=ADC6, S4=ADC7, S5=ADC11, S6=ADC10, S7=ADC9, S8=ADC8
const uint8_t sensor_channels[8] = {4, 5, 6, 7, 11, 10, 9, 8};

void init_ADC() 
{
    // Set reference to AVcc
    ADMUX |= (1 << REFS0); 
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 

    // Enable High Speed Mode
    ADCSRB |= (1 << ADHSM);
}

uint16_t read_ADC(uint8_t channel)
{
    // Clear MUX5 in ADCSRB and MUX4:0 in ADMUX
    ADCSRB &= ~(1 << MUX5);
    ADMUX &= 0xE0; 

    // Handle high channels (ADC8-ADC11)
    if (channel >= 8) 
    {
        ADCSRB |= (1 << MUX5);
        channel -= 8;
    }

    ADMUX |= (channel & 0x07);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC)); 

    return ADC; 
}

bool read_sensor_binary(uint8_t sensor_index) 
{
    // Prevent out-of-bounds array access
    if (sensor_index > 7) return false; 
    
    // Route the requested sensor index through the channel map
    uint16_t adc_value = read_ADC(sensor_channels[sensor_index]);
    
    // Return true if seeing white (below threshold)
    return (adc_value < SENSOR_THRESHOLD); 
}

uint16_t read_sensor(uint8_t sensor_index)
{
    if (sensor_index > 7) return 0; // Prevent out-of-bounds access
    
    return read_ADC(sensor_channels[sensor_index]);
}

