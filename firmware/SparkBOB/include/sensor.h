#pragma once

#include <avr/io.h>
#include <stdint.h>

constexpr uint16_t SENSOR_THRESHOLD = 512;

void init_ADC();
uint16_t read_ADC(uint8_t channel);
bool read_sensor(uint8_t sensor_num);

