#pragma once

#include <avr/io.h>
#include <stdint.h>

void init_LEDS();
void LED_on(uint8_t led_num, int8_t brightness);
void init_buttons();
void button_led_control();
