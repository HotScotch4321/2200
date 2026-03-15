#pragma once

#include <avr/io.h>
#include <stdint.h>

void init_LEDS();
void LED_on(uint8_t led_num);
void init_buttons();
void button_led_control();
void LED_off(uint8_t led_num);
