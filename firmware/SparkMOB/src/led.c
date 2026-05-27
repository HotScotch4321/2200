#include "led.h"
#include "pins.h"
#include <avr/io.h>

void led_init(void) {
    LED0_DDR  |= (1 << LED0_BIT);
    LED12_DDR |= (1 << LED1_BIT) | (1 << LED2_BIT);
    LED0_PORT  &= ~(1 << LED0_BIT);
    LED12_PORT &= ~((1 << LED1_BIT) | (1 << LED2_BIT));
}

void led_set(uint8_t idx, uint8_t on) {
    switch (idx) {
        case 0: // RED
            if (on) LED0_PORT  |=  (1 << LED0_BIT);
            else    LED0_PORT  &= ~(1 << LED0_BIT);
            break;
        case 1: // GREEN
            if (on) LED12_PORT |=  (1 << LED1_BIT);
            else    LED12_PORT &= ~(1 << LED1_BIT);
            break;
        case 2: // BLUE
            if (on) LED12_PORT |=  (1 << LED2_BIT);
            else    LED12_PORT &= ~(1 << LED2_BIT);
            break;
    }
}

void led_toggle(uint8_t idx) {
    switch (idx) {
        case 0: // RED
            LED0_PIN  = (1 << LED0_BIT);
            break;
        case 1: // GREEN
            LED12_PIN = (1 << LED1_BIT);
            break;
        case 2: // BLUE
            LED12_PIN = (1 << LED2_BIT);
            break;
    }
}

void led_all(uint8_t on) {
    led_set(0, on);
    led_set(1, on);
    led_set(2, on);
}
