#include <avr/io.h>
#include "led.h"

// LED0: PE6 LED1: PB0 LED2: PB1 LED3: PB2 LED4: PB7 LED5: PD0 LED6: PB6 LED7: PB5 
void init_LEDS()
{
    // Set all LED pins as outputs
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB5) | (1 << PB6) | (1 << PB7);
    DDRE |= (1 << PE6);
    DDRD |= (1 << PD0); 

    // Initialize duty cycle to zero
    OCR1A = 0;
    OCR1B = 0;

    // Setup Timer 1 for Fast PWM, 8-bit. 
    // Notice we do NOT set COM1A1 or COM1B1 here. We leave them disconnected initially.
    TCCR1A = (1 << WGM10); 
    TCCR1B = (1 << WGM12) | (1 << CS11); 
}

void LED_on(uint8_t led_num, uint8_t brightness)
{
    if (led_num == 6) {
        TCCR1A |= (1 << COM1B1); // Connect Timer 1B to PB6
        OCR1B = brightness;
    } else if (led_num == 7) {
        TCCR1A |= (1 << COM1A1); // Connect Timer 1A to PB5
        OCR1A = brightness;
    } else {
        switch (led_num) {
            case 0: PORTE |= (1 << PORTE6); break;
            case 1: PORTB |= (1 << PORTB0); break;
            case 2: PORTB |= (1 << PORTB1); break;
            case 3: PORTB |= (1 << PORTB2); break;
            case 4: PORTB |= (1 << PORTB7); break;
            case 5: PORTD |= (1 << PORTD0); break;
        }
    }
}

void LED_off(uint8_t led_num)
{
    if (led_num == 6) {
        TCCR1A &= ~(1 << COM1B1); // Disconnect Timer 1B from PB6
        PORTB &= ~(1 << PORTB6);  // Force pin LOW via standard GPIO
    } else if (led_num == 7) {
        TCCR1A &= ~(1 << COM1A1); // Disconnect Timer 1A from PB5
        PORTB &= ~(1 << PORTB5);  // Force pin LOW via standard GPIO
    } else {
        switch (led_num) {
            case 0: PORTE &= ~(1 << PORTE6); break;
            case 1: PORTB &= ~(1 << PORTB0); break;
            case 2: PORTB &= ~(1 << PORTB1); break;
            case 3: PORTB &= ~(1 << PORTB2); break;
            case 4: PORTB &= ~(1 << PORTB7); break;
            case 5: PORTD &= ~(1 << PORTD0); break;
        }
    }
}

// PC6 and PC7 are buttons, they are active low, so we can set them as input with pull-up resistors
void init_buttons()
{
    DDRC &= ~((1 << DDC6) | (1 << DDC7)); // set PC6 and PC7 as input
    PORTC |= (1 << PORTC6) | (1 << PORTC7); // enable pull-up resistors on PC6 and PC7
}

void button_led_control()
{
    if (!(PINC & (1 << PINC6))) { 
        LED_on(0, 255); // turn on LED0 at full brightness
    } else {
        PORTE &= ~(1 << PORTE6); 
    }

    if (!(PINC & (1 << PINC7))) { 
        LED_on(1, 255); // turn on LED1 at full brightness
    } else {
        PORTB &= ~(1 << PORTB0); 
    }
}