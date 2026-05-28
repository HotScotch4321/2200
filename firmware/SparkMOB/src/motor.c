#include "motor.h"
#include <avr/io.h>

void motor_init(void) {
    // Direction pins as outputs
    DDRB |= (1 << PB0);   // Motor 1 direction
    DDRE |= (1 << PE6);   // Motor 2 direction
 
    
    // PWM pins as outputs
    DDRB |= (1 << PB7);   // Motor 1 PWM (OC0A)
    DDRD |= (1 << PD0);   // Motor 2 PWM (OC0B)

    // Timer0: Fast PWM, non-inverting on OC0A and OC0B
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01);  // prescaler /8
    OCR0A = 0;
    OCR0B = 0;
    motor1_direction(MOTOR_FORWARD);
    motor2_direction(MOTOR_FORWARD);
}

void motor1_direction(uint8_t dir) {
    if (dir == MOTOR_FORWARD) PORTB |=  (1 << PB0);
    else                      PORTB &= ~(1 << PB0);
}

void motor2_direction(uint8_t dir) {
    if (dir == MOTOR_FORWARD) PORTE |=  (1 << PE6);
    else                      PORTE &= ~(1 << PE6);
}

void motor1_speed(uint8_t speed) { // 0-255
    OCR0A = speed;
}

void motor2_speed(uint8_t speed) {
    OCR0B = speed;
}

void motors_stop(void) {
    OCR0A = 0;
    OCR0B = 0;
}
