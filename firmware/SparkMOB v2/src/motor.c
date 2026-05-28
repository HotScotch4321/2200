#include "motor.h"

// Motor control functions
void setupMotors(void) {
    // Set Direction and PWM pins as outputs
    DDRB |= (1 << PB0) | (1 << PB7); // Motor 1 (Left): Dir PB0, PWM PB7
    DDRD |= (1 << PD0);              // Motor 2 (Right): PWM PD0
    DDRE |= (1 << PE6);              // Motor 2 (Right): Dir PE6

    // Fast PWM for Timer 0
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); 

    // Prescaler 8
    TCCR0B = (1 << CS01); 

    OCR0A = 0; // PWM for Motor 1 (Left, PB7)
    OCR0B = 0; // PWM for Motor 2 (Right, PD0)
    
    motor1Direction(CW); // motor direction 1
    motor2Direction(CW);
}

void motor1Direction(int direction) {
    if (direction == CW) PORTB |= (1 << PB0);
    else PORTB &= ~(1 << PB0);
}

void motor2Direction(int direction) {
    if (direction == CW) PORTE |= (1 << PE6);
    else PORTE &= ~(1 << PE6);
}

void motor1Speed(uint8_t speed) { // 0 - 255
    OCR0A = speed;
}

void motor2Speed(uint8_t speed) {
    OCR0B = speed;
}