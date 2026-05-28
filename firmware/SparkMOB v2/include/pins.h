#pragma once
#include <avr/io.h>
// SDA = PD1, SCL = PD0

// Motor 1 encoder
#define ENC1_A_BIT  PD3   // INT3
#define ENC1_B_BIT  PD2   // INT2
#define ENC1_PINREG PIND

// Motor 2 encoder
#define ENC2_A_BIT  PB3   // PCINT3
#define ENC2_B_BIT  PB2   // PCINT2
#define ENC2_PINREG PINB

// 74HC4067 mux – select lines
#define MUX_DDR     DDRF
#define MUX_PORT    PORTF
#define MUX_S0_BIT  PF4
#define MUX_S1_BIT  PF5
#define MUX_S2_BIT  PF6
#define MUX_S3_BIT  PF7
#define MUX_ADC_CH  0     // PF0 = ADC0

#define BATT_ADC_CH 1     // PF1 = ADC1

// Ultrasonic: TRIG = PC7, ECHO = PD4 (ICP1)
#define ULTRASONIC_TRIG_DDR  DDRC
#define ULTRASONIC_TRIG_PORT PORTC
#define ULTRASONIC_TRIG_BIT  PC7
#define ULTRASONIC_ECHO_DDR  DDRD
#define ULTRASONIC_ECHO_PORT PORTD
#define ULTRASONIC_ECHO_BIT  PD4   // ICP1

// LEDs
#define LED0_DDR    DDRE
#define LED0_PORT   PORTE
#define LED0_PIN    PINE
#define LED0_BIT    PE6

#define LED12_DDR   DDRB
#define LED12_PORT  PORTB
#define LED12_PIN   PINB
#define LED1_BIT    PB0
#define LED2_BIT    PB1

#define SIG_DDR     DDRB // Track / size signal inputs (Timer1 OC pins used as GPIO)
#define SIG_PORT    PORTB
#define SIG_PINREG  PINB
#define SIG_TRK_BIT PB6   // OC1B
#define SIG_SZ_BIT  PB5   // OC1A
