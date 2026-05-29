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

// Indicators (LEDS)
#define IND0_DDR    DDRC // RED LED - PC7 - stle from ultrasonic pin
#define IND0_PORT   PORTC           
#define IND0_PIN    PINC        
#define IND0_BIT    PC7    

#define IND1_DDR   DDRD // GREEN LED / buzzer (shared pin)- ADC11 PD4 also stole form ultrasonic
#define IND1_PORT  PORTD
#define IND1_PIN   PIND
#define IND1_BIT   PD4

#define IND2_DDR   DDRC // BLUE LED - ADC11 PC6 - curved wire
#define IND2_PORT  PORTC
#define IND2_PIN   PINC
#define IND2_BIT   PC6

#define SIG_DDR     DDRB // Track / size signal inputs (Timer1 OC pins used as GPIO)
#define SIG_PORT    PORTB
#define SIG_PINREG  PINB
#define SIG_TRK_BIT PB6   // OC1B
#define SIG_SZ_BIT  PB5   // OC1A
