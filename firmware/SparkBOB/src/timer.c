#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"

// Define the volatile flag that tells main() when to run the PID loop
volatile uint8_t pid_run_flag = 0;

// Setup timer 1 for 100Hz interrupt (10ms)
void init_timer() 
{
    // Disable global interrupts during setup
    cli();
    
    // Clear Timer/Counter Control Registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // We want 100Hz. F_CPU is usually 16,000,000.
    // Timer clock = F_CPU / 8 (Prescaler) = 2,000,000 Hz.
    // Target ticks = 2,000,000 / 100 = 20,000 ticks.
    // Compare match register = Target ticks - 1 = 19999.
    OCR1A = 19999; 
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    sei();
}

static volatile uint16_t timer_ticks = 0;

// Interrupt Service Routine that fires at 100Hz
ISR(TIMER1_COMPA_vect)
{
    timer_ticks++;
    // Tell the main loop it is time to do math
    pid_run_flag = 1; 
}