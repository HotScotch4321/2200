#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins.h"
#include "encoder.h"
#include "indicators.h"
#include "mux.h"
#include "tcs34725.h"
#include "pid.h"
// #include "battery.h" No Don't need it 

typedef enum {
    STATE_STRAIGHT,
    STATE_TURN_LEFT,        // huh? not sure we need this   
    STATE_TURN_RIGHT,       // huh? not sure we need this          
    STATE_LOST, 
    STATE_FOLLOW_LINE,
    STATE_SLOW_ZONE,
    STATE_PUSH_OBSTACLE,
    STATE_START_FINISH_STOP,
    STATE_LOST_LINE
} RobotState;

// Finish-area stop settings
#define RED_LED 0             // Replace with the actual red LED number
#define STOP_TICKS 200        // 100 Hz = 2 seconds

RobotState current_state = STATE_FOLLOW_LINE;
uint16_t stop_counter = 0;


static uint8_t finish_confirm = 0;
uint8_t finish_marker_detected(void) {
    int32_t pos = get_position();
    if (pos == 100) { finish_confirm = 0; return 0; }

    if (mux_read(13) && pos > -40 && pos < 40) {
        if (finish_confirm < 3) finish_confirm++;
    } else {
        finish_confirm = 0;
    }
    return finish_confirm >= 3;
}

// Payloaed communication signals:
static inline void sig_init(void) {
    SIG_DDR  |=  (1 << SIG_SZ_BIT);    // SZ output
    SIG_DDR  &= ~(1 << SIG_TRK_BIT);   // TRK input
    SIG_PORT &= ~(1 << SIG_SZ_BIT);    // SZ idle LOW
    SIG_PORT |=  (1 << SIG_TRK_BIT);   // TRK pull-up
}

static inline void sig_sz_set(uint8_t high) { //   SIG_SZ  (PB5, output): HIGH = in slow zone, LOW = not
    if (high) SIG_PORT |=  (1 << SIG_SZ_BIT);
    else      SIG_PORT &= ~(1 << SIG_SZ_BIT);
}

static inline uint8_t sig_trk_get(void) { //   SIG_TRK (PB6, input):  HIGH = light follower tracking, LOW = not
    return (SIG_PINREG >> SIG_TRK_BIT) & 1;
}

//Red rectangular marker = start of slow zone 
//Green rectangular marker = end of slow zone 
// for tcs34725 - needs to detect red and green markers to determine when in slow zone

void main_loop(void) {
    // state machine
    if (pid_run_flag) {
        pid_run_flag = 0; // Clear it so we wait for the next tick
    }
    switch (current_state) {
        case STATE_START_FINISH_STOP:
            motor1Speed(0);
            motor2Speed(0);
            LED_off(2);
            LED_on(RED_LED);

            stop_counter++;
            if (stop_counter >= STOP_TICKS) {
                stop_counter = 0;
                LED_off(RED_LED);
                current_state = STATE_FOLLOW_LINE;
            }
            break;
        case STATE_FOLLOW_LINE:
            int16_t pid_output = compute_PID();
            adjust_motor_speed(pid_output);

            if (finish_marker_detected() && !finish_confirm) {
                current_state = STATE_START_FINISH_STOP;
                stop_counter = 0;
                finish_confirm = 0;
                break;
            } 
    }       // TODO: I'll fin this - zone slow detection, obstacle detect, last line, led, go circle and know when to stop
    // when in slow zone, set SIG_SZ HIGH, otherwise LOW. This is for the payload communication to the main controller.
}

int main(void) {
    sei();
    led_init();
    mux_init();
    sig_init();
    encoder_init();
    // battery_init();
    // ultrasonic_init();
    tcs34725_init();

    while (1) {
        // Heartbeat
        led_toggle(2);

        // Mirror TRK input on LED1 for testing
        led_set(1, sig_trk_get());

        // TODO: slow-zone detection — drive SIG_SZ HIGH when in slow zone
        sig_sz_set(0);

        _delay_ms(100);
    }
}
