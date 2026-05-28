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
#define FINISH_SENSOR 1      // I'm assuming 1 as it's left most?
#define RED_LED 0             // Replace with the actual red LED number
#define STOP_TICKS 200        // 100 Hz = 2 seconds

RobotState current_state = STATE_FOLLOW_LINE;
uint16_t stop_counter = 0;

// Prevents cycling start/stop if sesnor is over a "Start - Finish" area marker
uint8_t finish_lockout = 0;

// Detects finish marker on right side of track
uint8_t finish_marker_detected(void) 
{
    // needs proper logic conditions? lap counter maybe?
    return read_sensor_binary(FINISH_SENSOR);
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

            if (finish_marker_detected() && !finish_lockout) {
                current_state = STATE_START_FINISH_STOP;
                stop_counter = 0;
                finish_lockout = 1;
                break;
            } 
    }       // TODO: I'll fin this - zone slow detection, obstacle detect, last line, led, go circle and know when to stop
    // when in slow zone, set SIG_SZ HIGH, otherwise LOW. This is for the payload communication to the main controller.
}

void loop(void)
{
    if (pid_run_flag) {
        pid_run_flag = 0; // Clear it so we wait for the next tick

    // Stops for 2 seconds in the "Start - Finish" area
    if (current_state == STATE_START_FINISH_STOP) {
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

        return;
    }

    // Detect finish area marker -> enter stop state
    if (finish_marker_detected() && !finish_lockout) {
        current_state = STATE_START_FINISH_STOP;
        stop_counter = 0;
        finish_lockout = 1;
        return;
    }

// "Re-arms" finish area detection
    if (!finish_marker_detected()) {
        finish_lockout = 0;
    }

    // Normal Line-Following
    int16_t pid_output = compute_PID();
    adjust_motor_speed(pid_output);

    // Green LED/Buzzer or both - for straight section
    if (robot_on_straight) {
            LED_on(2);   // Replace 2 with the actual number
        } else {
            LED_off(2);
        }

    }
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
