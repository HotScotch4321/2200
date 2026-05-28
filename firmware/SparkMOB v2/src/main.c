#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pins.h"
#include "encoder.h"
#include "led.h"
#include "mux.h"
#include "tcs34725.h"
#include "pid.h"
#include "motor.h"
#include "sensor_array.h"
#include "battery.h"

typedef enum {
    STATE_STRAIGHT,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_LOST,
    STATE_FOLLOW_LINE,
    STATE_SLOW_ZONE,
    STATE_PUSH_OBSTACLE,
    STATE_START_FINISH_STOP,
    STATE_LOST_LINE
} RobotState;


// Checks whether sensors are detecting the line
bool read_sensor_binary(uint8_t sensor_num)
{
    return mux_read(sensor_num) > SENSOR_THRESHOLD;
}


RobotState current_state = STATE_FOLLOW_LINE;
uint16_t stop_counter = 0;

// Prevents cycling start/stop if sesnor is over a "Start - Finish" area marker
uint8_t finish_lockout = 0;

// Detects finish marker on right side of track
uint8_t finish_marker_detected(void)
{
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

// Slow zone
bool slow_zone_active = false;

void update_slow_zone(void)
{
    RGBCData colour;

    if (tcs34725_read(&colour)) {
        TCSColour detected_colour = tcs34725_classify(&colour);

        if (detected_colour == TCS_COLOUR_RED) {
            slow_zone_active = true;
            sig_sz_set(1);
        }

        if (detected_colour == TCS_COLOUR_GREEN) {
            slow_zone_active = false;
            sig_sz_set(0);
        }
    }
}


// Red rectangular marker = start of slow zone 
// Green rectangular marker = end of slow zone 
// For tcs34725 - needs to detect red and green markers to determine when in slow zone

void loop(void)
{
    if (pid_run_flag) {
        pid_run_flag = 0; // Clear it so we wait for the next tick

    // Stops for 2 seconds in the "Start - Finish" area
    if (current_state == STATE_START_FINISH_STOP) {
        motor1Speed(0);
        motor2Speed(0);
        led_set(RED_LED, 1);   // Turns on LED
        //led_set(RED_LED, 0);   // Turns off LED

        stop_counter++;

        if (stop_counter >= STOP_TICKS) {
            stop_counter = 0;
            led_set(RED_LED, 0);
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
            led_set(RED_LED, 1);   // Replace 2 with the actual number
        } else {
            led_set(RED_LED, 0);
        }

    }
}


int main(void) {
    sei();
    led_init();
    mux_init();
    sig_init();
    encoder_init();
    setupMotors();
    // battery_init();
    // ultrasonic_init();
    tcs34725_init();

    while (1) {

        pid_run_flag = 1;    //
        loop();

        // Heartbeat
        led_toggle(2);

        // Mirror TRK input on LED1 for testing
        led_set(1, sig_trk_get());

        update_slow_zone();

        _delay_ms(10);
    }
}
