#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "pins.h"
#include "encoder.h"
#include "indicators.h"
#include "mux.h"
#include "tcs34725.h"
#include "switch.h"
#include "motor.h"
#include "pid.h"
#include "timer.h"

#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE   2

#define LEFT_SENSOR      0          // leftmost line sensor (corner markers)
#define FINISH_SENSOR    13         // rightmost line sensor (start-finish marker)
#define SENSOR_THRESHOLD 512        // ADC count above which a sensor sees white

#define STOP_TICKS        200       // 2 s at 100 Hz
#define MARKER_CONFIRM    3
#define ROUTE_BIAS_TICKS  60
#define ROUTE_BIAS_MAG    25

extern uint8_t robot_in_slow_zone;  // defined in pid.c, drives SLOW_SPEED base

typedef enum {
    STATE_FOLLOW_LINE,
    STATE_SLOW_ZONE,
    STATE_START_FINISH_STOP
} RobotState;

static RobotState state = STATE_FOLLOW_LINE;

static uint16_t stop_counter   = 0;
static uint8_t  lap_count      = 0;
static uint16_t route_bias_ctr = 0;

static uint8_t finish_confirm = 0;
static uint8_t red_confirm    = 0;
static uint8_t green_confirm  = 0;
static uint8_t left_confirm   = 0;
static uint8_t left_latched   = 0;

static inline void sig_init(void) {
    SIG_DDR  |=  (1 << SIG_SZ_BIT);
    SIG_DDR  &= ~(1 << SIG_TRK_BIT);
    SIG_PORT &= ~(1 << SIG_SZ_BIT);
    SIG_PORT |=  (1 << SIG_TRK_BIT);
}

static inline void sig_sz_set(uint8_t high) {
    if (high) SIG_PORT |=  (1 << SIG_SZ_BIT);
    else      SIG_PORT &= ~(1 << SIG_SZ_BIT);
}

// Read both edge 
static void read_edges(uint8_t *left_white, uint8_t *right_white) {
    *left_white  = mux_read(LEFT_SENSOR)   > SENSOR_THRESHOLD;
    *right_white = mux_read(FINISH_SENSOR) > SENSOR_THRESHOLD;
}

// Left marker only: a crossover lights both edges, so right_white rejects it.
// Markers sit at the start and end of every arc, so each one toggles state.
static void corner_marker_update(uint8_t left_white, uint8_t right_white) {
    uint8_t marker = left_white && !right_white;
    if (marker) { if (left_confirm < MARKER_CONFIRM) left_confirm++; }
    else left_confirm = 0;

    uint8_t seen = left_confirm >= MARKER_CONFIRM;
    if (seen && !left_latched) robot_on_straight = !robot_on_straight;
    left_latched = seen;

    led_set(LED_GREEN, robot_on_straight);
}

// Right marker only, rejecting crossovers the same way.
static uint8_t finish_marker_detected(uint8_t left_white, uint8_t right_white) {
    uint8_t marker = right_white && !left_white;
    if (marker) { if (finish_confirm < MARKER_CONFIRM) finish_confirm++; }
    else finish_confirm = 0;
    return finish_confirm >= MARKER_CONFIRM;
}

static uint8_t red_marker_detected(void) {
    RGBCData d;
    if (tcs34725_read(&d) && tcs34725_classify(&d) == TCS_COLOUR_RED) {
        if (red_confirm < MARKER_CONFIRM) red_confirm++;
    } else red_confirm = 0;
    return red_confirm >= MARKER_CONFIRM;
}

static uint8_t green_marker_detected(void) {
    RGBCData d;
    if (tcs34725_read(&d) && tcs34725_classify(&d) == TCS_COLOUR_GREEN) {
        if (green_confirm < MARKER_CONFIRM) green_confirm++;
    } else green_confirm = 0;
    return green_confirm >= MARKER_CONFIRM;
}

void drive(void) {
    switch (state) {

    case STATE_START_FINISH_STOP:
        motors_stop();
        led_set(LED_RED,   1);
        led_set(LED_GREEN, 0);
        led_set(LED_BLUE,  0);
        if (++stop_counter >= STOP_TICKS) {
            stop_counter = 0;
            led_set(LED_RED, 0);
            state = STATE_FOLLOW_LINE;
        }
        break;

    case STATE_FOLLOW_LINE: {
        uint8_t lw, rw;
        read_edges(&lw, &rw);
        corner_marker_update(lw, rw);

        robot_in_slow_zone = 0;
        sig_sz_set(0);
        led_set(LED_BLUE, 0);

        adjust_motor_speed(compute_PID());

        if (finish_marker_detected(lw, rw)) {
            lap_count++;
            finish_confirm = 0;
            stop_counter   = 0;
            state = STATE_START_FINISH_STOP;
            break;
        }
        if (red_marker_detected()) {
            red_confirm    = 0;
            route_bias_ctr = ROUTE_BIAS_TICKS;
            state = STATE_SLOW_ZONE;
        }
        break;
    }

    case STATE_SLOW_ZONE: {
        uint8_t lw, rw;
        read_edges(&lw, &rw);
        corner_marker_update(lw, rw);

        robot_in_slow_zone = 1;        // forces SLOW_SPEED in adjust_motor_speed
        sig_sz_set(1);
        led_set(LED_BLUE, bump_read(0) || bump_read(1));

        int16_t pid_output = compute_PID();

        // Alternate the fork each lap: even laps left, odd laps right.
        // Hold the bias until the robot commits, then let tracking resume.
        if (route_bias_ctr > 0) {
            route_bias_ctr--;
            pid_output += (lap_count % 2 == 0) ? -ROUTE_BIAS_MAG : ROUTE_BIAS_MAG;
        }
        adjust_motor_speed(pid_output);

        if (green_marker_detected()) {
            green_confirm  = 0;
            route_bias_ctr = 0;
            robot_in_slow_zone = 0;
            sig_sz_set(0);
            state = STATE_FOLLOW_LINE;
        }
        break;
    }
    }
}

int main(void) {
    led_init();
    mux_init();
    motor_init();
    encoder_init();
    sig_init();
    tcs34725_init();
    bump_init();
    timer_init();

    led_all(0);
    motors_stop();
    robot_on_straight = 1;
    sei();

    while (1) {
        if (pid_run_flag) {
            pid_run_flag = 0;
            drive();
        }
    }
}