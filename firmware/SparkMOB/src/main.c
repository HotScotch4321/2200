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

#define LEFT_SENSOR      0          // leftmost line sensor 1 (corner markers)
#define LMID_SENSOR      4          // spread samples for crossover detection - sensor 5 is right-middle
#define RMID_SENSOR      9          // spread samples for crossover detection - sensor 10 is left-middle
#define FINISH_SENSOR    13         // rightmost line sensor 14 (start-finish marker)
#define SENSOR_THRESHOLD 512

#define STOP_TICKS        200       // 2 s at 100 Hz
#define MARKER_CONFIRM    3
#define MARKER_LOCKOUT    20        // ignore markers for ~200 ms after a crossover
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
static uint8_t  marker_lockout = 0;

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

// One array snapshot per tick. Crossover is judged from how many sensors are
// white at once, so it survives skew where one edge leads the other.
static void scan_markers(uint8_t *left_white, uint8_t *right_white,
                         uint8_t *crossover) {
    uint8_t l  = mux_read(LEFT_SENSOR)   > SENSOR_THRESHOLD;
    uint8_t lm = mux_read(LMID_SENSOR)   > SENSOR_THRESHOLD;
    uint8_t rm = mux_read(RMID_SENSOR)   > SENSOR_THRESHOLD;
    uint8_t r  = mux_read(FINISH_SENSOR) > SENSOR_THRESHOLD;

    *left_white  = l;
    *right_white = r;
    *crossover   = (l + lm + rm + r) >= MARKER_CONFIRM;
}

// Left marker toggles straight/curve. Blocked during and shortly after any
// crossover so a skewed crossing cannot flip the state.
static void corner_marker_update(uint8_t left_white, uint8_t right_white,
                                 uint8_t blocked) {
    uint8_t marker = !blocked && left_white && !right_white;
    if (marker) { if (left_confirm < MARKER_CONFIRM) left_confirm++; }
    else left_confirm = 0;

    uint8_t seen = left_confirm >= MARKER_CONFIRM;
    if (seen && !left_latched) robot_on_straight = !robot_on_straight;
    left_latched = seen;

    indicators_set(LED_GREEN, robot_on_straight);
}

static uint8_t finish_marker_detected(uint8_t left_white, uint8_t right_white,
                                      uint8_t blocked) {
    uint8_t marker = !blocked && right_white && !left_white;
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

static uint8_t update_lockout(uint8_t crossover) { // Maintain the crossover lockout from one array snapshot.
    if (crossover) marker_lockout = MARKER_LOCKOUT;
    else if (marker_lockout) marker_lockout--;
    return crossover || marker_lockout > 0;
}

void drive(void) {
    switch (state) {

    case STATE_START_FINISH_STOP:
        motors_stop();
        indicators_set(0, 1); // LED_RED
        indicators_set(1, 0); // LED_GREEN
        indicators_set(2, 0); // LED_BLUE
        if (++stop_counter >= STOP_TICKS) {
            stop_counter = 0;
            indicators_set(0, 0);
            state = STATE_FOLLOW_LINE;
        }
        break;

    case STATE_FOLLOW_LINE: {
        uint8_t lw, rw, cx;
        scan_markers(&lw, &rw, &cx);
        uint8_t blocked = update_lockout(cx);
        corner_marker_update(lw, rw, blocked);

        robot_in_slow_zone = 0;
        sig_sz_set(0);
        indicators_set(LED_BLUE, 0); // LED_BLUE

        adjust_motor_speed(compute_PID());

        if (finish_marker_detected(lw, rw, blocked)) {
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
        uint8_t lw, rw, cx;
        scan_markers(&lw, &rw, &cx);
        uint8_t blocked = update_lockout(cx);
        corner_marker_update(lw, rw, blocked);

        robot_in_slow_zone = 1;        // forces SLOW_SPEED in adjust_motor_speed
        sig_sz_set(1);
        indicators_set(LED_BLUE, bump_read(0) || bump_read(1));

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

// TESTING ONLY BELOW

void LED_test(void) {
    // for (uint8_t i = 0; i < 3; i++) {
    //     indicators_set(i, 1); // Turn on LED
    //     _delay_ms(1000);
    //     indicators_set(i, 0); // Turn off LED
    // }
    indicators_set(LED_GREEN, 1);
    indicators_set(LED_RED, 1);
    
}

void test_drive(void) { // Simple open-loop forward movement for testing
              motor1_speed(150);
              motor2_speed(150);
              // test the speeds
          }

void bump_test(void) {
    uint8_t bump1 = bump_read(0);
    uint8_t bump2 = bump_read(1);
    if (bump1) {
        motor1_speed(150);
        indicators_set(LED_GREEN, 1);
    } else {
        motor1_speed(0);
        indicators_set(LED_GREEN, 0);
    }

    if (bump2) {
        motor2_speed(150);
        indicators_set(LED_RED, 1);
    } else {
        motor2_speed(0);
        indicators_set(LED_RED, 0);
    }
}
// TESTING ABOVE

int main(void) {
    indicators_init();
    mux_init();
    motor_init();
    encoder_init();
    sig_init();
    tcs34725_init();
    bump_init();
    init_timer();
//  
    //  indicators_all(0);
    //  motors_stop();
    //  robot_on_straight = 1;
    sei();
    test_drive();
     LED_test();

    while (1) {
        if (pid_run_flag) {
            pid_run_flag = 0;
            drive();
        }

         // LED_test();
        // bump_test();
        test_drive();
    }
}