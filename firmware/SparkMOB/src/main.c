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

#define LEFT_SENSOR      0
#define LMID_SENSOR      4
#define RMID_SENSOR      9
#define FINISH_SENSOR    13
#define SENSOR_THRESHOLD 512
#define CROSSOVER_WHITE  3

#define STOP_TICKS        200
#define MARKER_CONFIRM    3
#define MARKER_LOCKOUT    20
#define ROUTE_BIAS_TICKS  60
#define ROUTE_BIAS_MAG    25

extern uint8_t robot_in_slow_zone;

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
static uint8_t  departed       = 0;  // set once sensor 13 goes dark after a stop

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

// Both motors coast while the TCS34725 reads over I2C (SCL is PD0 = OC0B).
// Motor 1 is paused too because PWM switching injects noise on the shared rails.
static inline void motors_pwm_pause(void) {
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));  // release PB7 from OC0A
    TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));  // release PD0 from OC0B
    PORTB  &= ~(1 << PB7);
    PORTD  &= ~(1 << PD0);
}

static inline void motors_pwm_resume(void) {
    TCCR0A |= (1 << COM0A1);  // reconnect OC0A
    TCCR0A |= (1 << COM0B1);  // reconnect OC0B
}

static void scan_markers(uint8_t *left_white, uint8_t *right_white,
                         uint8_t *crossover) {
    uint8_t l  = mux_read(LEFT_SENSOR)   > SENSOR_THRESHOLD;
    uint8_t lm = mux_read(LMID_SENSOR)   > SENSOR_THRESHOLD;
    uint8_t rm = mux_read(RMID_SENSOR)   > SENSOR_THRESHOLD;
    uint8_t r  = mux_read(FINISH_SENSOR) > SENSOR_THRESHOLD;
    *left_white  = l;
    *right_white = r;
    *crossover   = (l + lm + rm + r) >= CROSSOVER_WHITE;
}

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

// departed must be set before this can fire, preventing a false trigger on
// the start line at power-on or immediately after a stop.
static uint8_t finish_marker_detected(uint8_t left_white, uint8_t right_white,
                                      uint8_t blocked) {
    if (!departed) {
        if (!right_white) departed = 1;  // robot has left the line, arm detection
        return 0;
    }
    uint8_t marker = !blocked && right_white && !left_white;
    if (marker) { if (finish_confirm < MARKER_CONFIRM) finish_confirm++; }
    else finish_confirm = 0;
    return finish_confirm >= MARKER_CONFIRM;
}

static uint8_t red_marker_detected(void) {
    RGBCData d;
    motors_pwm_pause();
    uint8_t valid = tcs34725_read(&d);
    motors_pwm_resume();
    if (valid && tcs34725_classify(&d) == TCS_COLOUR_RED) {
        if (red_confirm < MARKER_CONFIRM) red_confirm++;
    } else red_confirm = 0;
    return red_confirm >= MARKER_CONFIRM;
}

static uint8_t green_marker_detected(void) {
    RGBCData d;
    motors_pwm_pause();
    uint8_t valid = tcs34725_read(&d);
    motors_pwm_resume();
    if (valid && tcs34725_classify(&d) == TCS_COLOUR_GREEN) {
        if (green_confirm < MARKER_CONFIRM) green_confirm++;
    } else green_confirm = 0;
    return green_confirm >= MARKER_CONFIRM;
}

static uint8_t update_lockout(uint8_t crossover) {
    if (crossover) marker_lockout = MARKER_LOCKOUT;
    else if (marker_lockout) marker_lockout--;
    return crossover || marker_lockout > 0;
}

void drive(void) {
    switch (state) {

    case STATE_START_FINISH_STOP:
        motors_stop();
        indicators_set(LED_RED,   1);
        indicators_set(LED_GREEN, 0);
        indicators_set(LED_BLUE,  0);
        if (++stop_counter >= STOP_TICKS) {
            stop_counter = 0;
            departed     = 0;       // re-arm: must leave the line before next finish
            indicators_set(LED_RED, 0);
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
        indicators_set(LED_BLUE, 0);

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

        robot_in_slow_zone = 1;
        sig_sz_set(1);
        indicators_set(LED_BLUE, bump_read(0) || bump_read(1));

        int16_t pid_output = compute_PID();

        if (route_bias_ctr > 0) {
            route_bias_ctr--;
            pid_output += (lap_count % 2 == 0) ? -ROUTE_BIAS_MAG : ROUTE_BIAS_MAG;
        }
        adjust_motor_speed(pid_output);

        if (green_marker_detected()) {
            green_confirm      = 0;
            route_bias_ctr     = 0;
            robot_in_slow_zone = 0;
            sig_sz_set(0);
            state = STATE_FOLLOW_LINE;
        }
        break;
    }
    }
}

// --- test functions ---

void LED_test(void) {
    indicators_set(LED_GREEN, 1);
    indicators_set(LED_RED,   1);
}

void test_drive(void) {
    motor1_speed(150);
    motor2_speed(150);
}

void bump_test(void) {
    uint8_t b1 = bump_read(0);
    uint8_t b2 = bump_read(1);
    motor1_speed(b1 ? 150 : 0);
    indicators_set(LED_GREEN, b1);
    motor2_speed(b2 ? 150 : 0);
    indicators_set(LED_RED, b2);
}

// ---

int main(void) {
    indicators_init();
    mux_init();
    motor_init();
    encoder_init();
    sig_init();
    tcs34725_init();
    bump_init();
    init_timer();

    indicators_all(0);
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