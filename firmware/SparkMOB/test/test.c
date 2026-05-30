/*
 * test.c  -  Logging/validation tests for the SparkMOB robot.
 *
 * Each test streams CSV rows to serial at 100 Hz, one per control tick.
 * Capture the output to a .csv file, then run plot_test_data.py on it.
 *
 * Serial transport is in serial_arduino.cpp (Arduino Serial over USB CDC).
 * Timestamps use the tick counter, not millis() -- motor_init() reconfigures
 * Timer0 for PWM so millis() runs at the wrong rate after that point.
 *
 * To run a test, call test_main() from your Arduino setup(), or replace
 * the normal main loop with it during a dedicated logging build.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <Arduino.h>

#include "pins.h"
#include "encoder.h"
#include "indicators.h"
#include "mux.h"
#include "tcs34725.h"
#include "switch.h"
#include "motor.h"
#include "pid.h"
#include "timer.h"

extern volatile uint8_t pid_run_flag;
extern uint8_t robot_on_straight;
extern uint8_t robot_in_slow_zone;

/* must match CORNER/STRAIGHT/SLOW_SPEED in pid.c */
#define LOG_CORNER_SPEED   130
#define LOG_STRAIGHT_SPEED 180
#define LOG_SLOW_SPEED      90

#define MARKER_CONFIRM 3

void serial_init(void);
void serial_str(const char *s);

static char g_line[48];

extern "C" void serial_init(void) {
    Serial.begin(115200);
    while (!Serial);   // wait for USB enumeration on Leonardo
}
 
extern "C" void serial_str(const char *s) {
    Serial.print(s);
}
 

static uint32_t wait_tick(uint32_t *ticks) {
    while (!pid_run_flag) { }
    pid_run_flag = 0;
    return ++(*ticks);
}

/*
 * test_speed_step
 * CSV: t_ms, target, actual_left, actual_right
 *
 * Three phases at corner/straight/slow setpoints, 0.6 s each.
 * pid_output is forced to zero so both wheels share the same target --
 * we're looking at the speed loop in isolation, not the line controller.
 * adjust_motor_speed() calls encoder_speed_update() internally so the
 * encoder reads immediately after are current for that tick.
 */
void test_speed_step(void) {
    const uint16_t phase_ticks = 60;
    uint32_t ticks = 0;

    motor1_direction(MOTOR_FORWARD);
    motor2_direction(MOTOR_FORWARD);
    serial_str("t_ms,target,actual_left,actual_right\n");

    for (uint8_t phase = 0; phase < 3; phase++) {
        int16_t target;
        if      (phase == 0) { robot_in_slow_zone = 0; robot_on_straight = 0; target = LOG_CORNER_SPEED;   }
        else if (phase == 1) { robot_in_slow_zone = 0; robot_on_straight = 1; target = LOG_STRAIGHT_SPEED; }
        else                 { robot_in_slow_zone = 1; robot_on_straight = 0; target = LOG_SLOW_SPEED;     }

        for (uint16_t i = 0; i < phase_ticks; i++) {
            uint32_t t = wait_tick(&ticks);
            adjust_motor_speed(0);
            snprintf(g_line, sizeof(g_line), "%lu,%d,%d,%d\n",
                     (unsigned long)(t * 10UL), target,
                     (int)encoder1_speed(), (int)encoder2_speed());
            serial_str(g_line);
        }
    }
    motors_stop();
}

/*
 * test_line_tune
 * CSV: t_ms, position, error, pid_out
 *
 * Place the robot with the line offset to one side, then run.
 * Logs position recovery for 3 s. Run once per gain set -- change Kp/Kd
 * in pid.c and rebuild between runs, then overlay the CSVs in the plotter.
 */
void test_line_tune(void) {
    const uint16_t run_ticks = 300;
    uint32_t ticks = 0;

    motor1_direction(MOTOR_FORWARD);
    motor2_direction(MOTOR_FORWARD);
    serial_str("t_ms,position,error,pid_out\n");

    for (uint16_t i = 0; i < run_ticks; i++) {
        uint32_t t  = wait_tick(&ticks);
        int32_t pos = get_position();
        int16_t out = compute_PID();
        adjust_motor_speed(out);
        snprintf(g_line, sizeof(g_line), "%lu,%ld,%d,%d\n",
                 (unsigned long)(t * 10UL), (long)pos, (int)(-pos), out);
        serial_str(g_line);
    }
    motors_stop();
}

/*
 * test_slow_zone
 * CSV: t_ms, state, target, actual_left, actual_right
 *   state 0 = following, 1 = slow zone
 *
 * Run the robot over a red marker, then a green marker. Logs wheel speeds
 * across the transition so response time can be measured in the plotter.
 * PWM is paused during TCS reads to match main.c behaviour -- the motor
 * switching noise otherwise corrupts I2C reads from the colour sensor.
 */
static uint8_t confirm_colour(TCSColour want) {
    static uint8_t count = 0;
    RGBCData d;
    motor2_pwm_pause();
    uint8_t ok = tcs34725_read(&d);
    motor2_pwm_resume();
    if (ok && tcs34725_classify(&d) == want) { if (count < MARKER_CONFIRM) count++; }
    else count = 0;
    return count >= MARKER_CONFIRM;
}

void test_slow_zone(void) {
    const uint16_t max_ticks = 400;
    uint32_t ticks = 0;
    uint8_t  logstate = 0;

    motor1_direction(MOTOR_FORWARD);
    motor2_direction(MOTOR_FORWARD);
    robot_in_slow_zone = 0;
    serial_str("t_ms,state,target,actual_left,actual_right\n");

    for (uint16_t i = 0; i < max_ticks; i++) {
        uint32_t t = wait_tick(&ticks);
        adjust_motor_speed(compute_PID());

        if (logstate == 0 && confirm_colour(TCS_COLOUR_RED))   { robot_in_slow_zone = 1; logstate = 1; }
        if (logstate == 1 && confirm_colour(TCS_COLOUR_GREEN)) { robot_in_slow_zone = 0; logstate = 0; }

        int16_t target = robot_in_slow_zone ? LOG_SLOW_SPEED
                       : robot_on_straight  ? LOG_STRAIGHT_SPEED
                                            : LOG_CORNER_SPEED;
        snprintf(g_line, sizeof(g_line), "%lu,%d,%d,%d,%d\n",
                 (unsigned long)(t * 10UL), logstate, target,
                 (int)encoder1_speed(), (int)encoder2_speed());
        serial_str(g_line);
    }
    motors_stop();
}

void test_main(void) {
    indicators_init();
    mux_init();
    motor_init();
    encoder_init();
    tcs34725_init();
    bump_init();
    init_timer();
    serial_init();
    indicators_all(0);
    motors_stop();
    robot_on_straight = 0;
    sei();

    _delay_ms(500);

    test_speed_step();
    // test_line_tune();
    // test_slow_zone();

    while (1) { motors_stop(); }
}