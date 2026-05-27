#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "mux.h"
#include "pid.h"

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

// Finish-area stop settings
#define FINISH_SENSOR 14      // I'm assuming 14 cause there are 14 sensors, and I'm guessing 14 is the right-most one 
#define RED_LED 0             // Replace with the actual red LED number
#define STOP_TICKS 200        // @ 100 Hz = 2 seconds

RobotState current_state = STATE_FOLLOW_LINE;
uint16_t stop_counter = 0;


void setup(void) {
    init_ADC();
    init_LEDS();
    setupMotors();
    init_timer();
}

// Prevents cycling start/stop if sesnor is over a "Start - Finish" area marker
uint8_t finish_lockout = 0;

// Detects finish marker on right side of track
uint8_t finish_marker_detected(void)
{
    return read_sensor_binary(FINISH_SENSOR);
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

// Simple open-loop forward movement for testing
void test_loop(void) {
    motor1Speed(150);
    motor2Speed(150);
    // test the speeds
}

// Mirror each sensor to its corresponding LED (LED on = sensor sees line)
void sensor_test_loop(void) {
    // Disconnect Timer0 PWM from PB7 (LED4) and PD0 (LED5) — they share pins with motors
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0B1));

    for (uint8_t i = 0; i < 8; i++) {
        if (read_sensor_binary(i)) {
            LED_on(i);
        } else {
            LED_off(i);
        }
    }
}

int main(void)
{
    setup();
    while (1) {
        loop();            // Use this for line following
        //test_loop();       // Use this for open loop driving tests
        //sensor_test_loop(); // Use this to verify sensors with LEDs
    }
}
