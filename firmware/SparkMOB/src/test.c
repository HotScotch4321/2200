#include <avr/io.h>
#include <util/delay.h>
#include "indicators.h"
#include "mux.h"
#include "pid.h"
#include "motor.h"
#include "switch.h"
#include "tcs34725.h"

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
    // Initialize hardware components
    led_init();
    mux_init();
    motor_init();
    timer_init();
    bump_init();
    tcs34725_init();

    // Initial state: all LEDs off, motors stopped
    led_all(0);
    motors_stop();
}

// Prevents cycling start/stop if sesnor is over a "Start - Finish" area marker
uint8_t finish_lockout = 0;

// Detects finish marker on right side of track
uint8_t finish_marker_detected(void)
{
    return read_sensor_binary(FINISH_SENSOR);
}

void loop(void) // test loop
{
    if (pid_run_flag) {
        pid_run_flag = 0; // Clear it so we wait for the next tick

    // Stops for 2 seconds in the "Start - Finish" area
    if (current_state == STATE_START_FINISH_STOP) {
        motor1Speed(0);
        motor2Speed(0);
        led_set(2, 0);
        led_set(RED_LED, 1); // Turn on red LED to indicate stop state

        stop_counter++;

        if (stop_counter >= STOP_TICKS) {
            stop_counter = 0;
            led_set(RED_LED, 0); // Turn off red LED
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

void test_drive(void) { // Simple open-loop forward movement for testing
    motor1Speed(150);
    motor2Speed(150);
    // test the speeds
}

// Mirror each sensor to its corresponding LED (LED on = sensor sees line)
void sensor_test_loop(void) {
    for (uint8_t i = 0; i < 14; i++) {
        uint8_t sensor_val = mux_read(i);
        uint8_t sensor_on = (sensor_val > 512 ? 1 : 0); 
        led_set(0, sensor_on); // flash and hold red led for status, if sense
    }
}

void LED_test(void) {
    for (uint8_t i = 0; i < 3; i++) {
        led_set(i, 1); // Turn on LED
        _delay_ms(100);
        led_set(i, 0); // Turn off LED
    }
}

void tcs34725_test(void) {
    RGBCData data;
    tcs34725_read(&data);
    TCSColour colour = tcs34725_classify(&data);
    switch (colour) {
        case TCS_COLOUR_RED:
            led_set(0, 1); // Turn on red LED
            led_set(1, 0); // Ensure green LED is off
            led_set(2, 0); // Ensure blue LED is off
            break;
        case TCS_COLOUR_GREEN:
            led_set(1, 1); // Turn on green LED
            led_set(0, 0); // Ensure red LED is off
            led_set(2, 0); // Ensure blue LED is off
            break;
        case TCS_COLOUR_WHITE:
            led_set(2, 1); // Turn on blue LED
            led_set(0, 0); // Ensure red LED is off
            led_set(1, 0); // Ensure green LED is off
            break;
        default:
            // Handle unknown colour
            led_set(0, 0); // Ensure red LED is off
            led_set(1, 0); // Ensure green LED is off
            led_set(2, 0); // Ensure blue LED is off
            break;
    }
}

void bump_test(void) {
    uint8_t bump1 = bump_read(0);
    uint8_t bump2 = bump_read(1);
    if (bump1) {
        led_set(0, 1); // Turn on red LED for bump 1
    } else {
        led_set(0, 0); // Turn off red LED
    }

    if (bump2) {
        led_set(1, 1); // Turn on green LED for bump 2
    } else {
        led_set(1, 0); // Turn off green LED
    }
}

int main(void)
{
    setup();
    while (1) {
        loop();            // Use this for line following
        //test_drive();      // Use this for open loop driving tests
        //sensor_test_loop(); // Use this to verify sensors with LEDs
    }
}
