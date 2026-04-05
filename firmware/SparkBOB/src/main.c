#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "sensor.h"
#include "motor.h"
#include "timer.h" 
#include "pid.h"   

typedef enum {
    STATE_STRAIGHT,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_LOST
} RobotState;

void setup() {
    init_ADC();
    init_LEDS();
    setupMotors();
    init_timer(); 
}

void loop() 
{
    if (pid_run_flag) {
        pid_run_flag = 0; // Clear it so we wait for the next tick
        int16_t pid_output = compute_PID();
        adjust_motor_speed(pid_output);
    }
}

// Simple open-loop forward movement for testing
void test_loop() {
    motor1Speed(150);
    motor2Speed(150);
    // test the speeds 
}

int main(void)
{
    setup();
    while (1) {
        loop(); // Use this for line following
        // test_loop(); // Use this for open loop driving tests
    }
}