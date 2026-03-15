#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "sensor.h"
#include "motor.h"

// Define states for the line follower
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

}

void loop() {
    // Read sensor states
    bool s[8];
    for(int i = 0; i < 8; i++) {
        s[i] = read_sensor(i);
    }
    
    // Group sensors for easier logic
    bool left_sensors = s[1] || s[2];
    bool center_sensors = s[3] || s[4];
    bool right_sensors = s[5] || s[6];

    // Determine current state based on sensors
    RobotState current_state = STATE_LOST; // Default to lost

    if (center_sensors) {
        current_state = STATE_STRAIGHT;
    } else if (left_sensors) {
        current_state = STATE_TURN_LEFT;
    } else if (right_sensors) {
        current_state = STATE_TURN_RIGHT;
    }
    
    // Execute state behavior
    switch (current_state) {
        case STATE_STRAIGHT:
            motor1Speed(150); // 0 - 255
            motor2Speed(150);
            break;

        case STATE_TURN_LEFT:
            motor1Speed(100);
            motor2Speed(150);
            break;

        case STATE_TURN_RIGHT:
            motor1Speed(150);
            motor2Speed(100);
            break;

        case STATE_LOST:
        default:
            // Stop motors
            motor1Speed(0);
            motor2Speed(0);
            break;
    }
}

// Simple open-loop forward movement for testing
void test_loop() {
    motor1Speed(150);
    motor2Speed(150);
    // test the speeds 
}

// TODO: If S7 and S8 or S1 and S2, sees the line, slow down speed

int main(void)
{
    setup();
    while (1) {
        // loop(); // Use this for line following
        
        test_loop(); // Use this for open loop driving tests
    }
}