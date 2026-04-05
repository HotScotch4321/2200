#include <avr/io.h>
#include "pid.h"
#include "sensor.h"
#include "motor.h"

// Position value at each sensor
uint16_t p[8] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000}; 

// State variables for PID
int16_t previous_error = 0;
int32_t integral_sum = 0;

// Calculates and returns the position of the line (0 to 7000)
// Returns -1 if the line is not detected
int32_t get_position(void)
{
    uint32_t weighted_value = 0;
    uint32_t sum_value = 0;
    
    for(int i = 0; i < 8; i++) {
        uint16_t s = read_sensor(i);
        weighted_value += (uint32_t)s * p[i];
        sum_value += s;
    }

    if (sum_value == 0) {
        return -1; /
    }

    // Returns a position from 0 to 7000
    return (weighted_value / sum_value); 
}

int16_t compute_PID(void)
{
    const float Kp = 0.5f;  // PID constants | test HERE 
    const float Ki = 0.5f;
    const float Kd = 0.5f;

    const float dt = 0.01f;    // Time step (dt)

    int32_t current_position = get_position();
    if (current_position == -1) {
        return 0; 
    }

    // 1. Calculate Error (Center is 3500)
    int16_t error = 3500 - (int16_t)current_position; 

    // 2. Proportional term
    float P = Kp * error;

    // 3. Integral term
    integral_sum += error;
    float I = Ki * (integral_sum * dt);

    // 4. Derivative term
    float derivative = ((float)(error - previous_error)) / dt;
    float D = Kd * derivative;

    previous_error = error;
    return (int16_t)(P + I + D); 
}

void adjust_motor_speed(int16_t pid_output) {
    int16_t base_speed = 150;
    int16_t left_speed = base_speed - pid_output;
    int16_t right_speed = base_speed + pid_output;

    if (left_speed < 0) left_speed = 0;
    if (left_speed > 255) left_speed = 255;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 255) right_speed = 255;

    motor1Speed((uint8_t)left_speed);
    motor2Speed((uint8_t)right_speed);

}