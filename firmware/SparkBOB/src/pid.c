#include <avr/io.h>
#include "pid.h"
#include "sensor.h"
#include "motor.h"

// Position value at each sensor
int16_t p[8] = {-8, -4, -2, -1, 1, 2, 4, 8}; 

// State variables for PID
int16_t previous_error = 0;
int32_t integral_sum = 0;
// Calculates and returns the position of the line (-8 to 8)
int32_t get_position(void)
{
    int32_t weighted_value = 0;
    uint32_t sum_value = 0;
    
    for(int i = 0; i < 8; i++) {
        uint16_t s = read_sensor(i);
        weighted_value += (int32_t)s * p[i];
        sum_value += s;
    }

    if (sum_value == 0) {
        return 100; // out of bound
    }

    // Returns a position from -8 to 8
    return (weighted_value / (int32_t)sum_value); 
}

int16_t compute_PID(void)
{
    const int32_t Kp = 1; 
    const int32_t Ki = 0.5;  
    const int32_t Kd = 1; 

    int32_t current_position = get_position();
    if (current_position == 100) {
        return 0; 
    }

    int16_t error = 0 - (int16_t)current_position; // 1. Calculate Error (Center is 0)

    int32_t P = Kp * error; 
    
    integral_sum += error;

    int32_t max_integral = 1000; 
    if (integral_sum > max_integral) integral_sum = max_integral;
    if (integral_sum < -max_integral) integral_sum = -max_integral;

    int32_t I = Ki * integral_sum;

    int32_t raw_derivative = (error - previous_error);
    int32_t D = Kd * raw_derivative;

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