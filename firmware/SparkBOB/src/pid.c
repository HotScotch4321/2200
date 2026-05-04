#include <avr/io.h>
#include "pid.h"
#include "sensor.h"
#include "motor.h"

// Position value at each sensor
int16_t p[8] = {-3, -2, -1, 0, 1, 2, 0, 3};

// State variables for PID
int16_t previous_error = 0;
int32_t integral_sum = 0;

int32_t get_position(void)
{
    int32_t weighted_value = 0;
    uint32_t sum_value = 0;
    
    for(int i = 0; i < 8; i++) {
        if (i == 6) continue; // sensor 6 broken
        uint16_t s = read_sensor(i);
        weighted_value += (int32_t)s * p[i];
        sum_value += s;
    }

    if (sum_value == 0) {
        return 100; // out of bound
    }

    return ((weighted_value * 100) / (int32_t)sum_value);
}

int16_t compute_PID(void)
{
    const float Kp = 1.0f;
    const float Ki = 0.0f;
    const float Kd = 0.5f;

    int32_t current_position = get_position();
    if (current_position == 100) {
        motor1Speed(0);
        motor2Speed(0);
        return 0;
    }

    int16_t error = 0 - (int16_t)current_position; // 1. Calculate Error (Center is 0)

    float P = Kp * error;

    integral_sum += error;

    int32_t max_integral = 1000;
    if (integral_sum > max_integral) integral_sum = max_integral;
    if (integral_sum < -max_integral) integral_sum = -max_integral;

    float I = Ki * integral_sum;

    int32_t raw_derivative = (error - previous_error);
    float D = Kd * raw_derivative;

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