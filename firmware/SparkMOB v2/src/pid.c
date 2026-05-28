#include <stdbool.h>
#include "pid.h"
#include "mux.h"
#include "encoder.h"
#include "motor.h"

// mux channel 0-13
static const int16_t sensor_pos[14] = { -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7 };

// State variables for line PID
int16_t previous_error = 0;
int32_t integral_sum = 0;


// Define speeds (target encoder ticks per PID tick not raw PWM, will need re-tuning)
#define STRAIGHT_ERROR_LIMIT 40
#define STRAIGHT_COUNT_LIMIT 20
#define CORNER_SPEED 120
#define STRAIGHT_SPEED 180

// Speed PID — one per wheel, drives encoder speed to match target
#define SPEED_KP 2.0f
#define SPEED_KI 0.5f
#define SPEED_KD 0.0f
#define SPEED_INTEGRAL_MAX 500

extern bool slow_zone_active;

int32_t speed_integral_1 = 0;
int32_t speed_integral_2 = 0;
int32_t speed_prev_err_1 = 0;
int32_t speed_prev_err_2 = 0;

volatile uint8_t pid_run_flag = 0;
uint8_t robot_on_straight = 0;
uint8_t straight_count = 0;
uint8_t line_lost = 0;

int32_t get_position(void)
{
    int32_t weighted_value = 0;
    uint32_t sum_value = 0;

    for (int i = 0; i < 14; i++) {
        uint16_t s = mux_read(i);
        weighted_value += (int32_t)s * sensor_pos[i];
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

// If the line is lost, stop the motors and clear the straight-line state
    if (current_position == 100) {
        robot_on_straight = 0;
        straight_count = 0;
        line_lost = 1;
        // Clear speed PID state so wheels don't lurch when line is reacquired
        speed_integral_1 = 0;
        speed_integral_2 = 0;
        speed_prev_err_1 = 0;
        speed_prev_err_2 = 0;
        motor1Speed(0);
        motor2Speed(0);
        return 0;
    }
    line_lost = 0;

// Counts how long the robot has stayed centred on the line
    if (current_position > -STRAIGHT_ERROR_LIMIT && current_position < STRAIGHT_ERROR_LIMIT) {
        if (straight_count < STRAIGHT_COUNT_LIMIT) {
        straight_count++;
        }
    } else {
        straight_count = 0;
    }

// Marks the track as straight once the robot has stayed centred for long enough.
    if (straight_count >= STRAIGHT_COUNT_LIMIT) {
        robot_on_straight = 1;
    } else {
        robot_on_straight = 0;
    }


    int16_t error = - (int16_t)current_position; // 1. Calculate Error (Center is 0) --- I removed the 0 - Kyle

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

static uint8_t speed_pid_m1(int16_t target_ticks, int32_t actual_ticks)
{
    int32_t error = (int32_t)target_ticks - actual_ticks;

    speed_integral_1 += error;
    if (speed_integral_1 >  SPEED_INTEGRAL_MAX) speed_integral_1 =  SPEED_INTEGRAL_MAX;
    if (speed_integral_1 < -SPEED_INTEGRAL_MAX) speed_integral_1 = -SPEED_INTEGRAL_MAX;

    int32_t deriv = error - speed_prev_err_1;
    speed_prev_err_1 = error;

    float out = SPEED_KP * error + SPEED_KI * speed_integral_1 + SPEED_KD * deriv;

    if (out < 0)   out = 0;
    if (out > 255) out = 255;
    return (uint8_t)out;
}

static uint8_t speed_pid_m2(int16_t target_ticks, int32_t actual_ticks)
{
    int32_t error = (int32_t)target_ticks - actual_ticks;

    speed_integral_2 += error;
    if (speed_integral_2 >  SPEED_INTEGRAL_MAX) speed_integral_2 =  SPEED_INTEGRAL_MAX;
    if (speed_integral_2 < -SPEED_INTEGRAL_MAX) speed_integral_2 = -SPEED_INTEGRAL_MAX;

    int32_t deriv = error - speed_prev_err_2;
    speed_prev_err_2 = error;

    float out = SPEED_KP * error + SPEED_KI * speed_integral_2 + SPEED_KD * deriv;

    if (out < 0)   out = 0;
    if (out > 255) out = 255;
    return (uint8_t)out;
}

void adjust_motor_speed(int16_t pid_output) {
    if (line_lost) {
        motor1Speed(0);
        motor2Speed(0);
        return;
    }

    int16_t base_speed;

// Sets base speed to defined speeds for corners & straights
    if (slow_zone_active) {
        base_speed = SLOW_SPEED;
    } else if (robot_on_straight) {
        base_speed = STRAIGHT_SPEED;
    } else {
        base_speed = CORNER_SPEED;
    }

    int16_t target_left  = base_speed - pid_output;
    int16_t target_right = base_speed + pid_output;

    if (target_left  < 0) target_left  = 0;
    if (target_right < 0) target_right = 0;

    encoder_speed_update(); // drive each wheel to its target via encoder feedback
    uint8_t pwm_left  = speed_pid_m1(target_left,  encoder1_speed());
    uint8_t pwm_right = speed_pid_m2(target_right, encoder2_speed());

    motor1Speed(pwm_left);
    motor2Speed(pwm_right);

}
