#pragma once
#include <stdint.h>

#define MOTOR_FORWARD  1
#define MOTOR_BACKWARD 0
 
void motor_init(void);
void motor1_direction(uint8_t dir);
void motor2_direction(uint8_t dir);
void motor1_speed(uint8_t speed);    // 0 = stop, 255 = full speed
void motor2_speed(uint8_t speed);
void motors_stop(void);
 