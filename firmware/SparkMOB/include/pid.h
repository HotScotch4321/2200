#pragma once
#include <stdint.h>

extern volatile uint8_t pid_run_flag;   // set by timer ISR each control tick
extern uint8_t robot_on_straight;
extern uint8_t robot_in_slow_zone;

int32_t get_position(void);
int16_t compute_PID(void);
void    adjust_motor_speed(int16_t pid_output);
