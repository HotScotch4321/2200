#ifndef PID_H
#define PID_H

#include <stdint.h>

int32_t get_position(void);
int16_t compute_PID(void);
void adjust_motor_speed(int16_t pid_output);

#endif
