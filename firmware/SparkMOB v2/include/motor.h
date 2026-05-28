#pragma once
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "sensor_array.h"

// Stops warning if it's already defined
#ifndef F_CPU
#define F_CPU 16000000UL
#endif


#define MOTOR_H

void motor_init(void);
void motor1Speed(uint8_t speed);
void motor2Speed(uint8_t speed);


// Motor direction definitions
#define CW  1
#define CCW 0

void setupMotors(void);
void motor1Direction(int direction);
void motor2Direction(int direction);
void motor1Speed(uint8_t speed);
void motor2Speed(uint8_t speed);

// Finish-area stop settings
#define FINISH_SENSOR 13     // I'm assuming 13 cause there are 14 sensors, and I'm guessing 14 is the right-most one 
#define STOP_TICKS 200        // 100 Hz = 2 seconds


