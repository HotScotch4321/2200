#pragma once

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "sensor.h"

// Motor direction definitions
#define CW  1
#define CCW 0

void setupMotors();
void motor1Direction(int direction);
void motor2Direction(int direction);
void motor1Speed(uint8_t speed);
void motor2Speed(uint8_t speed);
