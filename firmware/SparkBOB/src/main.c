#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "sensor.h"

// put function declarations here:
int myFunction(int, int);

void setup() {
  init_ADC();
  init_LEDS();
  init_buttons();
}

void loop() {
  if (read_sensor(3)) 
  {
    LED_on(0, 255); // turn on LED0 at full brightness
  } else {
    PORTB &= ~(1 << PORTB0); // turn off LED0
  }

  if (read_sensor(4)) 
  {
    LED_on(1, 255); // turn on LED1 at full brightness
  } else {
    PORTB &= ~(1 << PORTB1); // turn off LED1
  }

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}