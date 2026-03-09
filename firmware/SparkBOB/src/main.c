#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "sensor.h"

void setup() {
    init_ADC();
    init_LEDS();
}

void loop() {
    // Loop iterates exactly 8 times (0 through 7)
    for (uint8_t i = 0; i < 8; i++) {
        
        // 'i' represents both the sensor index and the LED number
        if (read_sensor(i)) {
            LED_on(i, 20); 
        } else {
            LED_off(i);     
        }
    }
    button_led_control(); 
}

int main(void)
{
    setup();
    while (1) {
        loop();
    }
}