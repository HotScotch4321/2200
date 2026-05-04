#include <Arduino.h>

#define BAUD_RATE 115200
#define LED_PIN 13

static String inputBuffer = "";

void handleCommand(const String& cmd) {
    if (cmd == "ping") {
        Serial.println("pong");
    } else if (cmd == "led on") {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED on");
    } else if (cmd == "led off") {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED off");
    } else if (cmd == "status") {
        Serial.print("uptime_ms:");
        Serial.println(millis());
    } else {
        Serial.print("unknown:");
        Serial.println(cmd);
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial);
    pinMode(LED_PIN, OUTPUT);
    Serial.println("EGB220 MEGA32U4 ready");
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}
