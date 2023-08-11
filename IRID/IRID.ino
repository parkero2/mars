#include <Arduino.h>
#include <IRremote.hpp>

#define IR_RECEIVE_PIN 2

void setup() {
    Serial.begin(9600);
    Serial.println("IR Receiver Button Decode");
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
    if (IrReceiver.decode()) {
        Serial.println(IrReceiver.decodedIRData.command, HEX);
        IrReceiver.resume();
    }
}