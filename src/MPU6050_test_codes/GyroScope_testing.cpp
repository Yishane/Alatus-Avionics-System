#include <Arduino.h>
#include <Wire.h>


void setup(){   

    Serial.begin(115200);
    Wire.begin();

}

void loop(){

    Serial.println("Hello World");
    delay(1000);

}