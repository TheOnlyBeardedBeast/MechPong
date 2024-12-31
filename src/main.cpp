#include <Arduino.h>
#include <PaddleStepper.hpp>
#include <Paddle.hpp>

Paddle paddle0;
Paddle paddle1;

void setup() {
  Serial.begin(115200);

  paddle0.initializeStepper(4,5);
  paddle0.initializeEncoder(6,7);
  paddle0.attachPaddles();

  paddle1.initializeStepper(9,10);
  paddle1.initializeEncoder(11,12);
  paddle1.attachPaddles();

  Paddle::instances[0] = &paddle0;
  Paddle::instances[1] = &paddle1;

  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  Paddle::run();
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  Serial.println("I am the pico");
  
}
