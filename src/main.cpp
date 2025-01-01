#include <Arduino.h>
#include <PaddleStepper.hpp>
#include <Paddle.hpp>

Paddle paddle0;
Paddle paddle1;

void setup() {
  // Serial.begin(9600);
  paddle0.initializeStepper(8,9);
  paddle0.initializeEncoder(10,11);

  paddle1.initializeStepper(50,51);
  paddle1.initializeEncoder(52,53);

  Paddle::instances[0] = &paddle0;
  Paddle::instances[1] = &paddle1;

  Paddle::attachPaddles();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
}

void loop() {
  Paddle::run();
  // Serial.print(paddle0._stepper->currentPosition());
  // Serial.print(",");
  // Serial.println(paddle0._stepper->targetPosition());
}
