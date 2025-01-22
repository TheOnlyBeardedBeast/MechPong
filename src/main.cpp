#include <Arduino.h>
#include <PaddleStepper.hpp>
#include <PongStepper.hpp>
#include <Paddle.hpp>

Paddle paddle0;
Paddle paddle1;

void setup() {
  paddle0.initializeStepper(2,3);
  paddle0.initializeEncoder(4,5);

  paddle1.initializeStepper(6,7);
  paddle1.initializeEncoder(8,9);

  Paddle::instances[0] = &paddle0;
  Paddle::instances[1] = &paddle1;

  Paddle::attachPaddles();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  delay(1000);
}

void setup1()
{
  delay(2000);
}

void loop() {
}

void loop1() {
  Paddle::run();
}
