#include <Arduino.h>
#include <PaddleStepper.hpp>
#include <PongStepper.hpp>
#include <Paddle.hpp>

Paddle paddle0;
Paddle paddle1;
PongStepper y(21,20);
PongStepper x(18,19);


void setup() {
  paddle0.initializeStepper(4,5);
  paddle0.initializeEncoder(0,1);

  paddle1.initializeStepper(10,11);
  paddle1.initializeEncoder(14,15);

  Paddle::instances[0] = &paddle0;
  Paddle::instances[1] = &paddle1;
  x.setMaxSpeed(1600);
  y.setMaxSpeed(1600);
  x.setAcceleration(25000);
  y.setAcceleration(25000);

  // y.setMaxSpeed(1200);
  // y.setAcceleration(9600);

  Paddle::attachPaddles();
  // paddle0.subscribe(&y);

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  // paddle0._stepper->moveTo(200);

  delay(1000);
}

void setup1()
{ 
  delay(2000);
}

void loop() {
  // if(paddle0._stepper->isRunning()){
  //   digitalWrite(LED_BUILTIN,HIGH);
  // } else {
  //   digitalWrite(LED_BUILTIN,LOW);
  // }
  delay(5000);
  // paddle0._stepper->moveTo(200);
  // paddle1._stepper->moveTo(200);
  y.moveTo(200);
  x.moveTo(200);
  delay(5000);
  // paddle0._stepper->moveTo(0);
  // paddle1._stepper->moveTo(0);
  y.moveTo(0);
  x.moveTo(0);

}

void loop1() {
  Paddle::update();
  Paddle::run();
  y.run();
  x.run();
}
