#include <Arduino.h>
#include <Pong.hpp>
#include <TrigTables.hpp>

Pong *game;

volatile bool initialized = false;

void setup() {
  initTrigTables();

  sleep_ms(1000);

  game = new Pong();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  initialized = true;
  sleep_ms(2000);
}

void setup1()
{ 
  while (!initialized)
  {
  }
  
  sleep_ms(2000);
}

void loop() {
  game->run();
  sleep_us(1);
}

void loop1() {
  if(game->gameState != GameState::CALIBRATION)
  {
    Paddle::update();
    // Paddle::run();
    // game->ball->run();
  }
  sleep_us(1);
}
