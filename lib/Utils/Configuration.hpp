#pragma once

#include <Arduino.h>

// ATARI PONG ANALYSIS
// GAME SPEEDS
// 0-----0.3175m/s - 635step/s
// 1-----0.4978m/s - 995stes/s
// 2-----0.6782m/s - 1,357step/s
// 3-----0.8585m/s - 1,717step/s
// 4-----1.0389m/s - 2,078step/s
// 5-----1.2192m/s - 2,440step/s
// 6-----1.5875m/s - 3,175step/s

const uint16_t MAX_SPEED = 1750;
const uint16_t START_SPEED = 400;
const uint32_t START_SPEED_PW = 400*400;
const uint16_t END_SPEED = 400;
const uint16_t BALL_ACCELERATION = 10 * MAX_SPEED;
const uint16_t PADDLE_ACCELERATION = 8000;

const uint16_t GAMEPLAY_AREA_Y = 1100;
const uint16_t GAMEPLAY_AREA_X = 1180;

const uint16_t PADDLE_WIDTH = 150; // 7.5 cm
const uint16_t PADDLE_WIDTH_HALF = 75;
const uint16_t BALL_WIDTH = 70; // 3.5 cm, will change to 2.5cm
const uint16_t BALL_WIDTH_HALF = 35;

const uint16_t ENCODER_RESOLUTION = 600;
const uint8_t PADDLE_SENSITIVITY = 8;
const uint16_t PADDLE_LIMIT = 980;
const uint16_t PADDLE_CENTER = 490;

const uint8_t MIN_ANGLE_MUL = 9;
const uint8_t MIN_ANGLE = 45;
const uint8_t MAX_ANGLE = 135;
const uint8_t MAX_ANGLE_MUL = 27;

const uint32_t TICKS = 1000000;