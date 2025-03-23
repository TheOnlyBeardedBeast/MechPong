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
constexpr uint16_t BALL_ACCELERATION = 20000;
const uint16_t PADDLE_ACCELERATION = 10000;
const uint16_t PADDLE_MAX_SPEED = 1200;
/// @brief returns PI/180
constexpr float DEG_RAD = M_PI / 180.0f;

const uint8_t MICRO_STEP = 1;

constexpr uint16_t GAMEPLAY_AREA_Y = 1100*MICRO_STEP;
constexpr uint16_t GAMEPLAY_AREA_X = 1300*MICRO_STEP;

const uint16_t PADDLE_WIDTH = 90;
const uint16_t PADDLE_WIDTH_HALF = 45;
const uint16_t BALL_WIDTH = 40;
const uint16_t BALL_WIDTH_HALF = 20;

constexpr uint16_t PADDLE_LIMIT = 1100*MICRO_STEP;
constexpr uint16_t PADDLE_CENTER = PADDLE_LIMIT/2;

const uint8_t MIN_ANGLE_MUL = 6;
const uint8_t MIN_ANGLE = 30;
const uint8_t MAX_ANGLE = 150;
const uint8_t MAX_ANGLE_MUL = 30;

const uint32_t TICKS = 1000000;