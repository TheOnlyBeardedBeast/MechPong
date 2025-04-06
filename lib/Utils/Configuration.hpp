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
const uint8_t MICRO_STEP = 1;

constexpr uint16_t MAX_SPEED = 1750 * MICRO_STEP;
constexpr uint16_t BALL_ACCELERATION = 20000 * MICRO_STEP;
constexpr uint16_t PADDLE_ACCELERATION = 10000 * MICRO_STEP;
constexpr uint16_t PADDLE_MAX_SPEED = 1200 * MICRO_STEP;
/// @brief returns PI/180
constexpr float DEG_RAD = M_PI / 180.0f;

constexpr uint16_t GAMEPLAY_AREA_Y = 1100 * MICRO_STEP;
constexpr uint16_t GAMEPLAY_AREA_X = 1400 * MICRO_STEP;

constexpr uint16_t PADDLE_WIDTH = 120 * MICRO_STEP;
constexpr uint16_t PADDLE_WIDTH_HALF = PADDLE_WIDTH >> 1;
constexpr uint16_t BALL_WIDTH = 40 * MICRO_STEP;
constexpr uint16_t BALL_WIDTH_HALF = BALL_WIDTH >> 1;
constexpr uint16_t BALL_OFFSET = BALL_WIDTH + BALL_WIDTH_HALF;
constexpr uint16_t BALL_SHOOT_OFFSET = BALL_WIDTH << 1;

constexpr uint16_t PADDLE_LIMIT = 1020 * MICRO_STEP;
constexpr uint16_t PADDLE_CENTER = PADDLE_LIMIT >> 1;

const uint8_t ANGLE_STEP = 5;
const uint8_t MIN_ANGLE_MUL = 6;
constexpr uint8_t MIN_ANGLE = MIN_ANGLE_MUL * ANGLE_STEP;
const uint8_t MAX_ANGLE_MUL = 30;
constexpr uint8_t MAX_ANGLE = MAX_ANGLE_MUL * ANGLE_STEP;
constexpr uint8_t ANGLE_COUNT = MAX_ANGLE_MUL + 1 - MIN_ANGLE_MUL;

const uint32_t TICKS = 1000000;