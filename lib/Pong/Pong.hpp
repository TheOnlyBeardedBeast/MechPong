#pragma once

#include <ScoreBoard.hpp>
#include <GameState.hpp>
#include <Paddle.hpp>
#include <Player.hpp>
#include <Ball.hpp>

class Pong
{
public:
    GameState gameState = GameState::CALIBRATION;
    void init(Ball *ball, Paddle *paddle1, Paddle *paddle2);
    void calibrate();
    void initMatch();
    void initMatchProgress();
    void initMatchDone();
    void serveMatch();
    void runMatch();
    void endMatch();
    void serveProgress();
    void bounceProgess();
    void center();
    void centerProgress();

private:
    Paddle *paddles[2] = {nullptr, nullptr};
    Ball *ball = nullptr;
    Player lastWinner = Player::NOONE;
    Player shooter = Player::NOONE;
    ScoreBoard score;
};