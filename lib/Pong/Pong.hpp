#pragma once

#include <ScoreBoard.hpp>
#include <GameState.hpp>
#include <Paddle.hpp>
#include <Player.hpp>
#include <Ball.hpp>
#include <PongPlayer.hpp>

class Pong
{
public:
    GameState gameState = GameState::STAND_BY;
    Ball *ball = NULL;
    Pong();
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
    void standBy();
    void run();
    void centerStandBy();
    void centerStandByProgress();
    void alignBall();
    void alignBallProgress();

private:
    // Paddle *paddles[2] = {nullptr, nullptr};
    PongPlayer *players[2] = {NULL,NULL};
    Player lastWinner = Player::NOONE;
    Player shooter = Player::NOONE;
    ScoreBoard score;
};