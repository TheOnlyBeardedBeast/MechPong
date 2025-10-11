#include "PongPlayer.hpp"

PongPlayer::PongPlayer(Paddle *paddle, Switch *shootButtonR,  Switch *shootButtonL)
{
    this->paddle = paddle;
    this->shootButtonR = shootButtonR;
    this->shootButtonR = shootButtonL;
}