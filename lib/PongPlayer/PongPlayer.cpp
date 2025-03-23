#include "PongPlayer.hpp"

PongPlayer::PongPlayer(Paddle *paddle, Switch *shootButtonR)
{
    this->paddle = paddle;
    this->shootButtonR = shootButtonR;
}