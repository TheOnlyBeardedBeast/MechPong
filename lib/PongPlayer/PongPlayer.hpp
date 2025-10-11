#pragma once

#include <Paddle.hpp>
#include <Switch.hpp>

class PongPlayer
{
public:
    Paddle *paddle;
    Switch *shootButtonR;
    Switch *shootButtonL;

    PongPlayer(Paddle *paddle, Switch *shootButtonR, Switch *shootButtonL);
};