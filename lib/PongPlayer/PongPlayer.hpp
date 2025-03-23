#pragma once

#include <Paddle.hpp>
#include <Switch.hpp>

class PongPlayer
{
public:
    Paddle *paddle;
    Switch *shootButtonR;

    PongPlayer(Paddle *paddle, Switch *shootButtonR);
};