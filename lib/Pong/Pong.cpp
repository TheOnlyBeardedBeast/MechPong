#include "Pong.hpp"

#define PLAYER1_BUTTON 47
#define PLAYER2_BUTTON 46

void Pong::init(Ball *ball, Paddle *paddle1, Paddle *paddle2)
{
    this->ball = ball;
    this->paddles[0] = paddle1;
    this->paddles[1] = paddle2;
}

void Pong::calibrate()
{
    Paddle::calibrate();
    this->ball->calibrate();

    this->gameState = GameState::CENTER;
}

void Pong::center()
{
    score.resetScore();
    Paddle::instances[0]->_stepper->moveTo(PADDLE_CENTER);
    Paddle::instances[1]->_stepper->moveTo(PADDLE_CENTER);

    this->ball->setposition(this->ball->limits.x >> 1, this->ball->limits.y >> 1, 500);

    this->gameState = GameState::CENTER_PROGRESS;
}

void Pong::centerProgress()
{
    if (Paddle::isRunning())
    {
        Paddle::run();
        return;
    }

    if (this->ball->needsToMove())
    {
        return;
    }

    delay(500);
    // TODO: put the game into stand by mode so the game can be started by the ball shooting button
    // this->gameState = GameState::STAND_BY;
    this->gameState = GameState::MATCH_INIT;
}

void Pong::initMatch()
{
    if (this->lastWinner == Player::NOONE)
    {
        this->lastWinner = (Player)random(0, 2);
        this->shooter = this->lastWinner;
    }

    if (this->shooter == Player::Player1)
    {
        this->ball->setposition(this->ball->limits.x, (this->ball->limits.y >> 1) + paddles[this->shooter]->getCenterRelativePosition(), 500);
    }
    else
    {
        this->ball->setposition(0, (this->ball->limits.y >> 1) + paddles[this->shooter]->getCenterRelativePosition(), 500);
    }

    this->gameState = GameState::MATCH_INIT_PROGRESS;
}

void Pong::initMatchProgress()
{
    if (this->ball->needsToMove())
    {
        return;
    }

    this->gameState = GameState::MATCH_INIT_DONE;
}

void Pong::initMatchDone()
{

    paddles[this->shooter]->subscribe(ball->subscriber());

    Paddle::attachPaddles();

    this->gameState = GameState::MATCH_SERVE;
}

/// @brief Runs a serving step in the game, must be called in a loop
void Pong::serveMatch()
{
    size_t inputPin = this->shooter == 0 ? 47 : 46;

    if (!digitalRead(inputPin))
    {
        Paddle::instances[(int)this->shooter]->unsubScribe();
        delay(1);

        float modifier = this->shooter == Player::Player1 ? (float)map(Paddle::instances[(int)this->shooter]->getPosition(), 0, PADDLE_LIMIT, MIN_ANGLE_MUL, MAX_ANGLE_MUL) : (float)map(Paddle::instances[(int)this->shooter]->getPosition(), 0, PADDLE_LIMIT, MAX_ANGLE_MUL, MIN_ANGLE_MUL);
        float angle = ((modifier * 5.f) + (this->shooter == Player::Player1 ? 180.f : 0.0f));

        this->ball->shootDeg(angle);
        this->gameState = GameState::SERVE_PROGRESS;
        return;
    }

    return;
}

void Pong::serveProgress()
{
    Point ballPosition = this->ball->getPosition();
    Point ballLimits = this->ball->limits;

    if (ballLimits.x == ballPosition.x || 0 == ballPosition.x)
    {
        return;
    }

    this->gameState = GameState::MATCH_RUN;
    return;
}

void Pong::runMatch()
{
    if (this->ball->needsToMove())
    {
        return;
    }

    Point ballPosition = this->ball->getPosition();
    Point ballLimits = this->ball->limits;

    // POINT OR PADDLE HIT
    if (ballLimits.x == ballPosition.x || 0 == ballPosition.x)
    {

        Player nextShooter = this->shooter == Player::Player1 ? Player::Player2 : Player::Player1;

        byte shot = Paddle::instances[(int)nextShooter]->canShoot(ball->getCenterRelativePosition());

        if (shot != 0)
        {
            if (nextShooter == Player::Player2)
            {
                shot = map(shot, MIN_ANGLE, MAX_ANGLE, MAX_ANGLE, MIN_ANGLE);
            }

            float angle = (shot + (nextShooter == Player::Player1 ? 180.f : 0.0f));
            this->ball->shootDeg(angle);
            this->shooter = nextShooter;
            this->gameState = GameState::SERVE_PROGRESS;
            return;
        }

        score.incrementScore(this->shooter);

        Paddle::detachPaddles();
        Paddle::instances[0]->unsubScribe();
        Paddle::instances[1]->unsubScribe();

        if (score.checkForWinner())
        {
            delay(2000);

            score.resetScore();

            this->shooter = Player::NOONE;
            this->lastWinner = Player::NOONE;
            this->gameState = GameState::CENTER;
            return;
        }

        this->shooter = nextShooter;
        this->gameState = GameState::MATCH_INIT;
        return;
    }

    // BOUNCE
    if ((ballLimits.y == ballPosition.y || 0 == ballPosition.y) && ballPosition.x != 0 && ballPosition.x != ballLimits.x && !Ball::instance->needsToMove())
    {
        ball->bounce();
        this->gameState = GameState::BOUNCE_PROGRESS;
        return;
    }

    return;
}

void Pong::endMatch()
{
    delay(1000);
    this->ball->center();

    this->gameState = GameState::CENTER_PROGRESS;
}

void Pong::bounceProgess()
{
    Point ballPosition = this->ball->getPosition();
    Point ballLimits = this->ball->limits;

    if (ballLimits.y == ballPosition.y || 0 == ballPosition.y)
    {
        return;
    }

    this->gameState = GameState::MATCH_RUN;
}