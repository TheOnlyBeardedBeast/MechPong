#include "Pong.hpp"

Pong::Pong()
{
    this->ball = new Ball();
    this->ball->init(18,19,21,20);

    Paddle *paddle0 = new Paddle();
    paddle0->initializeStepper(4,5);
    paddle0->initializeEncoder(0,1);

    PongPlayer *player0 = new PongPlayer(paddle0, new Switch(2));

    Paddle *paddle1 = new Paddle();
    paddle1->initializeStepper(10,11);
    paddle1->initializeEncoder(14,15);

    PongPlayer *player1 = new PongPlayer(paddle1, new Switch(16));

    Paddle::instances[0] = paddle0;
    Paddle::instances[1] = paddle1;

    this->players[0] = player0;
    this->players[1] = player1;
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

    this->ball->setposition(500, 500);

    this->gameState = GameState::CENTER_PROGRESS;
    return;
}

void Pong::centerProgress()
{
    
    if (Paddle::isRunning() || this->ball->isRunning())
    {   
        return;
    }

    // TODO: put the game into stand by mode so the game can be started by the ball shooting button
    // this->gameState = GameState::STAND_BY;
    this->gameState = GameState::MATCH_INIT;
    return;
}

void Pong::standBy()
{
    if(players[0]->shootButtonR->isClicked() || players[1]->shootButtonR->isClicked())
    {
        this->gameState = GameState::CENTER;
        digitalWrite(LED_BUILTIN,LOW);
        delay(1000);
        digitalWrite(LED_BUILTIN,HIGH);
        return;
    }
}

void Pong::run()
{
    switch (this->gameState)
    {
        case GameState::CALIBRATION:
            this->calibrate();
            return;
        case GameState::CENTER:
            this->center();
            return;
        case GameState::CENTER_PROGRESS:
            this->centerProgress();
            return;
        case GameState::MATCH_INIT:
            this->initMatch();
            return;
        case GameState::MATCH_INIT_PROGRESS:
            this->initMatchProgress();
            return;
        case GameState::MATCH_INIT_DONE:
            this->initMatchDone();
            return;
        case GameState::MATCH_SERVE:
            this->serveMatch();
            return;
        case GameState::SERVE_PROGRESS:
            this->serveProgress();
            return;
        case GameState::MATCH_RUN:
            this->runMatch();
            return;
        case GameState::BOUNCE_PROGRESS:
            this->bounceProgess();
            return;
        case GameState::MATCH_END:
            this->endMatch();
            return;
        case GameState::STAND_BY:
            this->standBy();
            return;
        default:
            return;
    }
}

void Pong::initMatch()
{
    if (this->lastWinner == Player::NOONE)
    {
        this->lastWinner = (Player)((time_us_32() & 1) ? 1 : 0);
        this->shooter = this->lastWinner;
    }

    if (this->shooter == Player::Player1)
    {
        this->ball->setposition(1000, (1000 >> 1) + players[this->shooter]->paddle->getCenterRelativePosition());
    }
    else
    {
        this->ball->setposition(0, (1000 >> 1) + players[this->shooter]->paddle->getCenterRelativePosition());
    }

    this->gameState = GameState::MATCH_INIT_PROGRESS;
}

void Pong::initMatchProgress()
{
    if (this->ball->isRunning())
    {
        return;
    }

    this->gameState = GameState::MATCH_INIT_DONE;
}

void Pong::initMatchDone()
{
    this->ball->resetSpeeds();

    players[this->shooter]->paddle->subscribe(ball->subscriber());

    Paddle::attachPaddles();

    this->gameState = GameState::MATCH_SERVE;
}

/// @brief Runs a serving step in the game, must be called in a loop
void Pong::serveMatch()
{
    if (players[this->shooter]->shootButtonR->isClicked())
    {
        Paddle::instances[(int)this->shooter]->unsubScribe();
        // delay(1);

        // float modifier = this->shooter == Player::Player1 ? 
        //     (float)map(Paddle::instances[(int)this->shooter]->getPosition(), 0, PADDLE_LIMIT, MIN_ANGLE_MUL, MAX_ANGLE_MUL) : 
        //     (float)map(Paddle::instances[(int)this->shooter]->getPosition(), 0, PADDLE_LIMIT, MAX_ANGLE_MUL, MIN_ANGLE_MUL);
        // float angle = ((modifier * 5.f) + (this->shooter == Player::Player1 ? 180.f : 0.0f));

        float modifier = this->shooter == Player::Player1 ?
            (float)map(this->players[this->shooter]->paddle->_stepper->speed(),-PADDLE_MAX_SPEED,PADDLE_MAX_SPEED,MIN_ANGLE_MUL,MAX_ANGLE_MUL):
            (float)map(this->players[this->shooter]->paddle->_stepper->speed(),-PADDLE_MAX_SPEED,PADDLE_MAX_SPEED,MAX_ANGLE_MUL,MIN_ANGLE_MUL);
        
            float angle = ((modifier * 5.f) + (this->shooter == Player::Player1 ? 180.f : 0.0f));

        this->ball->shootDeg(angle,false);
        this->gameState = GameState::SERVE_PROGRESS;
        return;
    }

    return;
}

void Pong::serveProgress()
{
    long x = this->ball->getX();

    if (x >= 1000 || x <= 0)
    {
        return;
    }
    
    Serial.println("Serve progress");
    Serial.println("SERVE X:");
    Serial.println(""+x);
    this->gameState = GameState::MATCH_RUN;
}

void Pong::runMatch()
{
    long x = this->ball->getX();

    // POINT OR PADDLE HIT
    if (x >= 1000 || x <= 0)
    {
        Serial.println("POINT OR PADDLE HIT");

        Serial.println("HIT X:");
        Serial.println(x);

        Player nextShooter = this->shooter == Player::Player1 ? Player::Player2 : Player::Player1;

        byte shot = this->players[(int)nextShooter]->paddle->canShoot(ball->getCenterRelativePosition());

        if (shot != 0)
        {
            if (nextShooter == Player::Player2)
            {
                shot = map(shot, MIN_ANGLE, MAX_ANGLE, MAX_ANGLE, MIN_ANGLE);
            }

            float angle = (shot + (nextShooter == Player::Player1 ? 180.f : 0.0f));
            this->ball->shootDeg(angle,false);
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

    long y = this->ball->getY();

    // BOUNCE
    if (y >= 1000 || y <= 0)
    {
        Serial.println("Bounce");

        Serial.println("BOUNCE Y:");
        Serial.println(y);

        digitalWrite(LED_BUILTIN,LOW);
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
    long y = this->ball->getY();

    if (y >= 1000 || y <= 0)
    {
        return;
    }

    Serial.println("Bounce progress");
    Serial.println("Bounce Y:");
    Serial.println(y);

    this->gameState = GameState::MATCH_RUN;
}