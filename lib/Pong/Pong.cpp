#include "Pong.hpp"

Pong::Pong()
{
    // GPIO 27 and 28 cant be used as input
    // The board freezes
    // Further information says 27 and 28 are reserved for external memory
    
    this->ball = new Ball();
    this->ball->init(18,19,21,20);
    this->ball->instance = this->ball;
    this->ball->_xStepper->setCurrentPosition(-BALL_WIDTH_HALF);
    this->ball->limitSwitchX = new Switch(17);
    this->ball->limitSwitchY = new Switch(22);

    Paddle *paddle0 = new Paddle();
    paddle0->initializeStepper(4,5);
    paddle0->initializeEncoder(1,0);
    paddle0->_stepper->setCurrentPosition(-BALL_WIDTH);
    paddle0->limitSwitch = new Switch(3);

    PongPlayer *player0 = new PongPlayer(paddle0, new Switch(2), new Switch(2)); // 28

    Paddle *paddle1 = new Paddle();
    paddle1->initializeStepper(10,11);
    paddle1->initializeEncoder(14,15);
    paddle1->_stepper->setCurrentPosition(-BALL_WIDTH);
    paddle1->limitSwitch = new Switch(12);

    // audio 22-21
    

    PongPlayer *player1 = new PongPlayer(paddle1, new Switch(13), new Switch(13)); //27

    Paddle::instances[0] = paddle0;
    Paddle::instances[1] = paddle1;

    this->players[0] = player0;
    this->players[1] = player1;
}

void Pong::calibrate()
{
    Paddle::calibrate();
    delay(1000);

    Paddle::centerAll();

    while(Paddle::isRunning())
    {
        Paddle::run();
        sleep_us(1);
    }

    delay(1000);

    this->ball->calibrate();
    delay(1000);

    this->ball->center();

    if(this->ball->isRunning())
    {
        this->ball->run();
        sleep_us(1);
    }

    delay(1000);

    this->calibrated = true;
    this->gameState = GameState::CENTER;
}

void Pong::center()
{
    score.resetScore();
    Paddle::instances[0]->_stepper->moveTo(PADDLE_CENTER);
    Paddle::instances[1]->_stepper->moveTo(PADDLE_CENTER);

    this->ball->center();

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
    int clicker = -1;

    uint32_t clickedTime = time_us_32();

    if (players[0]->shootButtonR->isClicked()) {
        clicker = 0;
    } 
    else if (players[1]->shootButtonR->isClicked()) {
        clicker = 1;
    }

    if (clicker == -1) {
        return;
    }

    sleep_ms(10);

    while (players[clicker]->shootButtonR->isClicked()) {
        sleep_ms(10); 
    }

    uint32_t heldTime = (time_us_32() - clickedTime) / 1000;

    if (heldTime >= 1000) {
        this->gameState = GameState::CALIBRATION;
    } 
    else 
    {
        this->ball->setCurrentPosition(GAMEPLAY_AREA_X>>1,GAMEPLAY_AREA_Y>>1);
        Paddle::instances[0]->_stepper->setCurrentPosition(PADDLE_CENTER);
        Paddle::instances[1]->_stepper->setCurrentPosition(PADDLE_CENTER);

        this->gameState = GameState::CENTER;
    }

    digitalWrite(LED_BUILTIN, LOW);
    sleep_ms(1000);
    digitalWrite(LED_BUILTIN, HIGH); 
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
        case GameState::CENTER_STANDBY:
            this->centerStandBy();
            return;
        case GameState::CENTER_STANDBY_PROGRESS:
            this->centerStandByProgress();
            return;
        case GameState::ALIGN_BALL:
            this->alignBall();
            return;
        case GameState::ALIGN_BALL_PROGRESS:
            this->alignBallProgress();
            return;
        case GameState::ALIGN_BALL_END:
            this->alignBallEnd();
            return;
        case GameState::ALIGN_BALL_END_PROGRESS:
            this->alignBallEndProgress();
            return;
        default:
            return;
    }
}

void Pong::centerStandBy()
{
    score.resetScore();
    Paddle::instances[0]->_stepper->moveTo(PADDLE_CENTER);
    Paddle::instances[1]->_stepper->moveTo(PADDLE_CENTER);

    this->ball->center();

    this->gameState = GameState::CENTER_STANDBY_PROGRESS;
    return;
}

void Pong::centerStandByProgress()
{
    if (Paddle::isRunning() || this->ball->isRunning())
    {   
        return;
    }

    this->gameState = GameState::STAND_BY;
    return;
}

void Pong::alignBall()
{
    Paddle::resetAcceleration();
    Ball::instance->setposition(this->shooter==0 ? GAMEPLAY_AREA_X - BALL_OFFSET : BALL_OFFSET, Ball::instance->getY());
    this->gameState = GameState::ALIGN_BALL_PROGRESS;
    return;
}

void Pong::alignBallProgress()
{
    while(Ball::instance->isRunning())
    {
        sleep_us(1);
        return;
    }

    this->gameState = GameState::MATCH_INIT;
    return;
}

void Pong::alignBallEnd()
{
    Paddle::resetAcceleration();
    Ball::instance->setposition(this->shooter==0 ? GAMEPLAY_AREA_X - BALL_OFFSET : BALL_OFFSET, Ball::instance->getY());
    this->gameState = GameState::ALIGN_BALL_END_PROGRESS;
    return;
}

void Pong::alignBallEndProgress()
{
    while(Ball::instance->isRunning())
    {
        sleep_us(1);
        return;
    }

    this->shooter = Player::NOONE;
    this->gameState = GameState::CENTER_STANDBY;
    return;
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
        // this->ball->setposition(GAMEPLAY_AREA_X, (GAMEPLAY_AREA_Y >> 1) + players[this->shooter]->paddle->getCenterRelativePosition());
        this->ball->setposition(GAMEPLAY_AREA_X - BALL_OFFSET, (GAMEPLAY_AREA_Y >> 1) + players[this->shooter]->paddle->getCenterRelativePosition());
    }
    else
    {
        // this->ball->setposition(0, (GAMEPLAY_AREA_Y >> 1) + players[this->shooter]->paddle->getCenterRelativePosition());
        this->ball->setposition(BALL_OFFSET, (GAMEPLAY_AREA_Y >> 1) + players[this->shooter]->paddle->getCenterRelativePosition());
    }

    // Serial.println("INIT_MATCH:DONE");
    // Serial.println(this->ball->isRunning());
    this->gameState = GameState::MATCH_INIT_PROGRESS;
}

void Pong::initMatchProgress()
{
    if (this->ball->isRunning())
    {
        return;
    }

    // Serial.println("INIT_MATCH_PROGRESS:DONE");
    this->gameState = GameState::MATCH_INIT_DONE;
}

void Pong::initMatchDone()
{
    this->ball->resetSpeeds();

    players[this->shooter]->paddle->subscribe(ball->subscriber());

    Paddle::attachPaddles();

    // Serial.println("INIT_MATCH_DONE:DONE");
    this->gameState = GameState::MATCH_SERVE;
}

/// @brief Runs a serving step in the game, must be called in a loop
void Pong::serveMatch()
{
    if (players[this->shooter]->shootButtonR->isClicked())
    {
        Paddle::instances[(int)this->shooter]->unsubScribe();

        float modifier = this->shooter == Player::Player1 ?
            (float)map(this->players[this->shooter]->paddle->_stepper->speed(),-PADDLE_MAX_SPEED,PADDLE_MAX_SPEED,MIN_ANGLE_MUL,MAX_ANGLE_MUL):
            (float)map(this->players[this->shooter]->paddle->_stepper->speed(),-PADDLE_MAX_SPEED,PADDLE_MAX_SPEED,MAX_ANGLE_MUL,MIN_ANGLE_MUL);
        
            float angle = ((modifier * 5.f) + (this->shooter == Player::Player1 ? 180.f : 0.0f));

        this->ball->shootDeg(angle,false);
        // Serial.println("SERVE:DONE");
        this->gameState = GameState::SERVE_PROGRESS;
        return;
    }

    return;
}

void Pong::serveProgress()
{
    long x = this->ball->getX();

    if (x >= GAMEPLAY_AREA_X || x <= 0)
    {
        return;
    }
    
    // Serial.println("Serve progress");
    // Serial.println("SERVE X:");
    // Serial.println(""+x);
    // Serial.println("SERVER_PROGRESS:DONE");
    this->gameState = GameState::MATCH_RUN;
}

void Pong::runMatch()
{
    long x = this->ball->getX();

    // POINT OR PADDLE HIT
    if (
        (this->shooter == Player::Player2 && x >= GAMEPLAY_AREA_X - BALL_SHOOT_OFFSET - this->ball->stepsToStopX()) || 
        (this->shooter == Player::Player1 && x <= BALL_SHOOT_OFFSET + this->ball->stepsToStopX())
    )
    {
        // Serial.println("POINT OR PADDLE HIT");

        // Serial.println("HIT X:");
        // Serial.println(x);

        Player nextShooter = this->shooter == Player::Player1 ? Player::Player2 : Player::Player1;

        byte shot = this->players[(int)nextShooter]->paddle->tryShoot(
            ball->getCenterRelativePosition(), ball->getCenterRelativeTargetPosition(), ball->getPossibleHitOffset()
        );

        if (shot != 0)
        {
            if (nextShooter == Player::Player2)
            {
                shot = map(shot, MIN_ANGLE, MAX_ANGLE, MAX_ANGLE, MIN_ANGLE);
            }

            float angle = (shot + (nextShooter == Player::Player1 ? 180.f : 0.0f));
            this->ball->shootDeg(angle,false);
            this->shooter = nextShooter;
            // Serial.println("PADDLE_HIT:DONE");
            this->gameState = GameState::SERVE_PROGRESS;
            return;
        }

        score.incrementScore(this->shooter);

        Paddle::detachPaddles();
        Paddle::instances[0]->unsubScribe();
        Paddle::instances[1]->unsubScribe();

        Paddle::quickStop();

        this->ball->resetSpeeds();

        if (score.checkForWinner())
        {
            delay(2000);

            score.resetScore();

            this->shooter = nextShooter;
            this->lastWinner = Player::NOONE;
            // Serial.println("GAME_WIN:DONE");
            this->gameState = GameState::ALIGN_BALL_END;
            return;
        }

        

        // Serial.println("POINT_HIT:DONE");
        
        delay(1000);
        this->shooter = nextShooter;
        this->gameState = GameState::ALIGN_BALL;
        return;
    }

    long y = this->ball->getY();

    // BOUNCE
    if (
        (y >= GAMEPLAY_AREA_Y || y <= 0) 
        && (x > (BALL_WIDTH * 3) || this->shooter == Player::Player2) 
        && (x < GAMEPLAY_AREA_X - (BALL_WIDTH * 3) || this->shooter == Player::Player1)
    )
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
    long y = this->ball->getY();

    if (y >= GAMEPLAY_AREA_Y || y <= 0)
    {
        return;
    }

    // Serial.println("Bounce progress");
    // Serial.println("Bounce Y:");
    // Serial.println(y);

    this->gameState = GameState::MATCH_RUN;
}