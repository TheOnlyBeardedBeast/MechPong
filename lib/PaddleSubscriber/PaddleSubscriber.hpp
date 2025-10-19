#pragma once

class PaddleSubscriber {
public:
    virtual void externalUpdateDirection(bool direction);
    virtual void externalStep(long step);
    virtual void clear();
    virtual void moveTo(long relative);
    virtual void moveToAsync(long relative);
};