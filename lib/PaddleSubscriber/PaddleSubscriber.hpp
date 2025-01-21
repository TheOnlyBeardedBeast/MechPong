#ifndef PaddleSubscriber_hpp
#define PaddleSubscriber_hpp

#include <StepDirection.hpp>

class PaddleSubscriber {
public:
    virtual void setDirection(bool direction);
    virtual void step(long step);
    virtual void clear();
};

#endif