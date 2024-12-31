#ifndef PaddleSubscriber_hpp
#define PaddleSubscriber_hpp

#include <StepDirection.hpp>

class PaddleSubscriber {
public:
    virtual void setDirection(StepDirection direction);
    virtual void stepSingle();
    virtual void stepLow();
};

#endif