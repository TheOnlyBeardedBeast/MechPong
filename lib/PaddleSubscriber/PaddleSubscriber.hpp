#ifndef PaddleSubscriber_hpp
#define PaddleSubscriber_hpp

#include <StepDirection.hpp>

class PaddleSubscriber {
public:
    virtual void externalUpdateDirection(bool direction);
    virtual void externalStep(long step);
    virtual void clear();
};

#endif