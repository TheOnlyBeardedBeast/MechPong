#pragma once

class IAsyncStepper {
    public:
        virtual void runAsync();
        virtual void stepAsync(long step);
        virtual void updateDirectionAsync();
        virtual void moveToAsync(long absolute);
};