#pragma once

#include <Arduino.h>

template<typename T>
int sign(T x) {
    return (x > 0) - (x < 0);
}