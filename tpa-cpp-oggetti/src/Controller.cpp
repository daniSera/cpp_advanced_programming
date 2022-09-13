//
// Created by olli on 10/05/20.
//

#include "libsimcon/Controller.h"

Controller::Controller(Matrix &K) : K(K) {}

Controller::Controller(const Controller &other) : K(other.K), loopCount(other.loopCount) {}

Controller &Controller::operator=(const Controller &other)
{
    if (this != &other)
    {
        K = other.K;
        loopCount = other.loopCount;
    }
    return *this;
}

Vector Controller::control(const Vector &x)
{
    loopCount++;
    return K * x;
}