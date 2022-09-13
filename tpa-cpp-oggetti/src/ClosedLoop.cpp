//
// Created by olli on 11/05/20.
//

#include "ClosedLoop.h"
#include <iostream>

ClosedLoop::ClosedLoop(const ClosedLoop &other)
{
    *this = other;
}

ClosedLoop &ClosedLoop::operator=(const ClosedLoop &other)
{
    if (this != &other)
    {
        *s = *other.s;
        *c = *other.c;
    }
    return *this;
}

void ClosedLoop::simulationStep()
{
    auto *pt = new Vector(c->control(s->getX()));
    s->simulate(*pt);
    delete pt;
}

void ClosedLoop::completeSimulation(int nSteps)
{
    for (int i = 0; i < nSteps ; i++)
    {
        this->simulationStep();
    }
}

Vector ClosedLoop::getState() const
{
    Vector v = s->getX();
    return v;
}