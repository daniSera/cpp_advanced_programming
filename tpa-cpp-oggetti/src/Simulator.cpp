//
// Created by olli on 10/05/20.
//

#include "libsimcon/Simulator.h"

Simulator::Simulator( Matrix &A, Matrix &B, Vector &x0): A(A), B(B), x(x0) {}

Simulator::Simulator(const Simulator &other) : A(other.A), B(other.B), x(other.x) {}

Simulator &Simulator::operator=(const Simulator &other)
{
    if(this != &other)
    {
        A = other.A;
        B = other.B;
        x = other.x;
    }
    return *this;
}

void Simulator::simulate(Vector &u)
{
    auto *BU = new Vector(B*u);
    auto *AX = new Vector(A*x);

    x = *AX += *BU;  // maybe here I abused the operator +=

    delete BU;
    delete AX;
}

const Vector &Simulator::getX() const
{
    return x;
}