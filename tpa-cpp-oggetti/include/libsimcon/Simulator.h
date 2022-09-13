//
// Created by olli on 10/05/20.
//

#ifndef SECONDASSIGNMENT_SIMULATOR_H
#define SECONDASSIGNMENT_SIMULATOR_H

#include "libsimcon/Simulable.h"
#include "libsimcon/Controllable.h"
#include "liblinalg/Vector.h"
#include "liblinalg/Matrix.h"

class Simulator : public Simulable {

    Matrix A;
    Matrix B;
    Vector x;

public:

    // Constructor
    Simulator( Matrix &A, Matrix &B, Vector &x0);

    // Copy Constructor
    Simulator(const Simulator &other);

    // Copy Assignment
    Simulator &operator=(const Simulator &other);

    // Destructor
    ~Simulator() {}
    // destructor of objects self called

    // Setters
    void setMatrixA(const Matrix &other) { A = other; };

    void setMatrixB(const Matrix &other) { B = other; };

    void setX(const Vector &other) { x = other; };

    // Getters
    [[nodiscard]] const Matrix &getMatrixA() const { return A; };

    [[nodiscard]] const Matrix &getMatrixB() const { return B; };

    /**
     * Simulate the discrete-time system
     *     x[n+1] = A*x[n] + B*u[n]
     */
    void simulate(Vector &u);

    /**
     * Get the system state
     * @return
     */
    [[nodiscard]] const Vector &getX() const;
};


#endif //SECONDASSIGNMENT_SIMULATOR_H