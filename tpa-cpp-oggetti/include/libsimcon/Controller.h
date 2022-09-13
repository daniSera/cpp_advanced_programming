//
// Created by olli on 10/05/20.
//

#ifndef SECONDASSIGNMENT_CONTROLLER_H
#define SECONDASSIGNMENT_CONTROLLER_H

#include "libsimcon/Controllable.h"
#include "liblinalg/Vector.h"
#include "liblinalg/Matrix.h"

class Controller : public Controllable {

    int loopCount = 0;
    Matrix K;

public:

    // Constructor
    explicit Controller( Matrix &K);

    // Copy Constructor
    Controller(const Controller &other);

    //Copy Assignment
    Controller &operator=(const Controller &other);

    // Destructor
    ~Controller() {}
    //objects self handle destructions

    // Setter
    void setMatrix( Matrix &other) {K = other;};

    // Getter
    const Matrix &getMatrix() {return K;};

    Vector control(const Vector &x);

    [[nodiscard]] int getLoopCounter() const { return loopCount; };
};

#endif //SECONDASSIGNMENT_CONTROLLER_H