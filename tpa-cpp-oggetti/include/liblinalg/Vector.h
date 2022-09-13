//
// Created by olli on 04/05/20.
//

#ifndef SECONDASSIGNMENT_VECTOR_H
#define SECONDASSIGNMENT_VECTOR_H


#include "MatrixBase.h"

class Matrix;

class Vector : public MatrixBase {

public:

    /**
     * Construct a Vector of the specified size
     * @param size
     * @param vertical True if the vector is a column vector, false otherwise
     */
    explicit Vector( const int &size, bool vertical = true);

    //Constructor
    Vector( double *data,const int &size, bool vertical = true);

    //CopyConstructor
    Vector(const Vector &other);

    //Copy Assignment
    Vector &operator=(const Vector &other);

    //Destructor
    ~Vector() {}
    // does nothing since implicitly is called base destructor

    [[nodiscard]] int getSize() const { return size; }

    /**
     * Convert this Matrix into its own transpose. This should be equivalent
     * to: a = a.transpose();
     */
    Vector &transposeInPlace();

    /**
     * Compute the transpose of the Matrix.
     */
    Vector transpose();

    void resize(const int &size, bool vertical = true);

    // Access using single index
    virtual double operator()(const int &i) const;

    // Modify using single index
    virtual double &operator()(const int &i);

    // Multiplication
    Vector operator*(const Matrix &other) const;

    Matrix operator*(const Vector &other) const;

    // Sum
    Vector operator+(const MatrixBase &other) const;

    // Sum and assign
    Vector &operator+=(const MatrixBase &other);

    // Subtraction
    Vector operator-(const MatrixBase &other) const;

    // Subtract and assign
    Vector &operator-=(const MatrixBase &other);
};

#endif //SECONDASSIGNMENT_VECTOR_H
