//
// Created by olli on 04/05/20.
//

#ifndef SECONDASSIGNMENT_MATRIX_H
#define SECONDASSIGNMENT_MATRIX_H

#include "MatrixBase.h"

class Vector;

class Matrix : public MatrixBase {
public:

    //Constructor
    Matrix( const int &rows, const int &cols);

    //Constructor
    Matrix(double *data, const int &rows, const int &cols);

    //Copy Constructor
    Matrix(const Matrix &other);

    //Copy Assignment Operator
    Matrix &operator=(const Matrix &other);

    //Destructor
    ~Matrix() {};
    // does nothing since implicitly is called the base destructor

    /**
     * Convert this Matrix into its own transpose. This should be equivalent
     * to: a = a.transpose();
     */
    Matrix &transposeInPlace();

    /**
     * Compute the transpose of the Matrix.
     */
    Matrix transpose();

    void resize(const int &rows,const int &cols);

    // Multiplication
    Matrix operator*(const Matrix &other) const;

    Vector operator*(const Vector &other) const;

    // Sum
    Matrix operator+(const MatrixBase &other) const;

    // Sum and assign
    Matrix &operator+=(const MatrixBase &other);

    // Subtraction
    Matrix operator-(const MatrixBase &other) const;

    // Subtract and assign
    Matrix &operator-=(const MatrixBase &other);
};


#endif //SECONDASSIGNMENT_MATRIX_H
