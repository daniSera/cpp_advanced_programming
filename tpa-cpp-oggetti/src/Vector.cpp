//
// Created by olli on 04/05/20.
//

#include <cassert>
#include <iostream>
#include "liblinalg/Vector.h"
#include "liblinalg/Matrix.h"


Vector::Vector( const int &size, bool vertical) : MatrixBase((vertical) ? size : 1,(vertical) ? 1 : size) {}

Vector::Vector( double *data,const int &size, bool vertical)
        :MatrixBase(data, (vertical) ? size : 1,(vertical) ? 1 : size) {}

Vector::Vector(const Vector &other) : MatrixBase(other) {}

Vector &Vector::operator=(const Vector &other)
{
    this->assignmentOperator(other); // executes control for self assignment
    return *this;
}

Vector &Vector::transposeInPlace()
{
    int set = rows;
    rows = cols;
    cols = set;
    return *this;
}

Vector Vector::transpose()
{
    Vector now = *this;
    now.transposeInPlace();
    return now;
}

void Vector::resize(const int &size, bool vertical)
{
    assert(size > 0);

    delete[] data;
    canc++;
    data = new double[size];
    dyn++;
    this->size = size;
    if (vertical)
    {
        rows = size;
        cols = 1;
    }
    else
    {
        rows = 1;
        cols = size;
    }
}

double Vector::operator()(const int &i) const
{
    assert(i >= 0);

    return data[i];
}

double &Vector::operator()(const int &i)
{
    assert(i >= 0);

    return data[i];
}

Vector Vector::operator*(const Matrix &other) const
{
    double *pt = this->matrixMultiplication(other.getData(),other.getRows(),other.getCols());
    Vector now(pt, other.getCols(), false);
    return now;
}

Matrix Vector::operator*(const Vector &other) const
{
    double *pt = this->matrixMultiplication(other.data,other.rows,other.cols);
    Matrix now(pt, rows, other.cols);
    return now;
}

Vector Vector::operator+(const MatrixBase &other) const
{
    double *pt = this->matrixAddition(other.getData(),other.getRows(),other.getCols(),true);
    Vector now(pt, size,cols == 1);
    return now;
}

Vector &Vector::operator+=(const MatrixBase &other)
{
    double *pt = this->matrixAddition(other.getData(),other.getRows(),other.getCols(),true);
    delete[] data;
    canc++;
    data = pt;
    return *this;
}

Vector Vector::operator-(const MatrixBase &other) const
{
    double *pt = this->matrixAddition(other.getData(),other.getRows(),other.getCols(),false);
    Vector now(pt, size,cols == 1);
    return now;
}

Vector &Vector::operator-=(const MatrixBase &other)
{
    double *pt = this->matrixAddition(other.getData(),other.getRows(),other.getCols(),false);
    delete[] data;
    canc++;
    data = pt;
    return *this;
}
















