//
// Created by olli on 04/05/20.
//

#include <cassert>
#include <iostream>
#include "liblinalg/Matrix.h"
#include "liblinalg/Vector.h"


Matrix::Matrix( const int &rows, const int &cols) : MatrixBase(rows, cols) {}

Matrix::Matrix(double *data, const int &rows, const int &cols) : MatrixBase(data, rows, cols) {}

Matrix::Matrix(const Matrix &other) : MatrixBase(other) {}

Matrix &Matrix::operator=(const Matrix &other)
{
    this->assignmentOperator(other); // executes control for self assignment
    return *this;
}

Matrix &Matrix::transposeInPlace()
{
    auto *pt = new double[size];
    dyn++;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            pt[j * rows + i] = data[i * cols + j];
        }
    }
    int set = rows;
    rows = cols;
    cols = set;
    delete[] data;
    canc++;
    data = pt;
    return *this;
}

Matrix Matrix::transpose()
{
    Matrix now = *this;
    now.transposeInPlace();
    return now;
}

void Matrix::resize(const int &rows,const int &cols)
{
    assert(rows > 0 && cols > 0);

    this->rows = rows;
    this->cols = cols;
    size = rows*cols;
    initIndex = 0;
    delete[] data;
    canc++;
    data = new double[size];
    dyn++;
}

Matrix Matrix::operator*(const Matrix &other) const
{
    double *pt = this->matrixMultiplication(other.data, other.rows , other.cols);
    Matrix now(pt, rows, other.cols);
    return now;
}

Vector Matrix::operator*(const Vector &other) const
{
    double *pt = this->matrixMultiplication(other.getData(),other.getRows(),other.getCols());
    Vector now(pt, rows, true);
    return now;
}


Matrix Matrix::operator+(const MatrixBase &other) const
{
    double *pt = this->matrixAddition(other.getData(),rows,cols,true);
    Matrix now(pt,rows,cols);
    return now;
}

Matrix &Matrix::operator+=(const MatrixBase &other)
{
    double *pt = this->matrixAddition(other.getData(),rows,cols,true);
    delete[] data;
    canc++;
    data = pt;
    return *this;
}

Matrix Matrix::operator-(const MatrixBase &other) const
{

    double *pt = this->matrixAddition(other.getData(),rows,cols,false);
    Matrix now(pt,rows,cols);
    return now;
}

Matrix &Matrix::operator-=(const MatrixBase &other)
{
    double *pt = this->matrixAddition(other.getData(),rows,cols,false);
    delete[] data;
    canc++;
    data = pt;
    return *this;
    
}










