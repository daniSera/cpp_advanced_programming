//
// Created by olli on 04/05/20.
//

#include <cassert>
#include <ios>
#include <iomanip>
#include <string>
#include <sstream>
#include <iostream>
#include "liblinalg/MatrixBase.h"

int MatrixBase::dyn = 0;
int MatrixBase::canc = 0;

MatrixBase::MatrixBase(const int &rows,const int &cols) : rows(rows), cols(cols)
{
    assert(rows > 0 && cols >0);

    size = rows * cols;
    initIndex = 0;
    data = new double[size];
    dyn++;
}

MatrixBase::MatrixBase(double *data,const int &rows,const int &cols) : rows(rows), cols(cols)
{
    assert(rows > 0 && cols >0);

    size = rows * cols;
    initIndex = size;
    this->data = data;
}

void MatrixBase::assignmentOperator(const MatrixBase &other)
{
    if (this != &other) // address control
    {
        delete[] data; // must be same length, cannot overwrite
        canc++;
        data = new double[other.size];
        dyn++;
        size = other.size;
        rows = other.rows;
        cols = other.cols;
        initIndex = other.initIndex;

        for (int i = 0; i < size; i++)
        {
            data[i] = other.data[i];
        }
    }
}

double *MatrixBase::matrixAddition(const double B[],const int &rb,const int &cb, bool sub) const
{
    assert(rb == rows && cb == cols);

    auto *pt = new double[size];
    dyn++;
    int c = (sub) ? 1 : -1;
    for (int i = 0; i < size ; i++)
    {
        pt[i] = data[i] + c * B[i];
    }
    return pt;
}

double *MatrixBase::matrixMultiplication(const double *B,const int &rb,const int &cb) const
{
    assert(cols == rb && cb >0);

    auto *pt = new double[rows * cb];
    dyn++;
    for (int i = 0; i < rows * cb; i++)
    {
        pt[i] = 0;
    }
    for (int i = 0; i < rows ; i ++)
    {
        for (int j = 0; j < cb ; j++)
        {
            for (int k = 0; k < rb ; k++)
            {
                pt[i*cb + j] += data[i*rb + k] * B[k*cb + j];
            }
        }
    }
    return pt;
}

MatrixBase::MatrixBase(const MatrixBase &other)
{
   this->assignmentOperator(other);
}

double MatrixBase::operator()(const int &r,const int &c) const
{
    assert(r >= 0 && c >= 0);
    assert(r < rows && c < cols);

    return data[r*cols + c];
}

// example: if rows = 3 , cols = 2 , r = 1 , c = 0 this method will get che value in position
// 2, counting from 0

double &MatrixBase::operator()(const int &r,const int &c)
{
    assert(r >= 0 && c >= 0);
    assert(r < rows && c < cols);

    return data[r*cols + c];
}


void MatrixBase::setAllValuesAt(const double &value)
{
    assert(initIndex != size); // cannot add values to a full object

    for (int i = initIndex; i < size; i++) // it cannot excede the allocated value or overwrite
    {
        data[i] = value;
    }
    initIndex = size;
}

MatrixBase &MatrixBase::operator,(double val)
{
    return *this <<= val;
}

MatrixBase &MatrixBase::operator<<=(double val)
{
    assert(initIndex < size);
    data[initIndex] = val;
    initIndex++;
    return *this;
}

std::string MatrixBase::toString()
{
    std::stringstream stream;
    stream << std::fixed;
    for(int i = 0; i < size; i++)
    {
        stream << std::setprecision(2) << double(data[i]) << " ";
    }
    stream << std::endl;
    return stream.str();
}
















