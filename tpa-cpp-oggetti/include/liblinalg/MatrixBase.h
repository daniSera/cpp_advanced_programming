//
// Created by olli on 04/05/20.
//

#ifndef SECONDASSIGNMENT_MATRIXBASE_H
#define SECONDASSIGNMENT_MATRIXBASE_H

#include <string>
#include <iostream>

class MatrixBase {
protected:

    static int dyn;           // counts dynamic allocation
    static int canc;          // counts dynamic deallocation

    double *data = nullptr;   // pointer to the data
    int size{};               // total size of the matrix
    int rows{};               // number of rows
    int cols{};               // number of columns

    int initIndex{};          // index to be used for the list initialization with operators <<= and ,

    // Constructor
    MatrixBase(const int &rows,const int &cols);

    // Constructor
    MatrixBase(double *data,const int &rows,const int &cols);

    // Operator assign
    void assignmentOperator(const MatrixBase &other);

    // Constructors and assignment are protected, they can be called only by derived classes

    /**
     * Sum (or subtract) to the given matrix
     * @param B Matrix to sub or subtract
     * @param rb number of rows
     * @param cb number of columns
     * @param sub If true then perform subtraction, if false then perform addition
     * @return
     */

    double *matrixAddition(const double B[],const int &rb,const int &cb, bool sub = false) const;

    double *matrixMultiplication(const double B[],const int &rb,const int &cb) const;

public:

    // Destructor
    ~MatrixBase()
    {
        delete[] data;
        canc++;
    }

    // Copy Constructor
    MatrixBase(const MatrixBase &other);
    // why is it not protected? I think it should be

    [[nodiscard]] int getCols() const { return cols; }

    [[nodiscard]] int getRows() const { return rows; }

    [[nodiscard]] const double *getData() const { return data; }

    // Control for Dynamic Assignament
    static int getDyn() {return dyn;}
    static void setDyn(int val) {dyn = val;}

    static int getCanc() {return canc;}
    static void setCanc(int val) {canc = val;}

    // Get element
    virtual double operator()(const int &r,const int &c) const;

    // Set element
    virtual double &operator()(const int &r,const int &c);

    /**
     * Initialize all entries of the MatrixBase to the same specified value
     */
    void setAllValuesAt(const double &value);

    /**
     * Operator used for initializing a MatrixBase ith the following syntax:
     * A <<= 1, 2, 3;
     */
    virtual MatrixBase &operator,(double val);

    /**
     * Operator used for initializing a MatrixBase ith the following syntax:
     * MatrixBase a <<= 1, 2, 3;
     */
    virtual MatrixBase &operator<<=(double val);

    // Return a string representation of the MatrixBase
    std::string toString();
};


#endif //SECONDASSIGNMENT_MATRIXBASE_H
