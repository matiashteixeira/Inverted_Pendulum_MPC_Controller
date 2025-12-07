#ifndef MPC_H
#define MPC_H

#include <vector>
#include <string>
#include <iostream>
#include <cassert>
#include <Arduino.h>


struct Matrix {
    size_t r, c;
    std::vector<float> d;
    Matrix(): r(0), c(0) {}
    Matrix(size_t r_, size_t c_, float v=0.0): r(r_), c(c_), d(r_*c_, v) {}
    void resize(size_t r_, size_t c_, float v=0.0){ r=r_; c=c_; d.assign(r*c, v); }
    float& operator()(size_t i, size_t j){ return d[i*c + j]; }
    float  operator()(size_t i, size_t j) const { return d[i*c + j]; }
};

class MPC {
public:
    Matrix A, B, Cc, Dc, Cr;

    size_t n, nu, ny, nc;
    size_t N;

    Matrix Qu, Qy;

    Matrix ycmax, ycmin, deltamax, deltamin;
    Matrix umax, umin;

    Matrix H, F1, F2, F3;
    Matrix Aineq, G1, G2, G3;

    std::vector<float> utildemax;
    std::vector<float> utildemin;

    MPC();

    void compute_MPC_Matrices();
    void printMatrix(const Matrix& M);

private:
    void compute_cost_matrices();
    void compute_constraints_matrices();

};

#endif
