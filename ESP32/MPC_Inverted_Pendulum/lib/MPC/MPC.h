#ifndef MPC_H
#define MPC_H

#include <Arduino.h>
#include <qpOASES.hpp>
#include <cmath>
#include <vector>

using namespace std;

// Defina aqui o tamanho do horizonte do MPC
#define N 40
#define Nx 4

class MPCController {
public:
    MPCController();   
    float compute(float x, float theta, float x_dot, float theta_dot, float set_point_x);

private:
    float ulast;

    // Matrizes do MPC
    float  H[N][N];

    float  F[N];
    float  F1[N][Nx];
    float  F2[N][N*2];

    float  G1[N*6][Nx];
    float  G2[N*6];
    float  G3[N*6];

    float  Aineq[N*6][N];
    float  Bineq[N];

    float  utildemin[N];
    float  utildemax[N];
    float  utilde_opt[N];

    qpOASES::QProblem qp;
    bool qp_initialized;

    void generateYref(float set_point_x, float yref[]);
    void computeF_Bineq(const float x[], float yref[]);
    void initQpOases();
    void initMatrizes();
};

#endif
