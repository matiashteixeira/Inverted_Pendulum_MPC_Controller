#include <Arduino.h>
#include "qpOASES.hpp"

using namespace qpOASES;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("=== Teste qpOASES: QP com Restricoes ===");

    // ------------------------------
    // Define dimensões
    // ------------------------------
    int nV = 2;   // variáveis
    int nC = 3;   // restrições

    // ------------------------------
    // Matriz Hessiana H (simétrica)
    // ------------------------------
    real_t H[4] = {
        4.0, 1.0,
        1.0, 2.0
    };

    // Vetor g
    real_t g[2] = { -8.0, -3.0 };

    // ------------------------------
    // Matriz A das restrições
    // A*x <= ub
    // ------------------------------
    real_t A[6] = {
        1.0, 1.0,   // x1 + x2 <= 2
        -1.0, 0.0,  // -x1 <= 0  -> x1 >= 0
        0.0, -1.0   // -x2 <= 0  -> x2 >= 0
    };

    // Limites (somente superiores)
    real_t lbA[3] = { -1e20, -1e20, -1e20 };   // sem limites inferiores
    real_t ubA[3] = { 2.0, 0.0, 0.0 };

    // Variáveis x sem limites
    real_t lb[2] = { -1e20, -1e20 };
    real_t ub[2] = {  1e20,  1e20 };

    // ------------------------------
    // Cria o problema QP
    // ------------------------------
    QProblem qp(nV, nC);

    Options opts;
    opts.printLevel = PL_LOW;   // reduz prints
    qp.setOptions(opts);

    int nWSR = 50;

    // ------------------------------
    // Resolve o problema
    // ------------------------------
    qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

    // ------------------------------
    // Captura a solução
    // ------------------------------
    real_t xOpt[2];
    qp.getPrimalSolution(xOpt);

    Serial.println("Solução encontrada:");
    Serial.print("x1 = "); Serial.println(xOpt[0], 6);
    Serial.print("x2 = "); Serial.println(xOpt[1], 6);

    Serial.println("=== Fim ===");
}

void loop() {
}
