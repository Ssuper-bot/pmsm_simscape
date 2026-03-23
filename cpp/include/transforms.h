#pragma once
/**
 * @file transforms.h
 * @brief Clarke, Park, and inverse transforms for PMSM control.
 */

namespace pmsm {

struct AlphaBeta {
    double alpha;
    double beta;
};

struct DQ {
    double d;
    double q;
};

struct ABC {
    double a;
    double b;
    double c;
};

/**
 * Clarke transform: abc -> alpha-beta (power-invariant)
 */
AlphaBeta clarke(double ia, double ib, double ic);

/**
 * Park transform: alpha-beta -> dq
 */
DQ park(double alpha, double beta, double theta_e);

/**
 * Inverse Park transform: dq -> alpha-beta
 */
AlphaBeta inv_park(double d, double q, double theta_e);

/**
 * Inverse Clarke transform: alpha-beta -> abc
 */
ABC inv_clarke(double alpha, double beta);

}  // namespace pmsm
