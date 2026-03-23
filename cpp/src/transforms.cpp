#include "transforms.h"
#include <cmath>

namespace pmsm {

AlphaBeta clarke(double ia, double ib, double ic) {
    constexpr double k = 2.0 / 3.0;
    constexpr double sqrt3_2 = 0.86602540378; // sqrt(3)/2
    return {
        k * (ia - 0.5 * ib - 0.5 * ic),
        k * (sqrt3_2 * ib - sqrt3_2 * ic)
    };
}

DQ park(double alpha, double beta, double theta_e) {
    double cos_t = std::cos(theta_e);
    double sin_t = std::sin(theta_e);
    return {
         alpha * cos_t + beta * sin_t,
        -alpha * sin_t + beta * cos_t
    };
}

AlphaBeta inv_park(double d, double q, double theta_e) {
    double cos_t = std::cos(theta_e);
    double sin_t = std::sin(theta_e);
    return {
        d * cos_t - q * sin_t,
        d * sin_t + q * cos_t
    };
}

ABC inv_clarke(double alpha, double beta) {
    constexpr double sqrt3_2 = 0.86602540378;
    return {
        alpha,
        -0.5 * alpha + sqrt3_2 * beta,
        -0.5 * alpha - sqrt3_2 * beta
    };
}

}  // namespace pmsm
