#pragma once
/**
 * @file svpwm.h
 * @brief Space Vector PWM modulator.
 */

namespace pmsm {

struct DutyCycles {
    double da;
    double db;
    double dc;
};

/**
 * Space Vector PWM using min-max injection (centered SVPWM).
 *
 * @param v_alpha Stationary frame alpha voltage [V]
 * @param v_beta  Stationary frame beta voltage [V]
 * @param Vdc     DC bus voltage [V]
 * @return Duty cycles for phases a, b, c in [0, 1]
 */
DutyCycles svpwm(double v_alpha, double v_beta, double Vdc);

}  // namespace pmsm
