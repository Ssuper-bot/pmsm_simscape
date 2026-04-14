#include "pid_controller.h"
#include <algorithm>
#include <cmath>

namespace pmsm {

PIController::PIController(const PIConfig& config)
    : config_(config), integral_(0.0) {}

double PIController::step(double error, double dt) {
    // Proportional
    double p_term = config_.Kp * error;

    // Integral with clamping anti-windup
    integral_ += error * dt;
    double i_term = config_.Ki * integral_;

    // Total output
    double output = p_term + i_term;

    // Clamp output
    output = std::clamp(output, config_.output_min, config_.output_max);

    // Anti-windup: back-calculate integral if saturated
    if (output != p_term + i_term && std::abs(config_.Ki) > 1e-12) {
        integral_ = (output - p_term) / config_.Ki;
    }

    return output;
}

void PIController::reset() {
    integral_ = 0.0;
}

}  // namespace pmsm
