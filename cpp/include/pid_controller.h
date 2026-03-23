#pragma once
/**
 * @file pid_controller.h
 * @brief Discrete PI/PID controller with anti-windup.
 */

namespace pmsm {

struct PIConfig {
    double Kp = 0.0;
    double Ki = 0.0;
    double output_min = -1e6;
    double output_max = 1e6;
};

class PIController {
public:
    PIController() = default;
    explicit PIController(const PIConfig& config);

    /**
     * @brief Execute one control step.
     * @param error Current error (reference - measurement)
     * @param dt Sample time [s]
     * @return Controller output
     */
    double step(double error, double dt);

    /** Reset integrator state */
    void reset();

    /** Get/set integrator state (for S-Function DWork) */
    double get_integral() const { return integral_; }
    void set_integral(double val) { integral_ = val; }

    void set_config(const PIConfig& config) { config_ = config; }

private:
    PIConfig config_;
    double integral_ = 0.0;
};

}  // namespace pmsm
