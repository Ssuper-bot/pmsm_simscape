#pragma once
/**
 * @file foc_controller.h
 * @brief Complete FOC controller for PMSM motor control.
 *
 * This is the main controller interface used by:
 *   - MATLAB S-Function (Simulink/Simscape integration)
 *   - Python ROS2 node
 *   - Standalone C++ applications
 */

#include "transforms.h"
#include "pid_controller.h"
#include "svpwm.h"

namespace pmsm {

/** FOC controller configuration */
struct FOCConfig {
    // Current PI gains
    double Kp_id = 5.0;
    double Ki_id = 1000.0;
    double Kp_iq = 5.0;
    double Ki_iq = 1000.0;
    // Speed PI gains
    double Kp_speed = 0.5;
    double Ki_speed = 10.0;
    // Motor parameters
    double Rs = 0.5;           // Stator resistance [Ohm]
    double Ld = 1.4e-3;        // d-axis inductance [H]
    double Lq = 1.4e-3;        // q-axis inductance [H]
    double flux_pm = 0.0577;   // PM flux linkage [Wb]
    int    pole_pairs = 4;
    double J = 1.74e-5;        // Rotor inertia [kg*m^2]
    double B = 1e-4;           // Viscous damping [N*m*s]
    // PI auto-tuning bandwidths
    // omega_ci <= 0: derive from switching frequency (fsw/10).
    // omega_cs <= 0: derive from current-loop bandwidth (omega_ci/10).
    double omega_ci = 0.0;     // Current-loop bandwidth [rad/s]
    double omega_cs = 0.0;     // Speed-loop bandwidth [rad/s]
    // Limits
    double iq_max = 10.0;      // Max q-axis current [A]
    double id_max = 10.0;      // Max d-axis current [A]
    // System
    double Vdc = 24.0;         // DC bus voltage [V]
    double Ts = 50e-6;         // Sample time [s]
};

/**
 * @brief Derive PI gains from motor parameters and loop bandwidths.
 *
 * Uses zero-pole cancellation design:
 *   current loop: Ki/Kp = Rs/L,  Kp = L*omega_ci, Ki = Rs*omega_ci
 *   speed loop:   Ki/Kp = B/J,   Kp = J*omega_cs/Kt, Ki = B*omega_cs/Kt
 * where Kt = 1.5 * pole_pairs * flux_pm (id_ref ~ 0 region).
 */
FOCConfig derive_pi_gains_from_motor(const FOCConfig& config);

/** Convert torque reference to q-axis current reference. */
double torque_to_iq_ref(double torque_ref, double id_ref, const FOCConfig& config);

/** FOC controller output */
struct FOCOutput {
    double duty_a;
    double duty_b;
    double duty_c;
    double id_meas;
    double iq_meas;
    double vd;
    double vq;
    double iq_ref;
};

/**
 * @brief Stateless FOC controller step function.
 *
 * This is the primary interface for S-Function integration.
 * Integrator states are passed in/out via reference parameters.
 *
 * @param ia, ib, ic      Three-phase currents [A]
 * @param theta_e         Electrical angle [rad]
 * @param omega_m         Mechanical speed [rad/s]
 * @param speed_ref       Speed reference [rad/s]
 * @param id_ref          d-axis current reference [A]
 * @param torque_ref      Torque reference [N*m]
 * @param integral_id     d-axis PI integrator state (in/out)
 * @param integral_iq     q-axis PI integrator state (in/out)
 * @param integral_speed  Speed PI integrator state (in/out)
 * @param config          Controller configuration
 * @return FOCOutput with duty cycles and measurements
 */
FOCOutput foc_controller_step(
    double ia, double ib, double ic,
    double theta_e, double omega_m,
    double speed_ref, double id_ref, double torque_ref,
    double& integral_id, double& integral_iq, double& integral_speed,
    const FOCConfig& config
);

/**
 * @brief Stateful FOC controller class.
 *
 * Maintains internal PI controller states. Suitable for standalone
 * C++ usage and Python bindings.
 */
class FOCController {
public:
    FOCController() = default;
    explicit FOCController(const FOCConfig& config);

    /** Configure the controller */
    void configure(const FOCConfig& config);

    /** Execute one FOC control step */
    FOCOutput step(double ia, double ib, double ic,
                   double theta_e, double omega_m,
                   double speed_ref, double id_ref, double torque_ref = 0.0);

    /** Reset all controller states */
    void reset();

    /** Get current configuration */
    const FOCConfig& config() const { return config_; }

private:
    FOCConfig config_;
    PIController pi_id_;
    PIController pi_iq_;
    PIController pi_speed_;
};

}  // namespace pmsm
