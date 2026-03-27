#include "foc_controller.h"
#include <cmath>
#include <algorithm>

namespace pmsm {

// ===================== Stateless Step Function =====================

FOCOutput foc_controller_step_with_iq_ref(
    double ia, double ib, double ic,
    double theta_e, double omega_m,
    double speed_ref, double id_ref,
    double iq_ref_external, bool use_external_iq_ref,
    double& integral_id, double& integral_iq, double& integral_speed,
    const FOCConfig& config)
{
    double dt = config.Ts;

    // --- Clarke transform: abc -> alpha-beta ---
    AlphaBeta ab = clarke(ia, ib, ic);

    // --- Park transform: alpha-beta -> dq ---
    DQ dq = park(ab.alpha, ab.beta, theta_e);

    double iq_ref = 0.0;
    if (use_external_iq_ref) {
        iq_ref = std::clamp(iq_ref_external, -config.iq_max, config.iq_max);
        integral_speed = 0.0;
    } else {
        // --- Speed PI controller ---
        double speed_error = speed_ref - omega_m;
        integral_speed += speed_error * dt;
        iq_ref = config.Kp_speed * speed_error + config.Ki_speed * integral_speed;
        iq_ref = std::clamp(iq_ref, -config.iq_max, config.iq_max);

        // Anti-windup for speed loop
        if (std::abs(config.Ki_speed) > 1e-12) {
            double iq_ref_unsat = config.Kp_speed * speed_error + config.Ki_speed * integral_speed;
            if (iq_ref != iq_ref_unsat) {
                integral_speed = (iq_ref - config.Kp_speed * speed_error) / config.Ki_speed;
            }
        }
    }

    // --- d-axis current PI controller ---
    double id_error = id_ref - dq.d;
    integral_id += id_error * dt;
    double vd = config.Kp_id * id_error + config.Ki_id * integral_id;

    // --- q-axis current PI controller ---
    double iq_error = iq_ref - dq.q;
    integral_iq += iq_error * dt;
    double vq = config.Kp_iq * iq_error + config.Ki_iq * integral_iq;

    // --- Decoupling feedforward ---
    double omega_e = config.pole_pairs * omega_m;
    vd -= omega_e * config.Lq * dq.q;
    vq += omega_e * config.Ld * dq.d + omega_e * config.flux_pm;

    // --- Voltage limiting ---
    double Vmax = config.Vdc / std::sqrt(3.0);
    double V_mag = std::sqrt(vd * vd + vq * vq);
    if (V_mag > Vmax) {
        double scale = Vmax / V_mag;
        vd *= scale;
        vq *= scale;
    }

    // --- Inverse Park: dq -> alpha-beta ---
    AlphaBeta vab = inv_park(vd, vq, theta_e);

    // --- SVPWM ---
    DutyCycles duties = svpwm(vab.alpha, vab.beta, config.Vdc);

    // --- Output ---
    FOCOutput output;
    output.duty_a = duties.da;
    output.duty_b = duties.db;
    output.duty_c = duties.dc;
    output.id_meas = dq.d;
    output.iq_meas = dq.q;
    output.vd = vd;
    output.vq = vq;
    output.iq_ref = iq_ref;

    return output;
}

FOCOutput foc_controller_step(
    double ia, double ib, double ic,
    double theta_e, double omega_m,
    double speed_ref, double id_ref,
    double& integral_id, double& integral_iq, double& integral_speed,
    const FOCConfig& config)
{
    return foc_controller_step_with_iq_ref(
        ia, ib, ic,
        theta_e, omega_m,
        speed_ref, id_ref,
        0.0, false,
        integral_id, integral_iq, integral_speed,
        config);
}

// ===================== Stateful Controller Class =====================

FOCController::FOCController(const FOCConfig& config) {
    configure(config);
}

void FOCController::configure(const FOCConfig& config) {
    config_ = config;

    PIConfig id_cfg;
    id_cfg.Kp = config.Kp_id;
    id_cfg.Ki = config.Ki_id;
    id_cfg.output_min = -config.Vdc;
    id_cfg.output_max = config.Vdc;
    pi_id_.set_config(id_cfg);

    PIConfig iq_cfg;
    iq_cfg.Kp = config.Kp_iq;
    iq_cfg.Ki = config.Ki_iq;
    iq_cfg.output_min = -config.Vdc;
    iq_cfg.output_max = config.Vdc;
    pi_iq_.set_config(iq_cfg);

    PIConfig speed_cfg;
    speed_cfg.Kp = config.Kp_speed;
    speed_cfg.Ki = config.Ki_speed;
    speed_cfg.output_min = -config.iq_max;
    speed_cfg.output_max = config.iq_max;
    pi_speed_.set_config(speed_cfg);
}

FOCOutput FOCController::step(double ia, double ib, double ic,
                               double theta_e, double omega_m,
                               double speed_ref, double id_ref) {
    double dt = config_.Ts;

    // Clarke transform
    AlphaBeta ab = clarke(ia, ib, ic);

    // Park transform
    DQ dq = park(ab.alpha, ab.beta, theta_e);

    // Speed PI controller
    double speed_error = speed_ref - omega_m;
    double iq_ref = pi_speed_.step(speed_error, dt);

    // Current PI controllers
    double id_error = id_ref - dq.d;
    double vd = pi_id_.step(id_error, dt);

    double iq_error = iq_ref - dq.q;
    double vq = pi_iq_.step(iq_error, dt);

    // Decoupling feedforward
    double omega_e = config_.pole_pairs * omega_m;
    vd -= omega_e * config_.Lq * dq.q;
    vq += omega_e * config_.Ld * dq.d + omega_e * config_.flux_pm;

    // Voltage limiting
    double Vmax = config_.Vdc / std::sqrt(3.0);
    double V_mag = std::sqrt(vd * vd + vq * vq);
    if (V_mag > Vmax) {
        double scale = Vmax / V_mag;
        vd *= scale;
        vq *= scale;
    }

    // Inverse Park
    AlphaBeta vab = inv_park(vd, vq, theta_e);

    // SVPWM
    DutyCycles duties = svpwm(vab.alpha, vab.beta, config_.Vdc);

    FOCOutput output;
    output.duty_a = duties.da;
    output.duty_b = duties.db;
    output.duty_c = duties.dc;
    output.id_meas = dq.d;
    output.iq_meas = dq.q;
    output.vd = vd;
    output.vq = vq;
    output.iq_ref = iq_ref;

    return output;
}

void FOCController::reset() {
    pi_id_.reset();
    pi_iq_.reset();
    pi_speed_.reset();
}

}  // namespace pmsm
