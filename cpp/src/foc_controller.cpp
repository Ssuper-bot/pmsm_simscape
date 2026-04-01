#include "foc_controller.h"
#include <cmath>
#include <algorithm>

namespace pmsm {

FOCConfig derive_pi_gains_from_motor(const FOCConfig& config) {
    FOCConfig tuned = config;

    constexpr double kMin = 1e-12;
    constexpr double kTwoPi = 6.283185307179586;

    const double Ts_safe = std::max(config.Ts, kMin);
    const double fsw = 1.0 / Ts_safe;

    double omega_ci = config.omega_ci;
    if (omega_ci <= 0.0) {
        omega_ci = kTwoPi * (fsw / 10.0);
    }

    double omega_cs = config.omega_cs;
    if (omega_cs <= 0.0) {
        omega_cs = omega_ci / 10.0;
    }

    tuned.omega_ci = omega_ci;
    tuned.omega_cs = omega_cs;

    const double Ld_safe = std::max(config.Ld, kMin);
    const double Lq_safe = std::max(config.Lq, kMin);
    const double Rs_safe = std::max(config.Rs, kMin);

    // Current-loop PI design: cancel electrical pole at -Rs/L.
    tuned.Kp_id = Ld_safe * omega_ci;
    tuned.Ki_id = Rs_safe * omega_ci;
    tuned.Kp_iq = Lq_safe * omega_ci;
    tuned.Ki_iq = Rs_safe * omega_ci;

    // Speed-loop PI design: cancel mechanical pole at -B/J.
    const double kt = 1.5 * std::max(config.pole_pairs, 1) * std::max(config.flux_pm, kMin);
    const double J_safe = std::max(config.J, kMin);
    const double B_safe = std::max(config.B, kMin);
    tuned.Kp_speed = (J_safe * omega_cs) / kt;
    tuned.Ki_speed = (B_safe * omega_cs) / kt;

    return tuned;
}

// ===================== Stateless Step Function =====================

FOCOutput foc_controller_step(
    double ia, double ib, double ic,
    double theta_e, double omega_m,
    double speed_ref, double id_ref,
    double& integral_id, double& integral_iq, double& integral_speed,
    const FOCConfig& config)
{
    const FOCConfig tuned = derive_pi_gains_from_motor(config);
    double dt = tuned.Ts;

    // --- Clarke transform: abc -> alpha-beta ---
    AlphaBeta ab = clarke(ia, ib, ic);

    // --- Park transform: alpha-beta -> dq ---
    DQ dq = park(ab.alpha, ab.beta, theta_e);

    // --- Speed PI controller ---
    double speed_error = speed_ref - omega_m;
    integral_speed += speed_error * dt;
    double iq_ref = tuned.Kp_speed * speed_error + tuned.Ki_speed * integral_speed;
    iq_ref = std::clamp(iq_ref, -tuned.iq_max, tuned.iq_max);

    // Anti-windup for speed loop
    double iq_ref_unsat = tuned.Kp_speed * speed_error + tuned.Ki_speed * integral_speed;
    if (iq_ref != iq_ref_unsat) {
        integral_speed = (iq_ref - tuned.Kp_speed * speed_error) / std::max(tuned.Ki_speed, 1e-12);
    }

    // --- d-axis current PI controller ---
    double id_error = id_ref - dq.d;
    integral_id += id_error * dt;
    double vd = tuned.Kp_id * id_error + tuned.Ki_id * integral_id;

    // --- q-axis current PI controller ---
    double iq_error = iq_ref - dq.q;
    integral_iq += iq_error * dt;
    double vq = tuned.Kp_iq * iq_error + tuned.Ki_iq * integral_iq;

    // --- Decoupling feedforward ---
    double omega_e = tuned.pole_pairs * omega_m;
    vd -= omega_e * tuned.Lq * dq.q;
    vq += omega_e * tuned.Ld * dq.d + omega_e * tuned.flux_pm;

    // --- Voltage limiting ---
    double Vmax = tuned.Vdc / std::sqrt(3.0);
    double V_mag = std::sqrt(vd * vd + vq * vq);
    if (V_mag > Vmax) {
        double scale = Vmax / V_mag;
        vd *= scale;
        vq *= scale;
    }

    // --- Inverse Park: dq -> alpha-beta ---
    AlphaBeta vab = inv_park(vd, vq, theta_e);

    // --- SVPWM ---
    DutyCycles duties = svpwm(vab.alpha, vab.beta, tuned.Vdc);

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

// ===================== Stateful Controller Class =====================

FOCController::FOCController(const FOCConfig& config) {
    configure(config);
}

void FOCController::configure(const FOCConfig& config) {
    config_ = derive_pi_gains_from_motor(config);

    PIConfig id_cfg;
    id_cfg.Kp = config_.Kp_id;
    id_cfg.Ki = config_.Ki_id;
    id_cfg.output_min = -config_.Vdc;
    id_cfg.output_max = config_.Vdc;
    pi_id_.set_config(id_cfg);

    PIConfig iq_cfg;
    iq_cfg.Kp = config_.Kp_iq;
    iq_cfg.Ki = config_.Ki_iq;
    iq_cfg.output_min = -config_.Vdc;
    iq_cfg.output_max = config_.Vdc;
    pi_iq_.set_config(iq_cfg);

    PIConfig speed_cfg;
    speed_cfg.Kp = config_.Kp_speed;
    speed_cfg.Ki = config_.Ki_speed;
    speed_cfg.output_min = -config_.iq_max;
    speed_cfg.output_max = config_.iq_max;
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
