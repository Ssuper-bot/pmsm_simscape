/**
 * @file test_foc_controller.cpp
 * @brief Basic unit test for FOC controller components.
 */

#include "foc_controller.h"
#include <cmath>
#include <cstdio>
#include <cassert>

static constexpr double TOL = 1e-6;

static bool approx_eq(double a, double b, double tol = TOL) {
    return std::fabs(a - b) < tol;
}

void test_clarke_transform() {
    // Balanced three-phase: ia=1, ib=-0.5, ic=-0.5
    auto ab = pmsm::clarke(1.0, -0.5, -0.5);
    assert(approx_eq(ab.alpha, 1.0));
    assert(approx_eq(ab.beta, 0.0));
    printf("  Clarke transform: PASS\n");
}

void test_park_transform() {
    // At theta=0, Park is identity-like
    auto dq = pmsm::park(1.0, 0.0, 0.0);
    assert(approx_eq(dq.d, 1.0));
    assert(approx_eq(dq.q, 0.0));

    // At theta=pi/2
    dq = pmsm::park(1.0, 0.0, M_PI / 2);
    assert(approx_eq(dq.d, 0.0, 1e-10));
    assert(approx_eq(dq.q, -1.0, 1e-10));
    printf("  Park transform: PASS\n");
}

void test_inv_park_transform() {
    // Inverse of Park should recover original
    double d = 3.0, q = 4.0, theta = 0.7;
    auto ab = pmsm::inv_park(d, q, theta);
    auto dq = pmsm::park(ab.alpha, ab.beta, theta);
    assert(approx_eq(dq.d, d));
    assert(approx_eq(dq.q, q));
    printf("  Inverse Park transform: PASS\n");
}

void test_svpwm() {
    // Zero voltage should give 50% duty
    auto duty = pmsm::svpwm(0.0, 0.0, 24.0);
    assert(approx_eq(duty.da, 0.5));
    assert(approx_eq(duty.db, 0.5));
    assert(approx_eq(duty.dc, 0.5));
    printf("  SVPWM: PASS\n");
}

void test_pi_controller() {
    pmsm::PIConfig cfg;
    cfg.Kp = 1.0;
    cfg.Ki = 10.0;
    cfg.output_min = -100;
    cfg.output_max = 100;

    pmsm::PIController pi(cfg);
    double dt = 0.001;

    // Step response
    double out = pi.step(1.0, dt);
    // First step: P=1.0 + I=10*0.001*1.0=0.01 = 1.01
    assert(approx_eq(out, 1.01, 0.001));
    printf("  PI Controller: PASS\n");
}

void test_foc_controller_step() {
    pmsm::FOCConfig config;
    config.Ts = 50e-6;
    config.Vdc = 24.0;

    double int_id = 0, int_iq = 0, int_speed = 0;

    // Zero inputs, zero reference
    auto output = pmsm::foc_controller_step(
        0, 0, 0, 0, 0, 0, 0, 0,
        int_id, int_iq, int_speed, config);

    assert(output.duty_a >= 0.0 && output.duty_a <= 1.0);
    assert(output.duty_b >= 0.0 && output.duty_b <= 1.0);
    assert(output.duty_c >= 0.0 && output.duty_c <= 1.0);
    printf("  FOC Controller Step: PASS\n");
}

void test_foc_controller_class() {
    pmsm::FOCConfig config;
    config.Ts = 50e-6;
    config.Vdc = 24.0;

    pmsm::FOCController ctrl(config);

    // Run several steps with speed reference
    for (int i = 0; i < 100; i++) {
        auto out = ctrl.step(0, 0, 0, 0, 0, 100.0, 0, 0.0);
        assert(out.duty_a >= 0.0 && out.duty_a <= 1.0);
    }
    printf("  FOC Controller Class: PASS\n");
}

void test_torque_to_iq_reference() {
    pmsm::FOCConfig config;
    config.flux_pm = 0.0577;
    config.Ld = 1.4e-3;
    config.Lq = 1.4e-3;
    config.pole_pairs = 4;

    const double iq_ref = pmsm::torque_to_iq_ref(0.12, 0.0, config);
    const double expected = 0.12 / (1.5 * config.pole_pairs * config.flux_pm);
    assert(approx_eq(iq_ref, expected, 1e-12));
    printf("  Torque-to-iq conversion: PASS\n");
}

void test_motor_based_pi_derivation() {
    pmsm::FOCConfig config;
    config.Ts = 1.0 / 20000.0;
    config.Rs = 0.5;
    config.Ld = 1.4e-3;
    config.Lq = 1.4e-3;
    config.flux_pm = 0.0577;
    config.pole_pairs = 4;
    config.J = 1.74e-5;
    config.B = 1e-4;
    config.omega_ci = 0.0;
    config.omega_cs = 0.0;

    auto tuned = pmsm::derive_pi_gains_from_motor(config);

    double omega_ci_expected = 2.0 * M_PI * (20000.0 / 10.0);
    double omega_cs_expected = omega_ci_expected / 10.0;
    double kt = 1.5 * config.pole_pairs * config.flux_pm;

    assert(approx_eq(tuned.omega_ci, omega_ci_expected, 1e-9));
    assert(approx_eq(tuned.omega_cs, omega_cs_expected, 1e-9));
    assert(approx_eq(tuned.Kp_id, config.Ld * omega_ci_expected, 1e-9));
    assert(approx_eq(tuned.Ki_id, config.Rs * omega_ci_expected, 1e-9));
    assert(approx_eq(tuned.Kp_speed, (config.J * omega_cs_expected) / kt, 1e-9));
    assert(approx_eq(tuned.Ki_speed, (config.B * omega_cs_expected) / kt, 1e-9));
    printf("  Motor-based PI derivation: PASS\n");
}

int main() {
    printf("Running FOC Controller Tests...\n");
    test_clarke_transform();
    test_park_transform();
    test_inv_park_transform();
    test_svpwm();
    test_pi_controller();
    test_foc_controller_step();
    test_foc_controller_class();
    test_motor_based_pi_derivation();
    test_torque_to_iq_reference();
    printf("All tests PASSED.\n");
    return 0;
}
