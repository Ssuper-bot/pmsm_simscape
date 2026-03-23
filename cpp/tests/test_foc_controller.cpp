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
        0, 0, 0, 0, 0, 0, 0,
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
        auto out = ctrl.step(0, 0, 0, 0, 0, 100.0, 0);
        assert(out.duty_a >= 0.0 && out.duty_a <= 1.0);
    }
    printf("  FOC Controller Class: PASS\n");
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
    printf("All tests PASSED.\n");
    return 0;
}
