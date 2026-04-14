/**
 * @file pybind_module.cpp
 * @brief pybind11 module exposing C++ FOC controller to Python.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "foc_controller.h"

namespace py = pybind11;

PYBIND11_MODULE(_pmsm_cpp, m) {
    m.doc() = "PMSM FOC Controller C++ bindings";

    // Transform functions
    m.def("clarke", [](double ia, double ib, double ic) {
        auto ab = pmsm::clarke(ia, ib, ic);
        return std::make_tuple(ab.alpha, ab.beta);
    }, "Clarke transform: abc -> alpha-beta");

    m.def("park", [](double alpha, double beta, double theta_e) {
        auto dq = pmsm::park(alpha, beta, theta_e);
        return std::make_tuple(dq.d, dq.q);
    }, "Park transform: alpha-beta -> dq");

    m.def("inv_park", [](double d, double q, double theta_e) {
        auto ab = pmsm::inv_park(d, q, theta_e);
        return std::make_tuple(ab.alpha, ab.beta);
    }, "Inverse Park transform: dq -> alpha-beta");

    m.def("svpwm", [](double v_alpha, double v_beta, double Vdc) {
        auto duty = pmsm::svpwm(v_alpha, v_beta, Vdc);
        return std::make_tuple(duty.da, duty.db, duty.dc);
    }, "Space Vector PWM");

    m.def("derive_pi_gains_from_motor", &pmsm::derive_pi_gains_from_motor,
          "Derive PI gains from motor parameters and loop bandwidths");

    m.def("speed_controller_step", [](double speed_ref, double omega_m,
                                      double integral_speed,
                                      const pmsm::FOCConfig& config) {
        const double iq_ref = pmsm::speed_controller_step(
            speed_ref, omega_m, integral_speed, config);
        return std::make_tuple(iq_ref, integral_speed);
    }, "Stateless speed-loop PI step. Returns (iq_ref, updated_integral_speed)");

    m.def("current_controller_step", [](double ia, double ib, double ic,
                                        double theta_e, double omega_m,
                                        double id_ref, double iq_ref,
                                        double integral_id, double integral_iq,
                                        const pmsm::FOCConfig& config) {
        auto out = pmsm::current_controller_step(
            ia, ib, ic, theta_e, omega_m,
            id_ref, iq_ref,
            integral_id, integral_iq,
            config);
        return std::make_tuple(out, integral_id, integral_iq);
    }, "Stateless current-loop FOC step. Returns (FOCOutput, integral_id, integral_iq)");

    // FOCConfig
    py::class_<pmsm::FOCConfig>(m, "FOCConfig")
        .def(py::init<>())
        .def_readwrite("Kp_id", &pmsm::FOCConfig::Kp_id)
        .def_readwrite("Ki_id", &pmsm::FOCConfig::Ki_id)
        .def_readwrite("Kp_iq", &pmsm::FOCConfig::Kp_iq)
        .def_readwrite("Ki_iq", &pmsm::FOCConfig::Ki_iq)
        .def_readwrite("Kp_speed", &pmsm::FOCConfig::Kp_speed)
        .def_readwrite("Ki_speed", &pmsm::FOCConfig::Ki_speed)
        .def_readwrite("auto_tune_current", &pmsm::FOCConfig::auto_tune_current)
        .def_readwrite("auto_tune_speed", &pmsm::FOCConfig::auto_tune_speed)
        .def_readwrite("enable_internal_speed_loop", &pmsm::FOCConfig::enable_internal_speed_loop)
        .def_readwrite("Rs", &pmsm::FOCConfig::Rs)
        .def_readwrite("Ld", &pmsm::FOCConfig::Ld)
        .def_readwrite("Lq", &pmsm::FOCConfig::Lq)
        .def_readwrite("flux_pm", &pmsm::FOCConfig::flux_pm)
        .def_readwrite("pole_pairs", &pmsm::FOCConfig::pole_pairs)
        .def_readwrite("J", &pmsm::FOCConfig::J)
        .def_readwrite("B", &pmsm::FOCConfig::B)
        .def_readwrite("omega_ci", &pmsm::FOCConfig::omega_ci)
        .def_readwrite("omega_cs", &pmsm::FOCConfig::omega_cs)
        .def_readwrite("iq_max", &pmsm::FOCConfig::iq_max)
        .def_readwrite("id_max", &pmsm::FOCConfig::id_max)
        .def_readwrite("Vdc", &pmsm::FOCConfig::Vdc)
        .def_readwrite("Ts", &pmsm::FOCConfig::Ts);

    // FOCOutput
    py::class_<pmsm::FOCOutput>(m, "FOCOutput")
        .def(py::init<>())
        .def_readwrite("duty_a", &pmsm::FOCOutput::duty_a)
        .def_readwrite("duty_b", &pmsm::FOCOutput::duty_b)
        .def_readwrite("duty_c", &pmsm::FOCOutput::duty_c)
        .def_readwrite("id_meas", &pmsm::FOCOutput::id_meas)
        .def_readwrite("iq_meas", &pmsm::FOCOutput::iq_meas)
        .def_readwrite("vd", &pmsm::FOCOutput::vd)
        .def_readwrite("vq", &pmsm::FOCOutput::vq)
        .def_readwrite("iq_ref", &pmsm::FOCOutput::iq_ref);

    // FOCController
    py::class_<pmsm::FOCController>(m, "FOCController")
        .def(py::init<>())
        .def(py::init<const pmsm::FOCConfig&>())
        .def("configure", &pmsm::FOCController::configure)
        .def("step", &pmsm::FOCController::step)
        .def("step_speed", &pmsm::FOCController::step_speed)
        .def("step_current", &pmsm::FOCController::step_current)
        .def("reset", &pmsm::FOCController::reset)
        .def("config", &pmsm::FOCController::config, py::return_value_policy::reference_internal);
}
