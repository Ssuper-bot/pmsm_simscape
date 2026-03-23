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

    // FOCConfig
    py::class_<pmsm::FOCConfig>(m, "FOCConfig")
        .def(py::init<>())
        .def_readwrite("Kp_id", &pmsm::FOCConfig::Kp_id)
        .def_readwrite("Ki_id", &pmsm::FOCConfig::Ki_id)
        .def_readwrite("Kp_iq", &pmsm::FOCConfig::Kp_iq)
        .def_readwrite("Ki_iq", &pmsm::FOCConfig::Ki_iq)
        .def_readwrite("Kp_speed", &pmsm::FOCConfig::Kp_speed)
        .def_readwrite("Ki_speed", &pmsm::FOCConfig::Ki_speed)
        .def_readwrite("Rs", &pmsm::FOCConfig::Rs)
        .def_readwrite("Ld", &pmsm::FOCConfig::Ld)
        .def_readwrite("Lq", &pmsm::FOCConfig::Lq)
        .def_readwrite("flux_pm", &pmsm::FOCConfig::flux_pm)
        .def_readwrite("pole_pairs", &pmsm::FOCConfig::pole_pairs)
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
        .def("reset", &pmsm::FOCController::reset)
        .def("config", &pmsm::FOCController::config, py::return_value_policy::reference_internal);
}
