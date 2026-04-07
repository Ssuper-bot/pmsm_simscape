/**
 * @file foc_controller_mex.cpp
 * @brief MEX wrapper for calling the C++ FOC controller from MATLAB scripts.
 *
 * This is NOT an S-Function. It's a regular MEX function for use with
 * MATLAB scripts (e.g., run_pmsm_simulation.m).
 *
 * Usage in MATLAB:
 *   output = foc_controller_mex(ia, ib, ic, theta_e, omega_m, ...
 *                               speed_ref, id_ref, dt, ctrl_struct, motor_struct);
 *   output = foc_controller_mex(ia, ib, ic, theta_e, omega_m, ...
 *                               speed_ref, id_ref, dt, ctrl_struct, motor_struct, torque_ref);
 *
 *   output = [duty_a, duty_b, duty_c, id_meas, iq_meas, vd, vq, iq_ref]
 */

#include "mex.h"
#include "foc_controller.h"

static bool hasField(const mxArray* s, const char* name) {
    return mxGetField(s, 0, name) != nullptr;
}

static pmsm::FOCController g_controller;
static bool g_initialized = false;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 8) {
        mexErrMsgIdAndTxt("pmsm:foc_mex:nrhs",
            "Usage: foc_controller_mex(ia,ib,ic,theta_e,omega_m,speed_ref,id_ref,dt [,ctrl_struct, motor_struct, torque_ref])");
        return;
    }

    double ia        = mxGetScalar(prhs[0]);
    double ib        = mxGetScalar(prhs[1]);
    double ic        = mxGetScalar(prhs[2]);
    double theta_e   = mxGetScalar(prhs[3]);
    double omega_m   = mxGetScalar(prhs[4]);
    double speed_ref = mxGetScalar(prhs[5]);
    double id_ref    = mxGetScalar(prhs[6]);
    double dt        = mxGetScalar(prhs[7]);
    double torque_ref = (nrhs >= 11) ? mxGetScalar(prhs[10]) : 0.0;

    // Reconfigure if config structs provided
    if (nrhs >= 10) {
        pmsm::FOCConfig config;

        // ctrl_params struct
        const mxArray* ctrl = prhs[8];
        if (mxIsStruct(ctrl)) {
            config.Kp_id    = mxGetScalar(mxGetField(ctrl, 0, "Kp_id"));
            config.Ki_id    = mxGetScalar(mxGetField(ctrl, 0, "Ki_id"));
            config.Kp_iq    = mxGetScalar(mxGetField(ctrl, 0, "Kp_iq"));
            config.Ki_iq    = mxGetScalar(mxGetField(ctrl, 0, "Ki_iq"));
            config.Kp_speed = mxGetScalar(mxGetField(ctrl, 0, "Kp_speed"));
            config.Ki_speed = mxGetScalar(mxGetField(ctrl, 0, "Ki_speed"));
            config.iq_max   = mxGetScalar(mxGetField(ctrl, 0, "iq_max"));
            config.id_max   = mxGetScalar(mxGetField(ctrl, 0, "id_max"));
            if (hasField(ctrl, "omega_ci")) {
                config.omega_ci = mxGetScalar(mxGetField(ctrl, 0, "omega_ci"));
            }
            if (hasField(ctrl, "omega_cs")) {
                config.omega_cs = mxGetScalar(mxGetField(ctrl, 0, "omega_cs"));
            }
            if (hasField(ctrl, "Vdc")) {
                config.Vdc = mxGetScalar(mxGetField(ctrl, 0, "Vdc"));
            }
            if (hasField(ctrl, "Ts")) {
                config.Ts = mxGetScalar(mxGetField(ctrl, 0, "Ts"));
            }
        }

        // motor_params struct
        const mxArray* motor = prhs[9];
        if (mxIsStruct(motor)) {
            config.Rs       = mxGetScalar(mxGetField(motor, 0, "Rs"));
            config.Ld       = mxGetScalar(mxGetField(motor, 0, "Ld"));
            config.Lq       = mxGetScalar(mxGetField(motor, 0, "Lq"));
            config.flux_pm  = mxGetScalar(mxGetField(motor, 0, "flux_pm"));
            config.pole_pairs = static_cast<int>(mxGetScalar(mxGetField(motor, 0, "p")));
            if (hasField(motor, "J")) {
                config.J = mxGetScalar(mxGetField(motor, 0, "J"));
            }
            if (hasField(motor, "B")) {
                config.B = mxGetScalar(mxGetField(motor, 0, "B"));
            }
        }

        if (config.Ts <= 0.0) {
            config.Ts = dt;
        }
        g_controller.configure(config);
        g_initialized = true;
    } else if (!g_initialized) {
        pmsm::FOCConfig config;
        config.Ts = dt;
        g_controller.configure(config);
        g_initialized = true;
    }

    // Run one step
    pmsm::FOCOutput output = g_controller.step(
        ia, ib, ic, theta_e, omega_m, speed_ref, id_ref, torque_ref);

    // Return output as 1x8 vector
    plhs[0] = mxCreateDoubleMatrix(1, 8, mxREAL);
    double* out = mxGetPr(plhs[0]);
    out[0] = output.duty_a;
    out[1] = output.duty_b;
    out[2] = output.duty_c;
    out[3] = output.id_meas;
    out[4] = output.iq_meas;
    out[5] = output.vd;
    out[6] = output.vq;
    out[7] = output.iq_ref;
}
