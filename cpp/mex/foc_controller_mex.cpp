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
 *
 *   output = [duty_a, duty_b, duty_c, id_meas, iq_meas, vd, vq, iq_ref]
 */

#include "mex.h"
#include "foc_controller.h"

static pmsm::FOCController g_controller;
static bool g_initialized = false;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 8) {
        mexErrMsgIdAndTxt("pmsm:foc_mex:nrhs",
            "Usage: foc_controller_mex(ia,ib,ic,theta_e,omega_m,speed_ref,id_ref,dt [,ctrl_struct, motor_struct])");
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

    // Reconfigure if config structs provided
    if (nrhs >= 10 && !g_initialized) {
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
        }

        // motor_params struct
        const mxArray* motor = prhs[9];
        if (mxIsStruct(motor)) {
            config.Rs       = mxGetScalar(mxGetField(motor, 0, "Rs"));
            config.Ld       = mxGetScalar(mxGetField(motor, 0, "Ld"));
            config.Lq       = mxGetScalar(mxGetField(motor, 0, "Lq"));
            config.flux_pm  = mxGetScalar(mxGetField(motor, 0, "flux_pm"));
            config.pole_pairs = static_cast<int>(mxGetScalar(mxGetField(motor, 0, "p")));
        }

        config.Ts = dt;
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
        ia, ib, ic, theta_e, omega_m, speed_ref, id_ref);

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
