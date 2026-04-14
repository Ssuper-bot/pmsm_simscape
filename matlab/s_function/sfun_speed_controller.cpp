/*
 * sfun_speed_controller.cpp
 *
 * Simulink S-Function wrapper for speed-loop PI in the C++ control core.
 *
 * Inputs (port 0, width 2):
 *   [0] speed_ref - Speed reference [RPM]
 *   [1] omega_m   - Mechanical speed [rad/s]
 *
 * Outputs (port 0, width 1):
 *   [0] iq_ref    - q-axis current reference [A]
 *
 * Parameters (dialog):
 *   [0] Ts         - Sample time [s]
 *   [1] Kp_speed   - Speed PI proportional gain
 *   [2] Ki_speed   - Speed PI integral gain
 *   [3] iq_max     - Max q-axis current [A]
 */

#define S_FUNCTION_NAME  sfun_speed_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "foc_controller.h"

#define NUM_PARAMS  4
#define NUM_INPUTS  2
#define NUM_OUTPUTS 1
#define NUM_DWORK   1

#define PARAM_TS        0
#define PARAM_KP_SPEED  1
#define PARAM_KI_SPEED  2
#define PARAM_IQ_MAX    3

static double getParam(SimStruct *S, int idx) {
    return mxGetScalar(ssGetSFcnParam(S, idx));
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, NUM_INPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, NUM_OUTPUTS);

    ssSetNumSampleTimes(S, 1);

    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, 0, 1);  // integral_speed
    ssSetDWorkDataType(S, 0, SS_DOUBLE);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, getParam(S, PARAM_TS));
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *integral_speed = (real_T*)ssGetDWork(S, 0);
    *integral_speed = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);

    const real_T *u = ssGetInputPortRealSignal(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);

    real_T *integral_speed = (real_T*)ssGetDWork(S, 0);

    const double speed_ref_rpm = u[0];
    const double omega_m = u[1];

    const double speed_ref = speed_ref_rpm * (2.0 * 3.14159265358979323846 / 60.0);

    pmsm::FOCConfig config;
    config.Ts = getParam(S, PARAM_TS);
    config.Kp_speed = getParam(S, PARAM_KP_SPEED);
    config.Ki_speed = getParam(S, PARAM_KI_SPEED);
    config.iq_max = getParam(S, PARAM_IQ_MAX);
    config.auto_tune_speed = false;
    config.auto_tune_current = false;

    const double iq_ref = pmsm::speed_controller_step(
        speed_ref, omega_m, *integral_speed, config);

    y[0] = iq_ref;
}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
