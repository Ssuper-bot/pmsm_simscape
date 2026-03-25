/*
 * sfun_pi_controller.cpp
 *
 * Reusable PI S-Function for Simulink.
 *
 * Input  (port 0, width 1):
 *   [0] error = reference - measurement
 *
 * Output (port 0, width 1):
 *   [0] control output
 *
 * Parameters:
 *   [0] Ts          Sample time [s]
 *   [1] Kp          Proportional gain
 *   [2] Ki          Integral gain
 *   [3] output_min  Minimum output clamp
 *   [4] output_max  Maximum output clamp
 */

#define S_FUNCTION_NAME  sfun_pi_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "pid_controller.h"

#define NUM_PARAMS   5
#define NUM_INPUTS   1
#define NUM_OUTPUTS  1
#define NUM_DWORK    1

#define PARAM_TS      0
#define PARAM_KP      1
#define PARAM_KI      2
#define PARAM_OUT_MIN 3
#define PARAM_OUT_MAX 4

static double getParam(SimStruct *S, int idx)
{
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
    ssSetDWorkWidth(S, 0, 1);
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
    real_T *integral = (real_T *)ssGetDWork(S, 0);
    *integral = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);

    const real_T *u = ssGetInputPortRealSignal(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    real_T *integral = (real_T *)ssGetDWork(S, 0);

    const double Ts = getParam(S, PARAM_TS);
    const double Kp = getParam(S, PARAM_KP);
    const double Ki = getParam(S, PARAM_KI);
    const double out_min = getParam(S, PARAM_OUT_MIN);
    const double out_max = getParam(S, PARAM_OUT_MAX);

    pmsm::PIConfig cfg;
    cfg.Kp = Kp;
    cfg.Ki = Ki;
    cfg.output_min = out_min;
    cfg.output_max = out_max;

    pmsm::PIController pi(cfg);
    pi.set_integral(*integral);

    const double output = pi.step(u[0], Ts);
    *integral = pi.get_integral();

    y[0] = output;
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
