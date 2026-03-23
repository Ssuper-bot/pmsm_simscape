/*
 * sfun_foc_controller.cpp
 *
 * Simulink S-Function wrapper for the C++ FOC controller.
 * This S-Function can be used in Simulink/Simscape models to provide
 * Field-Oriented Control of a PMSM motor.
 *
 * Inputs (port 0, width 7):
 *   [0] ia        - Phase A current [A]
 *   [1] ib        - Phase B current [A]
 *   [2] ic        - Phase C current [A]
 *   [3] theta_e   - Electrical angle [rad]
 *   [4] omega_m   - Mechanical speed [rad/s]
 *   [5] speed_ref - Speed reference [rad/s]
 *   [6] id_ref    - d-axis current reference [A]
 *
 * Outputs (port 0, width 5):
 *   [0] da        - Phase A duty cycle [0,1]
 *   [1] db        - Phase B duty cycle [0,1]
 *   [2] dc        - Phase C duty cycle [0,1]
 *   [3] id_meas   - Measured d-axis current [A]
 *   [4] iq_meas   - Measured q-axis current [A]
 *
 * Parameters (dialog):
 *   [0] Ts         - Sample time [s]
 *   [1] Vdc        - DC bus voltage [V]
 *   [2] Kp_id      - d-axis current P gain
 *   [3] Ki_id      - d-axis current I gain
 *   [4] Kp_iq      - q-axis current P gain
 *   [5] Ki_iq      - q-axis current I gain
 *   [6] Kp_speed   - Speed P gain
 *   [7] Ki_speed   - Speed I gain
 *   [8] Rs         - Stator resistance [Ohm]
 *   [9] Ld         - d-axis inductance [H]
 *  [10] Lq         - q-axis inductance [H]
 *  [11] flux_pm    - PM flux linkage [Wb]
 *  [12] pole_pairs - Number of pole pairs
 *  [13] iq_max     - Max q-axis current [A]
 *  [14] id_max     - Max d-axis current [A]
 */

#define S_FUNCTION_NAME  sfun_foc_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "foc_controller.h"

#define NUM_PARAMS    15
#define NUM_INPUTS    7
#define NUM_OUTPUTS   5
#define NUM_DWORK     3   // integral_id, integral_iq, integral_speed

/* Parameter indices */
#define PARAM_TS         0
#define PARAM_VDC        1
#define PARAM_KP_ID      2
#define PARAM_KI_ID      3
#define PARAM_KP_IQ      4
#define PARAM_KI_IQ      5
#define PARAM_KP_SPEED   6
#define PARAM_KI_SPEED   7
#define PARAM_RS         8
#define PARAM_LD         9
#define PARAM_LQ        10
#define PARAM_FLUX_PM   11
#define PARAM_POLES     12
#define PARAM_IQ_MAX    13
#define PARAM_ID_MAX    14

static double getParam(SimStruct *S, int idx) {
    return mxGetScalar(ssGetSFcnParam(S, idx));
}

/*====================*
 * S-function methods *
 *====================*/

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    /* Input port */
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, NUM_INPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    /* Output port */
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, NUM_OUTPUTS);

    /* Sample time */
    ssSetNumSampleTimes(S, 1);

    /* DWork vectors for controller state */
    ssSetNumDWork(S, NUM_DWORK);
    ssSetDWorkWidth(S, 0, 1); // integral_id
    ssSetDWorkWidth(S, 1, 1); // integral_iq
    ssSetDWorkWidth(S, 2, 1); // integral_speed
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    ssSetDWorkDataType(S, 1, SS_DOUBLE);
    ssSetDWorkDataType(S, 2, SS_DOUBLE);

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
    /* Zero integrator states */
    real_T *integral_id    = (real_T*)ssGetDWork(S, 0);
    real_T *integral_iq    = (real_T*)ssGetDWork(S, 1);
    real_T *integral_speed = (real_T*)ssGetDWork(S, 2);
    *integral_id    = 0.0;
    *integral_iq    = 0.0;
    *integral_speed = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Get I/O pointers */
    const real_T *u = ssGetInputPortRealSignal(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);

    /* Get DWork pointers */
    real_T *integral_id    = (real_T*)ssGetDWork(S, 0);
    real_T *integral_iq    = (real_T*)ssGetDWork(S, 1);
    real_T *integral_speed = (real_T*)ssGetDWork(S, 2);

    /* Extract inputs */
    double ia        = u[0];
    double ib        = u[1];
    double ic        = u[2];
    double theta_e   = u[3];
    double omega_m   = u[4];
    double speed_ref = u[5];
    double id_ref    = u[6];

    /* Get parameters */
    double Ts       = getParam(S, PARAM_TS);
    double Vdc      = getParam(S, PARAM_VDC);

    /* Configure FOC controller */
    pmsm::FOCConfig config;
    config.Kp_id     = getParam(S, PARAM_KP_ID);
    config.Ki_id     = getParam(S, PARAM_KI_ID);
    config.Kp_iq     = getParam(S, PARAM_KP_IQ);
    config.Ki_iq     = getParam(S, PARAM_KI_IQ);
    config.Kp_speed  = getParam(S, PARAM_KP_SPEED);
    config.Ki_speed  = getParam(S, PARAM_KI_SPEED);
    config.Rs        = getParam(S, PARAM_RS);
    config.Ld        = getParam(S, PARAM_LD);
    config.Lq        = getParam(S, PARAM_LQ);
    config.flux_pm   = getParam(S, PARAM_FLUX_PM);
    config.pole_pairs = static_cast<int>(getParam(S, PARAM_POLES));
    config.iq_max    = getParam(S, PARAM_IQ_MAX);
    config.id_max    = getParam(S, PARAM_ID_MAX);
    config.Vdc       = Vdc;
    config.Ts        = Ts;

    /* Run FOC controller step */
    pmsm::FOCOutput output = pmsm::foc_controller_step(
        ia, ib, ic, theta_e, omega_m, speed_ref, id_ref,
        *integral_id, *integral_iq, *integral_speed,
        config
    );

    /* Write outputs */
    y[0] = output.duty_a;
    y[1] = output.duty_b;
    y[2] = output.duty_c;
    y[3] = output.id_meas;
    y[4] = output.iq_meas;
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
