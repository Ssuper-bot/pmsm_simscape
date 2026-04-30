/*
 * sfun_foc_controller.cpp
 *
 * Simulink S-Function wrapper for the C++ FOC controller.
 * This S-Function can be used in Simulink/Simscape models to provide
 * Field-Oriented Control of a PMSM motor.
 *
 * Inputs (port 0, width 13):
 *   [0] ia        - Phase A current [A]
 *   [1] ib        - Phase B current [A]
 *   [2] ic        - Phase C current [A]
 *   [3] theta_e   - Electrical angle [rad]
 *   [4] omega_m   - Mechanical speed [rad/s]
 *   [5] id_ref    - d-axis current reference [A]
 *   [6] iq_ref    - q-axis current reference [A] (from speed loop and feedforward)
 *   [7] Kp_id     - Runtime d-axis PI Kp (optional; NaN keeps dialog value)
 *   [8] Ki_id     - Runtime d-axis PI Ki (optional; NaN keeps dialog value)
 *   [9] Kp_iq     - Runtime q-axis PI Kp (optional; NaN keeps dialog value)
 *  [10] Ki_iq     - Runtime q-axis PI Ki (optional; NaN keeps dialog value)
 *  [11] id_max    - Runtime d-axis current limit [A] (optional; <=0 keeps dialog value)
 *  [12] iq_max    - Runtime q-axis current limit [A] (optional; <=0 keeps dialog value)
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
 *   [2] omega_ci   - Current-loop bandwidth [rad/s]
 *   [3] Rs         - Stator resistance [Ohm]
 *   [4] Ld         - d-axis inductance [H]
 *   [5] Lq         - q-axis inductance [H]
 *   [6] flux_pm    - PM flux linkage [Wb]
 *   [7] pole_pairs - Number of pole pairs
 *   [8] iq_max     - Max q-axis current [A]
 *   [9] id_max     - Max d-axis current [A]
 *  [10] Kp_id      - d-axis current PI proportional gain
 *  [11] Ki_id      - d-axis current PI integral gain
 *  [12] Kp_iq      - q-axis current PI proportional gain
 *  [13] Ki_iq      - q-axis current PI integral gain
 *  [14] auto_tune_current - 1: use omega_ci-based auto tuning, 0: use manual PI gains
 */

#define S_FUNCTION_NAME  sfun_foc_controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "foc_controller.h"
#include <cmath>

#define NUM_PARAMS    15
#define NUM_INPUTS    13
#define NUM_OUTPUTS   5
#define NUM_DWORK     2   // integral_id, integral_iq

/* Parameter indices */
#define PARAM_TS         0
#define PARAM_VDC        1
#define PARAM_OMEGA_CI   2
#define PARAM_RS         3
#define PARAM_LD         4
#define PARAM_LQ         5
#define PARAM_FLUX_PM    6
#define PARAM_POLES      7
#define PARAM_IQ_MAX     8
#define PARAM_ID_MAX     9
#define PARAM_KP_ID     10
#define PARAM_KI_ID     11
#define PARAM_KP_IQ     12
#define PARAM_KI_IQ     13
#define PARAM_AUTO_TUNE_CURRENT 14

#define INPUT_IA         0
#define INPUT_IB         1
#define INPUT_IC         2
#define INPUT_THETA_E    3
#define INPUT_OMEGA_M    4
#define INPUT_ID_REF     5
#define INPUT_IQ_REF     6
#define INPUT_KP_ID      7
#define INPUT_KI_ID      8
#define INPUT_KP_IQ      9
#define INPUT_KI_IQ     10
#define INPUT_ID_MAX    11
#define INPUT_IQ_MAX    12

static double finiteOrDefault(double value, double default_value) {
    return std::isfinite(value) ? value : default_value;
}

static double positiveFiniteOrDefault(double value, double default_value) {
    return (std::isfinite(value) && value > 0.0) ? value : default_value;
}

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
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    ssSetDWorkDataType(S, 1, SS_DOUBLE);

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
    *integral_id    = 0.0;
    *integral_iq    = 0.0;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);

    /* Get I/O pointers */
    const real_T *u = ssGetInputPortRealSignal(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);

    /* Get DWork pointers */
    real_T *integral_id    = (real_T*)ssGetDWork(S, 0);
    real_T *integral_iq    = (real_T*)ssGetDWork(S, 1);

    /* Extract inputs */
    double ia        = u[INPUT_IA];
    double ib        = u[INPUT_IB];
    double ic        = u[INPUT_IC];
    double theta_e   = u[INPUT_THETA_E];
    double omega_m   = u[INPUT_OMEGA_M];
    double id_ref    = u[INPUT_ID_REF];
    double iq_ref    = u[INPUT_IQ_REF];

    /* Get parameters */
    /* Configure FOC controller */
    pmsm::FOCConfig config;
    config.Ts        = getParam(S, PARAM_TS);
    config.Vdc       = getParam(S, PARAM_VDC);
    config.omega_ci  = getParam(S, PARAM_OMEGA_CI);
    config.Rs        = getParam(S, PARAM_RS);
    config.Ld        = getParam(S, PARAM_LD);
    config.Lq        = getParam(S, PARAM_LQ);
    config.flux_pm   = getParam(S, PARAM_FLUX_PM);
    config.pole_pairs = static_cast<int>(getParam(S, PARAM_POLES));
    const double iq_max_default = positiveFiniteOrDefault(getParam(S, PARAM_IQ_MAX), 10.0);
    const double id_max_default = positiveFiniteOrDefault(getParam(S, PARAM_ID_MAX), 10.0);
    config.iq_max = positiveFiniteOrDefault(u[INPUT_IQ_MAX], iq_max_default);
    config.id_max = positiveFiniteOrDefault(u[INPUT_ID_MAX], id_max_default);

    const double kp_id_rt = u[INPUT_KP_ID];
    const double ki_id_rt = u[INPUT_KI_ID];
    const double kp_iq_rt = u[INPUT_KP_IQ];
    const double ki_iq_rt = u[INPUT_KI_IQ];
    const bool has_runtime_pi = std::isfinite(kp_id_rt) && std::isfinite(ki_id_rt) &&
        std::isfinite(kp_iq_rt) && std::isfinite(ki_iq_rt);

    config.Kp_id = finiteOrDefault(kp_id_rt, getParam(S, PARAM_KP_ID));
    config.Ki_id = finiteOrDefault(ki_id_rt, getParam(S, PARAM_KI_ID));
    config.Kp_iq = finiteOrDefault(kp_iq_rt, getParam(S, PARAM_KP_IQ));
    config.Ki_iq = finiteOrDefault(ki_iq_rt, getParam(S, PARAM_KI_IQ));
    config.auto_tune_current = has_runtime_pi ? false : (getParam(S, PARAM_AUTO_TUNE_CURRENT) != 0.0);
    config.auto_tune_speed = false;
    /* Current-loop S-Function does not run speed control internally. */
    config.enable_internal_speed_loop = false;

    /* Run current-loop FOC step */
    pmsm::FOCOutput output = pmsm::current_controller_step(
        ia, ib, ic, theta_e, omega_m, id_ref, iq_ref,
        *integral_id, *integral_iq,
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
