/* sfun_rflymac.c: Simulink Level-2 C-MEX S-function "CopterSim (Mac)".
 *
 * Connects Simulink to `coptersim --ext` (lockstep). Every sample period Ts it sends the motor
 * commands (input 1, 1..8 values in 0..1, PX4 motor order) and advances the simulator by Ts.
 * Not direct-feedthrough: outputs at step k are the state after step k-1, so a controller can be
 * wired in a loop without an algebraic loop.
 *
 * Parameters: 1) UDP port (20100)  2) sample time Ts in s (multiple of 0.001)  3) host ('127.0.0.1')
 * Outputs: 1 time, 2 pos NED, 3 vel NED, 4 quat, 5 euler, 6 body rates, 7 accel, 8 gyro, 9 mag,
 *          10 baro alt, 11 gps [lat lon alt], 12 motor speeds, 13 on_ground
 * Build: see install_rflymac.m */
#define S_FUNCTION_NAME sfun_rflymac
#define S_FUNCTION_LEVEL 2

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "simstruc.h"
#include "rflymac_client.h"

#define P_PORT(S) ssGetSFcnParam(S, 0)
#define P_TS(S) ssGetSFcnParam(S, 1)
#define P_HOST(S) ssGetSFcnParam(S, 2)
#define NUM_OUT 13

static const int OUT_OFFSET[NUM_OUT] = {RFLYMAC_TIME,  RFLYMAC_POS,      RFLYMAC_VEL,   RFLYMAC_QUAT,
                                        RFLYMAC_EULER, RFLYMAC_RATE,     RFLYMAC_ACCEL, RFLYMAC_GYRO,
                                        RFLYMAC_MAG,   RFLYMAC_BARO_ALT, RFLYMAC_GPS,   RFLYMAC_MOTOR_W,
                                        RFLYMAC_ON_GROUND};
static const int OUT_WIDTH[NUM_OUT] = {1, 3, 3, 4, 3, 3, 3, 3, 3, 1, 3, 8, 1};

static char g_err[400];

static void fail(SimStruct *S, const char *what, const rflymac_client *c) {
    snprintf(g_err, sizeof(g_err), "CopterSim (Mac): %s: %s", what, c ? c->error : "");
    ssSetErrorStatus(S, g_err);
}

#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S) {
    if (!mxIsDouble(P_PORT(S)) || mxGetNumberOfElements(P_PORT(S)) != 1) {
        ssSetErrorStatus(S, "CopterSim (Mac): port must be a scalar");
        return;
    }
    if (!mxIsDouble(P_TS(S)) || mxGetNumberOfElements(P_TS(S)) != 1 || mxGetScalar(P_TS(S)) < 0.001) {
        ssSetErrorStatus(S, "CopterSim (Mac): Ts must be a scalar >= 0.001 s");
        return;
    }
    if (!mxIsChar(P_HOST(S))) {
        ssSetErrorStatus(S, "CopterSim (Mac): host must be a character vector, e.g. '127.0.0.1'");
    }
}

static void mdlInitializeSizes(SimStruct *S) {
    int i;
    ssSetNumSFcnParams(S, 3);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
    for (i = 0; i < 3; i++) ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, NUM_OUT)) return;
    for (i = 0; i < NUM_OUT; i++) ssSetOutputPortWidth(S, i, OUT_WIDTH[i]);

    ssSetNumSampleTimes(S, 1);
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, RFLYMAC_NUM_FIELDS);
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    ssSetNumPWork(S, 1);
    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, mxGetScalar(P_TS(S)));
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
    char host[256];
    double zero[RFLYMAC_NUM_MOTORS] = {0};
    real_T *state = (real_T *)ssGetDWork(S, 0);
    rflymac_client *c = (rflymac_client *)calloc(1, sizeof(rflymac_client));
    ssGetPWork(S)[0] = c;
    if (c == NULL) {
        ssSetErrorStatus(S, "CopterSim (Mac): out of memory");
        return;
    }
    c->fd = -1;
    if (mxGetString(P_HOST(S), host, sizeof(host)) != 0) {
        ssSetErrorStatus(S, "CopterSim (Mac): host name too long");
        return;
    }
    if (rflymac_open(c, host, (int)mxGetScalar(P_PORT(S)), 2000) != 0) {
        fail(S, "open", c);
        return;
    }
    if (rflymac_step(c, zero, 0, RFLYMAC_FLAG_RESET, state) != 0) {
        fail(S, "reset (start `coptersim --ext` first)", c);
    }
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    const real_T *state = (const real_T *)ssGetDWork(S, 0);
    int i, j;
    (void)tid;
    for (i = 0; i < NUM_OUT; i++) {
        real_T *y = ssGetOutputPortRealSignal(S, i);
        for (j = 0; j < OUT_WIDTH[i]; j++) y[j] = state[OUT_OFFSET[i] + j];
    }
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid) {
    rflymac_client *c = (rflymac_client *)ssGetPWork(S)[0];
    const real_T *u = ssGetInputPortRealSignal(S, 0);
    int w = ssGetInputPortWidth(S, 0), i;
    double motors[RFLYMAC_NUM_MOTORS] = {0};
    uint32_t nsteps = (uint32_t)floor(mxGetScalar(P_TS(S)) / 0.001 + 0.5);
    (void)tid;
    if (c == NULL) return;
    for (i = 0; i < w && i < RFLYMAC_NUM_MOTORS; i++) motors[i] = u[i];
    if (rflymac_step(c, motors, nsteps, 0, (double *)ssGetDWork(S, 0)) != 0) fail(S, "step", c);
}

static void mdlTerminate(SimStruct *S) {
    rflymac_client *c = (rflymac_client *)ssGetPWork(S)[0];
    if (c != NULL) {
        rflymac_close(c);
        free(c);
        ssGetPWork(S)[0] = NULL;
    }
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
