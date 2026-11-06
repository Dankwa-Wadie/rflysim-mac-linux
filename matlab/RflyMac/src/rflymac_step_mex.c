/* rflymac_step_mex.c: MATLAB function `rflymac_step` for scripting coptersim without Simulink.
 *
 *   y = rflymac_step('reset')            % restart simulation, return initial state (39x1)
 *   y = rflymac_step(motors, nsteps)     % apply motors (1..8 values, 0..1), advance nsteps ms
 *   rflymac_step('open', host, port)     % choose server (default '127.0.0.1', 20100)
 *   rflymac_step('close')
 *
 * Field layout of y: see rflymac_fields.m. Build: install_rflymac.m */
#include <string.h>
#include "mex.h"
#include "rflymac_client.h"

static rflymac_client g_c;
static int g_open = 0;
static char g_host[256] = "127.0.0.1";
static int g_port = RFLYMAC_DEFAULT_PORT;

static void close_client(void) {
    if (g_open) rflymac_close(&g_c);
    g_open = 0;
}

static void ensure_open(void) {
    if (g_open) return;
    if (rflymac_open(&g_c, g_host, g_port, 2000) != 0)
        mexErrMsgIdAndTxt("rflymac:open", "rflymac_step: %s", g_c.error);
    g_open = 1;
    mexAtExit(close_client);
}

static void do_step(const double *motors, int n, uint32_t nsteps, uint32_t flags, mxArray **out) {
    double m[RFLYMAC_NUM_MOTORS] = {0}, y[RFLYMAC_NUM_FIELDS];
    int i;
    for (i = 0; i < n && i < RFLYMAC_NUM_MOTORS; i++) m[i] = motors[i];
    ensure_open();
    if (rflymac_step(&g_c, m, nsteps, flags, y) != 0) {
        close_client();
        mexErrMsgIdAndTxt("rflymac:step", "rflymac_step: %s", g_c.error);
    }
    *out = mxCreateDoubleMatrix(RFLYMAC_NUM_FIELDS, 1, mxREAL);
    memcpy(mxGetPr(*out), y, sizeof(y));
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    (void)nlhs;
    if (nrhs >= 1 && mxIsChar(prhs[0])) {
        char cmd[16];
        mxGetString(prhs[0], cmd, sizeof(cmd));
        if (strcmp(cmd, "reset") == 0) {
            do_step(NULL, 0, 0, RFLYMAC_FLAG_RESET, &plhs[0]);
        } else if (strcmp(cmd, "open") == 0) {
            close_client();
            if (nrhs >= 2) mxGetString(prhs[1], g_host, sizeof(g_host));
            if (nrhs >= 3) g_port = (int)mxGetScalar(prhs[2]);
            ensure_open();
        } else if (strcmp(cmd, "close") == 0) {
            close_client();
        } else {
            mexErrMsgIdAndTxt("rflymac:usage", "unknown command '%s'", cmd);
        }
        return;
    }
    if (nrhs != 2 || !mxIsDouble(prhs[0]) || !mxIsDouble(prhs[1]))
        mexErrMsgIdAndTxt("rflymac:usage", "usage: y = rflymac_step(motors, nsteps)");
    {
        double ns = mxGetScalar(prhs[1]);
        if (ns < 0 || ns > 100000) mexErrMsgIdAndTxt("rflymac:usage", "nsteps must be 0..100000");
        do_step(mxGetPr(prhs[0]), (int)mxGetNumberOfElements(prhs[0]), (uint32_t)ns, 0, &plhs[0]);
    }
}
