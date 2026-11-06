/* Integration test for rflymac_client.c and the reference controller (C twin of
 * rflymac_attitude_ctrl.m). Needs `coptersim --ext` running on UDP 20100.
 * Build: clang -O2 -I../src ../src/rflymac_client.c test_client.c -o test_client -lm */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rflymac_client.h"

#define DT 0.004 /* controller period: 4 ms = 4 physics steps */

/* ---- Reference controller: keep in sync with rflymac_attitude_ctrl.m ---- */
typedef struct { double iz, ir[3]; } ctrl_state;
static const double HOVER = 0.608;
static const double KP_Z = 1.0, VZ_MAX = 2.0, KP_VZ = 0.05, KI_VZ = 0.02;
static const double KP_ANG[3] = {5.0, 5.0, 3.0};
static const double KP_RATE[3] = {0.03, 0.03, 0.2};
static const double KI_RATE[3] = {0.02, 0.02, 0.05};

static double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

static void controller(ctrl_state *s, const double sp[4], const double *y, double m[8]) {
    const double *eul = y + RFLYMAC_EULER, *rate = y + RFLYMAC_RATE;
    double ez = sp[3] - y[RFLYMAC_POS + 2], vz = y[RFLYMAC_VEL + 2];
    double vz_sp = clampd(KP_Z * ez, -VZ_MAX, VZ_MAX); /* NED: negative = climb */
    double thr, tau[3];
    int i;
    s->iz = clampd(s->iz + KI_VZ * (vz - vz_sp) * DT, -0.1, 0.1);
    thr = HOVER + KP_VZ * (vz - vz_sp) + s->iz;
    thr /= fmax(cos(eul[0]) * cos(eul[1]), 0.5);
    thr = clampd(thr, 0.05, 0.95);
    for (i = 0; i < 3; i++) {
        double e = sp[i] - eul[i];
        if (i == 2) e = atan2(sin(e), cos(e));
        double rate_sp = KP_ANG[i] * e;
        double er = rate_sp - rate[i];
        s->ir[i] = clampd(s->ir[i] + KI_RATE[i] * er * DT, -0.1, 0.1);
        tau[i] = KP_RATE[i] * er + s->ir[i];
    }
    /* PX4 quad-x: 1 FR(CCW) 2 RL(CCW) 3 FL(CW) 4 RR(CW) */
    m[0] = thr - tau[0] + tau[1] + tau[2];
    m[1] = thr + tau[0] - tau[1] + tau[2];
    m[2] = thr + tau[0] + tau[1] - tau[2];
    m[3] = thr - tau[0] - tau[1] - tau[2];
    for (i = 0; i < 4; i++) m[i] = clampd(m[i], 0.0, 1.0);
    for (i = 4; i < 8; i++) m[i] = 0.0;
}
/* ------------------------------------------------------------------------ */

static int fails = 0;
static void check(int ok, const char *what, double got) {
    printf("%s: %s (got %.3f)\n", ok ? "PASS" : "FAIL", what, got);
    if (!ok) fails++;
}

int main(void) {
    rflymac_client c;
    double y[RFLYMAC_NUM_FIELDS], m[8] = {0}, sp[4];
    ctrl_state cs;
    int k;
    double z_at_5 = 0, roll_at_7 = 0, yaw_at_13 = 0, maxtilt_hover = 0;

    if (rflymac_open(&c, "127.0.0.1", RFLYMAC_DEFAULT_PORT, 1000) != 0) {
        printf("FAIL: open: %s\n", c.error);
        return 1;
    }
    if (rflymac_step(&c, m, 0, RFLYMAC_FLAG_RESET, y) != 0) {
        printf("FAIL: reset: %s\n", c.error);
        return 1;
    }
    check(y[RFLYMAC_TIME] == 0.0 && y[RFLYMAC_ON_GROUND] == 1.0, "reset -> t=0, on ground", y[0]);
    check(fabs(y[RFLYMAC_ACCEL + 2] + 9.8) < 0.2, "accelerometer reads -g at rest", y[RFLYMAC_ACCEL + 2]);

    memset(&cs, 0, sizeof(cs));
    for (k = 0; k < 15 * 250; k++) {
        double t = y[RFLYMAC_TIME];
        sp[0] = (t >= 5.0 && t < 7.0) ? 0.17 : 0.0; /* roll step ~10 deg */
        sp[1] = 0.0;
        sp[2] = (t >= 9.0) ? 0.5 : 0.0;            /* yaw step */
        sp[3] = (t >= 0.5) ? -5.0 : 0.0;           /* climb to 5 m */
        controller(&cs, sp, y, m);
        if (rflymac_step(&c, m, 4, 0, y) != 0) {
            printf("FAIL: step: %s\n", c.error);
            return 1;
        }
        t = y[RFLYMAC_TIME];
        if (fabs(t - 4.9) < 1e-9) z_at_5 = y[RFLYMAC_POS + 2];
        if (fabs(t - 6.9) < 1e-9) roll_at_7 = y[RFLYMAC_EULER];
        if (fabs(t - 13.0) < 1e-9) yaw_at_13 = y[RFLYMAC_EULER + 2];
        if (t > 3.0 && t < 5.0) {
            double tilt = fmax(fabs(y[RFLYMAC_EULER]), fabs(y[RFLYMAC_EULER + 1]));
            if (tilt > maxtilt_hover) maxtilt_hover = tilt;
        }
        if (k % 250 == 249)
            printf("  t=%5.1f z=%6.2f rpy=(%6.1f %6.1f %6.1f) deg m=%.3f %.3f %.3f %.3f\n", t,
                   y[RFLYMAC_POS + 2], y[RFLYMAC_EULER] * 57.2958, y[RFLYMAC_EULER + 1] * 57.2958,
                   y[RFLYMAC_EULER + 2] * 57.2958, m[0], m[1], m[2], m[3]);
    }
    check(fabs(y[RFLYMAC_TIME] - 15.0) < 1e-9, "lockstep: exactly 15.000 s simulated", y[RFLYMAC_TIME]);
    check(fabs(z_at_5 + 5.0) < 0.3, "altitude 5 m reached by t=4.9 s", -z_at_5);
    check(maxtilt_hover < 0.02, "level hover (tilt < 1.1 deg)", maxtilt_hover * 57.2958);
    check(fabs(roll_at_7 - 0.17) < 0.03, "roll step tracked (~9.7 deg)", roll_at_7 * 57.2958);
    check(fabs(yaw_at_13 - 0.5) < 0.05, "yaw step tracked (~28.6 deg)", yaw_at_13 * 57.2958);
    check(fabs(y[RFLYMAC_POS + 2] + 5.0) < 0.5, "altitude held at end", -y[RFLYMAC_POS + 2]);
    rflymac_close(&c);
    printf(fails ? "SOME CHECKS FAILED\n" : "ALL CLIENT CHECKS PASSED\n");
    return fails ? 1 : 0;
}
