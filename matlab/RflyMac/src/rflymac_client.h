/* rflymac_client.h: UDP client for coptersim's external-controller lockstep mode
 * (RflyMac protocol v1, see crates/coptersim/src/ext.rs). Plain C99 + POSIX sockets,
 * shared by the Simulink S-function and the MATLAB MEX function. */
#ifndef RFLYMAC_CLIENT_H
#define RFLYMAC_CLIENT_H

#include <stdint.h>

#define RFLYMAC_NUM_MOTORS 8
#define RFLYMAC_NUM_FIELDS 39
#define RFLYMAC_DEFAULT_PORT 20100
#define RFLYMAC_FLAG_RESET 1u

/* Indices into the reply data vector. */
enum {
    RFLYMAC_TIME = 0,
    RFLYMAC_POS = 1,       /* 3: NED (m) */
    RFLYMAC_VEL = 4,       /* 3: NED (m/s) */
    RFLYMAC_QUAT = 7,      /* 4: w x y z */
    RFLYMAC_EULER = 11,    /* 3: roll pitch yaw (rad) */
    RFLYMAC_RATE = 14,     /* 3: p q r (rad/s) */
    RFLYMAC_ACCEL = 17,    /* 3: specific force, body (m/s^2) */
    RFLYMAC_GYRO = 20,     /* 3: rad/s */
    RFLYMAC_MAG = 23,      /* 3: gauss */
    RFLYMAC_BARO_ALT = 26, /* 1: m AMSL */
    RFLYMAC_GPS = 27,      /* 3: lat lon (deg), alt (m) */
    RFLYMAC_MOTOR_W = 30,  /* 8: rad/s */
    RFLYMAC_ON_GROUND = 38 /* 1 */
};

typedef struct {
    int fd;
    uint32_t seq;
    char error[256];
} rflymac_client;

/* Open a UDP socket to coptersim. timeout_ms bounds every exchange. Returns 0 on success. */
int rflymac_open(rflymac_client *c, const char *host, int port, int timeout_ms);

/* Send motors (0..1, PX4 order) + advance nsteps x 1 ms; receive state into out[39].
 * flags: RFLYMAC_FLAG_RESET to restart the simulation first. Returns 0 on success;
 * on failure c->error explains (e.g. coptersim not running). */
int rflymac_step(rflymac_client *c, const double motors[RFLYMAC_NUM_MOTORS], uint32_t nsteps,
                 uint32_t flags, double out[RFLYMAC_NUM_FIELDS]);

void rflymac_close(rflymac_client *c);

#endif
