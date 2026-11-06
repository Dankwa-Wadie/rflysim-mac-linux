/* rflymac_client.c: see rflymac_client.h */
#include "rflymac_client.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#define REQ_LEN 80
#define REP_LEN (8 + RFLYMAC_NUM_FIELDS * 8)
#define REQ_MAGIC 0x31434652u /* "RFC1" little-endian */
#define REP_MAGIC 0x31534652u /* "RFS1" */

static void put_u32(uint8_t *b, uint32_t v) {
    b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32(const uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}
/* Host is little-endian on all supported Macs (arm64, x86_64); memcpy keeps it alignment-safe. */
static void put_f64(uint8_t *b, double v) { memcpy(b, &v, 8); }
static double get_f64(const uint8_t *b) { double v; memcpy(&v, b, 8); return v; }

int rflymac_open(rflymac_client *c, const char *host, int port, int timeout_ms) {
    struct addrinfo hints, *res = NULL;
    char portstr[16];
    struct timeval tv;
    memset(c, 0, sizeof(*c));
    c->fd = -1;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    snprintf(portstr, sizeof(portstr), "%d", port);
    if (getaddrinfo(host, portstr, &hints, &res) != 0 || res == NULL) {
        snprintf(c->error, sizeof(c->error), "cannot resolve host '%s'", host);
        return -1;
    }
    c->fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (c->fd < 0 || connect(c->fd, res->ai_addr, res->ai_addrlen) != 0) {
        snprintf(c->error, sizeof(c->error), "socket/connect failed: %s", strerror(errno));
        freeaddrinfo(res);
        rflymac_close(c);
        return -1;
    }
    freeaddrinfo(res);
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(c->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return 0;
}

int rflymac_step(rflymac_client *c, const double motors[RFLYMAC_NUM_MOTORS], uint32_t nsteps,
                 uint32_t flags, double out[RFLYMAC_NUM_FIELDS]) {
    uint8_t req[REQ_LEN], rep[REP_LEN + 16];
    int i, attempt;
    if (c->fd < 0) {
        snprintf(c->error, sizeof(c->error), "client not open");
        return -1;
    }
    c->seq++;
    put_u32(req, REQ_MAGIC);
    put_u32(req + 4, c->seq);
    put_u32(req + 8, flags);
    put_u32(req + 12, nsteps);
    for (i = 0; i < RFLYMAC_NUM_MOTORS; i++) put_f64(req + 16 + 8 * i, motors ? motors[i] : 0.0);
    if (send(c->fd, req, REQ_LEN, 0) != REQ_LEN) {
        snprintf(c->error, sizeof(c->error), "send failed: %s", strerror(errno));
        return -1;
    }
    /* Skip stale replies (older sequence numbers) left over from a timed-out exchange. */
    for (attempt = 0; attempt < 16; attempt++) {
        ssize_t n = recv(c->fd, rep, sizeof(rep), 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                snprintf(c->error, sizeof(c->error),
                         "no reply from coptersim (is `coptersim --ext` running on that port?)");
            else if (errno == ECONNREFUSED)
                snprintf(c->error, sizeof(c->error),
                         "connection refused: start `coptersim --ext` first");
            else
                snprintf(c->error, sizeof(c->error), "recv failed: %s", strerror(errno));
            return -1;
        }
        if (n != REP_LEN || get_u32(rep) != REP_MAGIC) continue;
        if (get_u32(rep + 4) != c->seq) continue;
        for (i = 0; i < RFLYMAC_NUM_FIELDS; i++) out[i] = get_f64(rep + 8 + 8 * i);
        return 0;
    }
    snprintf(c->error, sizeof(c->error), "too many unexpected packets");
    return -1;
}

void rflymac_close(rflymac_client *c) {
    if (c->fd >= 0) close(c->fd);
    c->fd = -1;
}
