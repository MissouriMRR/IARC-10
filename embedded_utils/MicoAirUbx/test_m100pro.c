// Logs HGLRC M100-PRO GPS output to CSV for a given number of hours.
//
// Build:
//   gcc -O2 -c ublox.c -Dtime=ublox_utc && gcc -O2 -o test_m100pro test_m100pro.c ublox.o
// (-Dtime renames the `time` global in ublox.c so it doesn't clash with libc's time())
//
// Run:
//   ./test_m100pro <hours> [device=/dev/ttyUSB0] [out=m100pro_log.csv] [poll_s=1.0] [baud=auto]
//
// The M100-PRO uses a u-blox M10 chip, which differs from the MG-F10 setup in two ways:
//   - It ships with NMEA output for flight controllers, so we have to turn on UBX-NAV-PVT.
//   - M10 firmware dropped the legacy UBX-CFG-PRT/MSG/RATE/NAV5 messages that ublox_init()
//     sends, so this test configures it with UBX-CFG-VALSET instead.
// The config goes to the RAM layer only. A power cycle restores the factory settings, so
// the module still works with Betaflight/INAV/ArduPilot afterwards.
//
// Each poll writes one row with every GPS_t field plus UTC time, same columns as test_gps.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <time.h>
#include "ublox.h"

extern GPS_t gps_data;
extern UTC_TIME_t ublox_utc;
void ublox_decode(uint8_t data);

#define UBLOX_CFG_VALSET 0x8A

// M10 configuration keys (u-blox M10 SPG 5.10 interface description)
#define KEY_UART1OUTPROT_UBX      0x10740001u  // L
#define KEY_UART1OUTPROT_NMEA     0x10740002u  // L
#define KEY_MSGOUT_NAV_PVT_UART1  0x20910007u  // U1
#define KEY_RATE_MEAS             0x30210001u  // U2, ms
#define KEY_NAVSPG_DYNMODEL       0x20110021u  // E1, 7 = airborne <2g

static const int baud_list[] = { 115200, 38400, 9600, 57600, 230400 };

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static speed_t baud_const(int baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return 0;
    }
}

static int open_serial(const char *dev, int baud)
{
    speed_t sp = baud_const(baud);
    if (!sp) {
        fprintf(stderr, "unsupported baud %d\n", baud);
        return -1;
    }
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0)
        return -1;

    struct termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    tio.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// Listen for up to `secs` and return 1 if we see a UBX sync pair or an NMEA "$G" talker.
// At the wrong baud the UART delivers garbage that won't contain either pattern for long.
static int looks_like_gps(int fd, double secs)
{
    uint8_t buf[256];
    uint8_t prev = 0;
    int hits = 0;
    double end = now_s() + secs;

    while (now_s() < end) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { 0, 100000 };
        if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0)
            continue;
        ssize_t n = read(fd, buf, sizeof buf);
        for (ssize_t i = 0; i < n; i++) {
            if ((prev == UBLOX_SYNC1 && buf[i] == UBLOX_SYNC2) ||
                (prev == '$' && buf[i] == 'G'))
                hits++;
            prev = buf[i];
        }
        if (hits >= 3)
            return 1;
    }
    return 0;
}

static int put_key(uint8_t *p, uint32_t key, uint32_t val, int size)
{
    for (int i = 0; i < 4; i++)
        p[i] = (uint8_t)(key >> (8 * i));
    for (int i = 0; i < size; i++)
        p[4 + i] = (uint8_t)(val >> (8 * i));
    return 4 + size;
}

static int send_valset(int fd, uint16_t rate_ms)
{
    uint8_t msg[64];
    int n = 6;

    msg[n++] = 0x00;  // version
    msg[n++] = 0x01;  // layers: RAM only
    msg[n++] = 0x00;  // reserved
    msg[n++] = 0x00;
    n += put_key(msg + n, KEY_UART1OUTPROT_UBX, 1, 1);
    n += put_key(msg + n, KEY_UART1OUTPROT_NMEA, 0, 1);
    n += put_key(msg + n, KEY_MSGOUT_NAV_PVT_UART1, 1, 1);
    n += put_key(msg + n, KEY_RATE_MEAS, rate_ms, 2);
    n += put_key(msg + n, KEY_NAVSPG_DYNMODEL, AIRBORNE_2G, 1);

    uint16_t len = n - 6;
    msg[0] = UBLOX_SYNC1;
    msg[1] = UBLOX_SYNC2;
    msg[2] = UBLOX_CFG_CLASS;
    msg[3] = UBLOX_CFG_VALSET;
    msg[4] = (uint8_t)len;
    msg[5] = (uint8_t)(len >> 8);

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < n; i++) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    msg[n++] = ck_a;
    msg[n++] = ck_b;

    return write(fd, msg, n) == n ? 0 : -1;
}

// Feed bytes to the decoder until a NAV-PVT updates gps_data.time, or give up after `secs`.
static int wait_for_pvt(int fd, double secs)
{
    uint8_t buf[512];
    float t0 = gps_data.time;
    double end = now_s() + secs;

    while (now_s() < end) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { 0, 50000 };
        if (select(fd + 1, &rfds, NULL, NULL, &tv) > 0) {
            ssize_t n = read(fd, buf, sizeof buf);
            for (ssize_t i = 0; i < n; i++)
                ublox_decode(buf[i]);
        }
        if (gps_data.time != t0)
            return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <hours> [device] [out.csv] [poll_s] [baud|auto]\n", argv[0]);
        return 1;
    }
    double hours  = atof(argv[1]);
    const char *dev = argc > 2 ? argv[2] : "/dev/ttyUSB0";
    const char *out = argc > 3 ? argv[3] : "m100pro_log.csv";
    double poll_s = argc > 4 ? atof(argv[4]) : 1.0;
    int baud = (argc > 5 && strcmp(argv[5], "auto") != 0) ? atoi(argv[5]) : 0;

    int fd = -1;
    if (baud) {
        fd = open_serial(dev, baud);
        if (fd < 0) {
            perror(dev);
            return 1;
        }
    } else {
        for (size_t i = 0; i < sizeof baud_list / sizeof baud_list[0]; i++) {
            fd = open_serial(dev, baud_list[i]);
            if (fd < 0) {
                perror(dev);
                return 1;
            }
            printf("trying %d baud... ", baud_list[i]);
            fflush(stdout);
            if (looks_like_gps(fd, 2.0)) {
                baud = baud_list[i];
                printf("ok\n");
                break;
            }
            printf("no\n");
            close(fd);
            fd = -1;
        }
        if (fd < 0) {
            fprintf(stderr, "no GPS traffic on %s at any baud; check wiring (GPS TX -> adapter RX) and 5V\n", dev);
            return 1;
        }
    }

    // 125 ms (8 Hz) matches ublox_init().
    if (send_valset(fd, 125) < 0) {
        perror("write VALSET");
        return 1;
    }
    if (!wait_for_pvt(fd, 5.0)) {
        fprintf(stderr, "no NAV-PVT after VALSET at %d baud; the module may not be an M10, "
                        "or its UART RX isn't connected\n", baud);
        return 1;
    }
    printf("NAV-PVT streaming at %d baud\n", baud);

    FILE *f = fopen(out, "w");
    if (!f) {
        perror(out);
        return 1;
    }

    fprintf(f, "host_unix_s,utc,itow_s,lat_deg,lon_deg,alt_msl_m,velN,velE,velD,"
               "speed,heading_deg,hAcc_m,vAcc_m,sAcc,cAcc_deg,fix,numSV\n");

    double start = now_s();
    double end = start + hours * 3600.0;
    double next_poll = start;
    long rows = 0;
    uint8_t buf[512];

    while (now_s() < end) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = { 0, 50000 };
        if (select(fd + 1, &rfds, NULL, NULL, &tv) > 0) {
            ssize_t n = read(fd, buf, sizeof buf);
            for (ssize_t i = 0; i < n; i++)
                ublox_decode(buf[i]);
        }

        if (now_s() >= next_poll) {
            next_poll += poll_s;
            struct timespec wall;
            clock_gettime(CLOCK_REALTIME, &wall);
            GPS_t g = gps_data;
            fprintf(f, "%.3f,%04u-%02u-%02uT%02u:%02u:%02uZ,%.3f,%.7f,%.7f,%.3f,"
                       "%.3f,%.3f,%.3f,%.3f,%.5f,%.3f,%.3f,%.3f,%.5f,%u,%u\n",
                    wall.tv_sec + wall.tv_nsec * 1e-9,
                    ublox_utc.year, ublox_utc.month, ublox_utc.day,
                    ublox_utc.hour, ublox_utc.min, ublox_utc.sec,
                    g.time, g.latitude, g.longitude, g.altitude,
                    g.velN, g.velE, g.velD, g.speed, g.heading,
                    g.hAcc, g.vAcc, g.sAcc, g.cAcc, g.fixed, g.numSV);
            fflush(f);
            if (++rows % 60 == 0)
                printf("%ld rows, fix=%u sats=%u lat=%.7f lon=%.7f\n",
                       rows, g.fixed, g.numSV, g.latitude, g.longitude);
        }
    }

    fclose(f);
    close(fd);
    printf("done: %ld rows in %s\n", rows, out);
    return 0;
}
