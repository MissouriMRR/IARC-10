// Logs MG-F10 GPS output to CSV for a given number of hours.
//
// Build:
//   gcc -O2 -c ublox.c -Dtime=ublox_utc && gcc -O2 -o test_gps test_gps.c ublox.o
// (-Dtime renames the `time` global in ublox.c so it doesn't clash with libc's time())
//
// Run:
//   ./test_gps <hours> [device=/dev/ttyUSB0] [out=gps_log.csv] [poll_s=1.0]
//
// The MG-F10 ships configured for UBX-NAV-PVT at 115200 baud, so this does not
// call ublox_init(). Each poll writes one row with every GPS_t field plus UTC time.

#include <stdio.h>
#include <stdlib.h>
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

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int open_serial(const char *dev)
{
    int fd = open(dev, O_RDONLY | O_NOCTTY);
    if (fd < 0)
        return -1;

    struct termios tio;
    tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &tio);
    tcflush(fd, TCIFLUSH);
    return fd;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <hours> [device] [out.csv] [poll_s]\n", argv[0]);
        return 1;
    }
    double hours  = atof(argv[1]);
    const char *dev = argc > 2 ? argv[2] : "/dev/ttyUSB0";
    const char *out = argc > 3 ? argv[3] : "gps_log.csv";
    double poll_s = argc > 4 ? atof(argv[4]) : 1.0;

    int fd = open_serial(dev);
    if (fd < 0) {
        perror(dev);
        return 1;
    }
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
