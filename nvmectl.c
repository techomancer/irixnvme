/*
 * nvmectl.c - NVMe controller runtime control utility for IRIX
 *
 * Controls per-controller NVMe features via the /hw/nvme/ctrl<N>
 * character device exposed by the nvme kernel driver.
 *
 * Build: cc nvmectl.c -o nvmectl
 *
 * Usage:
 *   nvmectl <N> get-vwc          -- print current write cache state
 *   nvmectl <N> set-vwc 0        -- disable volatile write cache
 *   nvmectl <N> set-vwc 1        -- enable volatile write cache
 *   nvmectl <N> get-fua          -- print current FUA state
 *   nvmectl <N> set-fua 0        -- disable Force Unit Access
 *   nvmectl <N> set-fua 1        -- enable Force Unit Access
 *
 * <N> is the NVMe adapter number (usually 2 on IP32, matching dks<N>d0).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "nvme.h"

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s <adapter> get-vwc\n"
            "       %s <adapter> set-vwc 0|1\n"
            "       %s <adapter> get-fua\n"
            "       %s <adapter> set-fua 0|1\n"
            "\n"
            "  adapter   NVMe adapter number (e.g. 2 for dks2d0)\n"
            "  get-vwc   print current volatile write cache state\n"
            "  set-vwc   0 = disable write cache, 1 = enable\n"
            "  get-fua   print current Force Unit Access state\n"
            "  set-fua   0 = disable FUA, 1 = enable\n",
            prog, prog, prog, prog);
    exit(1);
}

static int do_get(int fd, int adap, unsigned long ioc, const char *label)
{
    int val = -1;
    if (ioctl(fd, ioc, &val) != 0) {
        perror("nvmectl: ioctl");
        return 1;
    }
    printf("adapter %d: %s %s\n", adap, label, val ? "enabled" : "disabled");
    return 0;
}

static int do_set(int fd, int adap, unsigned long ioc, const char *label, int val)
{
    if (ioctl(fd, ioc, &val) != 0) {
        perror("nvmectl: ioctl");
        return 1;
    }
    printf("adapter %d: %s %s\n", adap, label, val ? "enabled" : "disabled");
    return 0;
}

int main(int argc, char *argv[])
{
    char devpath[64];
    int fd, adap, val, rc;
    const char *cmd;

    if (argc < 3)
        usage(argv[0]);

    adap = atoi(argv[1]);
    cmd  = argv[2];

    snprintf(devpath, sizeof(devpath), "/hw/nvme/ctrl%d", adap);

    fd = open(devpath, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "nvmectl: cannot open %s: %s\n",
                devpath, strerror(errno));
        fprintf(stderr, "         (driver loaded? ioconfig run?)\n");
        return 1;
    }

    if (strcmp(cmd, "get-vwc") == 0) {
        rc = do_get(fd, adap, NVME_IOC_GET_VWC, "volatile write cache");

    } else if (strcmp(cmd, "set-vwc") == 0) {
        if (argc < 4) usage(argv[0]);
        val = atoi(argv[3]);
        if (val != 0 && val != 1) {
            fprintf(stderr, "nvmectl: value must be 0 or 1\n");
            close(fd); return 1;
        }
        rc = do_set(fd, adap, NVME_IOC_SET_VWC, "volatile write cache", val);

    } else if (strcmp(cmd, "get-fua") == 0) {
        rc = do_get(fd, adap, NVME_IOC_GET_FUA, "force unit access");

    } else if (strcmp(cmd, "set-fua") == 0) {
        if (argc < 4) usage(argv[0]);
        val = atoi(argv[3]);
        if (val != 0 && val != 1) {
            fprintf(stderr, "nvmectl: value must be 0 or 1\n");
            close(fd); return 1;
        }
        rc = do_set(fd, adap, NVME_IOC_SET_FUA, "force unit access", val);

    } else {
        fprintf(stderr, "nvmectl: unknown command '%s'\n", cmd);
        usage(argv[0]);
        rc = 1;
    }

    close(fd);
    return rc;
}
