/*
 * nvmetest.c - IRIX SCSI test program for NVMe driver
 *
 * Tests various SCSI operations on the NVMe controller via hardware graph
 *
 * Default path: /hw/scsi_ctlr/2/target/0/lun/0/scsi
 * (controller 2, target 0, lun 0)
 *
 * Compile: cc -o nvmetest nvmetest.c
 * Run as root: ./nvmetest [options]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/dkio.h>
#include <dslib.h>

/* IRIX hardware graph path for controller 2, target 0, lun 0, find some way to find it! */
#ifdef IP35
#define DEFAULT_SCSI_PATH "/hw/scsi_ctlr/5/target/0/lun/0/scsi"
#else
#define DEFAULT_SCSI_PATH "/hw/scsi_ctlr/2/target/0/lun/0/scsi"
#endif
#define BLOCK_SIZE 512

/* Test functions */
void test_inquiry(int fd);
void test_read_capacity(int fd);
void test_read_single(int fd, unsigned int lba);
void test_read_sequential(int fd, unsigned int start_lba, unsigned int num_blocks);
void test_write_single(int fd, unsigned int lba);
void test_write_sequential(int fd, unsigned int start_lba, unsigned int num_blocks);
void test_write_read(int fd, unsigned int lba);
void test_random_write_read(int fd, unsigned int max_lba, unsigned int num_iterations);
void dump_hex(unsigned char *data, size_t len, size_t offset);

/* Random number generator state (simple LCG) */
static unsigned int rng_seed = 0;

void usage(const char *progname)
{
    fprintf(stderr, "Usage: %s [options]\n", progname);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  -d PATH        Device path (default: %s)\n", DEFAULT_SCSI_PATH);
    fprintf(stderr, "  -i             Test INQUIRY\n");
    fprintf(stderr, "  -c             Test READ CAPACITY\n");
    fprintf(stderr, "  -r LBA         Read single block at LBA\n");
    fprintf(stderr, "  -s LBA COUNT   Read COUNT blocks starting at LBA\n");
    fprintf(stderr, "  -W LBA         Write single block at LBA\n");
    fprintf(stderr, "  -S LBA COUNT   Write COUNT blocks starting at LBA\n");
    fprintf(stderr, "  -w LBA         Write/Read test at LBA\n");
    fprintf(stderr, "  -R ITERATIONS  Random write/read stress test (default: 100 iterations)\n");
    fprintf(stderr, "  -a             Run all tests\n");
    fprintf(stderr, "  -h             Show this help\n");
    fprintf(stderr, "\nExamples:\n");
    fprintf(stderr, "  %s -a                           # Run all basic tests\n", progname);
    fprintf(stderr, "  %s -s 0 8192                    # Large read (4MB, tests PRP chaining)\n", progname);
    fprintf(stderr, "  %s -R 1000                      # Random write/read stress test\n", progname);
    fprintf(stderr, "  %s -d /hw/scsi_ctlr/1/... -i   # Test different controller\n", progname);
    exit(1);
}

int main(int argc, char *argv[])
{
    int fd;
    int opt;
    int test_all = 0;
    int test_inquiry_flag = 0;
    int test_capacity_flag = 0;
    int test_read_flag = 0;
    int test_seq_flag = 0;
    int test_write_single_flag = 0;
    int test_write_seq_flag = 0;
    int test_write_flag = 0;
    int test_random_flag = 0;
    unsigned int lba = 0;
    unsigned int lba_write = 0;
    unsigned int lba_write_seq = 0;
    unsigned int count = 1;
    unsigned int count_write = 1;
    unsigned int random_iterations = 100;
    const char *device_path = DEFAULT_SCSI_PATH;

    if (argc < 2) {
        usage(argv[0]);
    }

    /* Initialize RNG with current time */
    rng_seed = (unsigned int)time(NULL);

    /* Parse options */
    while ((opt = getopt(argc, argv, "d:icr:s:W:S:w:R:ah")) != -1) {
        switch (opt) {
        case 'd':
            device_path = optarg;
            break;
        case 'i':
            test_inquiry_flag = 1;
            break;
        case 'c':
            test_capacity_flag = 1;
            break;
        case 'r':
            test_read_flag = 1;
            lba = strtoul(optarg, NULL, 0);
            break;
        case 's':
            test_seq_flag = 1;
            lba = strtoul(optarg, NULL, 0);
            if (optind < argc) {
                count = strtoul(argv[optind], NULL, 0);
                optind++;
            }
            break;
        case 'W':
            test_write_single_flag = 1;
            lba_write = strtoul(optarg, NULL, 0);
            break;
        case 'S':
            test_write_seq_flag = 1;
            lba_write_seq = strtoul(optarg, NULL, 0);
            if (optind < argc) {
                count_write = strtoul(argv[optind], NULL, 0);
                optind++;
            }
            break;
        case 'w':
            test_write_flag = 1;
            lba = strtoul(optarg, NULL, 0);
            break;
        case 'R':
            test_random_flag = 1;
            random_iterations = strtoul(optarg, NULL, 0);
            if (random_iterations == 0) random_iterations = 100;
            break;
        case 'a':
            test_all = 1;
            break;
        case 'h':
        default:
            usage(argv[0]);
        }
    }

    /* Open SCSI device */
    printf("Opening %s...\n", device_path);
    fd = open(device_path, O_RDWR);
    if (fd < 0) {
        perror("open");
        fprintf(stderr, "Failed to open %s\n", device_path);
        fprintf(stderr, "Make sure you are root and the driver is loaded\n");
        fprintf(stderr, "Check if device exists: ls -l %s\n", device_path);
        exit(1);
    }

    printf("Device opened successfully (fd=%d)\n\n", fd);

    /* Run tests */
    if (test_all || test_inquiry_flag) {
        test_inquiry(fd);
    }

    if (test_all || test_capacity_flag) {
        test_read_capacity(fd);
    }

    if (test_all || test_read_flag) {
        test_read_single(fd, lba);
    }

    if (test_seq_flag) {
        test_read_sequential(fd, lba, count);
    }

    if (test_write_single_flag) {
        test_write_single(fd, lba_write);
    }

    if (test_write_seq_flag) {
        test_write_sequential(fd, lba_write_seq, count_write);
    }

    if (test_write_flag) {
        test_write_read(fd, lba);
    }

    if (test_random_flag) {
        test_random_write_read(fd, 10000, random_iterations);
    }

    close(fd);
    printf("\nAll tests completed.\n");
    return 0;
}

void test_inquiry(int fd)
{
    unsigned char inq_data[36];
    char vendor[9], product[17], revision[5];
    dsreq_t req;
    unsigned char cdb[6];

    printf("=== INQUIRY Test ===\n");

    /* Send INQUIRY command via dsreq */
    cdb[0] = 0x12; cdb[1] = 0; cdb[2] = 0; cdb[3] = 0; cdb[4] = 36; cdb[5] = 0;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_READ | DSRQ_SENSE;
    req.ds_time = 5000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 6;
    req.ds_databuf = (caddr_t)inq_data;
    req.ds_datalen = 36;
    bzero(inq_data, 36);

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (INQUIRY)");
        return;
    }

    if (req.ds_status != 0) {
        printf("INQUIRY failed: status=0x%02x\n", req.ds_status);
        return;
    }

    /* Parse INQUIRY data */
    strncpy(vendor, (char *)&inq_data[8], 8);
    vendor[8] = '\0';
    strncpy(product, (char *)&inq_data[16], 16);
    product[16] = '\0';
    strncpy(revision, (char *)&inq_data[32], 4);
    revision[4] = '\0';

    printf("Vendor:   '%s'\n", vendor);
    printf("Product:  '%s'\n", product);
    printf("Revision: '%s'\n", revision);
    printf("Device Type: 0x%02x\n", inq_data[0] & 0x1F);
    printf("\n");
}

void test_read_capacity(int fd)
{
    unsigned char cap_data[8];
    unsigned int max_lba, block_size;
    dsreq_t req;
    unsigned char cdb[10];

    printf("=== READ CAPACITY Test ===\n");

    /* Send READ CAPACITY(10) command */
    cdb[0] = 0x25;
    cdb[1] = 0; cdb[2] = 0; cdb[3] = 0; cdb[4] = 0;
    cdb[5] = 0; cdb[6] = 0; cdb[7] = 0; cdb[8] = 0; cdb[9] = 0;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_READ | DSRQ_SENSE;
    req.ds_time = 5000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)cap_data;
    req.ds_datalen = 8;
    bzero(cap_data, 8);

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (READ CAPACITY)");
        return;
    }

    if (req.ds_status != 0) {
        printf("READ CAPACITY failed: status=0x%02x\n", req.ds_status);
        return;
    }

    /* Parse capacity data (big-endian) */
    max_lba = (cap_data[0] << 24) | (cap_data[1] << 16) |
              (cap_data[2] << 8) | cap_data[3];
    block_size = (cap_data[4] << 24) | (cap_data[5] << 16) |
                 (cap_data[6] << 8) | cap_data[7];

    printf("Max LBA: %u (0x%08x)\n", max_lba, max_lba);
    printf("Block Size: %u bytes\n", block_size);
    printf("Capacity: %.2f GB\n", (max_lba + 1) * (double)block_size / (1024*1024*1024));
    printf("\n");
}

void test_read_single(int fd, unsigned int lba)
{
    unsigned char *buffer;
    dsreq_t req;
    unsigned char cdb[10];

    printf("=== Single Block Read Test (LBA %u) ===\n", lba);

    buffer = malloc(BLOCK_SIZE);
    if (!buffer) {
        perror("malloc");
        return;
    }

    /* Send READ(10) command */
    cdb[0] = 0x28; cdb[1] = 0;
    cdb[6] = 0; cdb[7] = 0; cdb[8] = 1; cdb[9] = 0;

    /* Fill in LBA (big-endian) */
    cdb[2] = (lba >> 24) & 0xFF;
    cdb[3] = (lba >> 16) & 0xFF;
    cdb[4] = (lba >> 8) & 0xFF;
    cdb[5] = lba & 0xFF;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_READ | DSRQ_SENSE;
    req.ds_time = 10000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)buffer;
    req.ds_datalen = BLOCK_SIZE;
    bzero(buffer, BLOCK_SIZE);

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (READ)");
        free(buffer);
        return;
    }

    if (req.ds_status != 0) {
        printf("READ failed: status=0x%02x\n", req.ds_status);
        free(buffer);
        return;
    }

    printf("Read successful, %d bytes transferred\n", req.ds_datasent);
    dump_hex(buffer, 64, lba * BLOCK_SIZE);

    free(buffer);
    printf("\n");
}

void test_read_sequential(int fd, unsigned int start_lba, unsigned int num_blocks)
{
    unsigned char *buffer;
    size_t buffer_size;
    dsreq_t req;
    unsigned char cdb[10];

    buffer_size = num_blocks * BLOCK_SIZE;

    printf("=== Sequential Read Test (LBA %u, %u blocks = %lu bytes) ===\n",
           start_lba, num_blocks, (unsigned long)buffer_size);

    buffer = malloc(buffer_size);
    if (!buffer) {
        perror("malloc");
        return;
    }

    /* Send READ(10) command */
    cdb[0] = 0x28; cdb[1] = 0;
    cdb[6] = 0; cdb[9] = 0;

    /* Fill in LBA (big-endian) */
    cdb[2] = (start_lba >> 24) & 0xFF;
    cdb[3] = (start_lba >> 16) & 0xFF;
    cdb[4] = (start_lba >> 8) & 0xFF;
    cdb[5] = start_lba & 0xFF;

    /* Fill in transfer length (big-endian) */
    cdb[7] = (num_blocks >> 8) & 0xFF;
    cdb[8] = num_blocks & 0xFF;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_READ | DSRQ_SENSE;
    req.ds_time = 30000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)buffer;
    req.ds_datalen = buffer_size;
    bzero(buffer, buffer_size);

    printf("Issuing READ command...\n");

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (READ)");
        free(buffer);
        return;
    }

    if (req.ds_status != 0) {
        printf("READ failed: status=0x%02x\n", req.ds_status);
        free(buffer);
        return;
    }

    printf("Read successful, %d bytes transferred\n", req.ds_datasent);
    dump_hex(buffer, 64, start_lba * BLOCK_SIZE);

    free(buffer);
    printf("\n");
}

void test_write_single(int fd, unsigned int lba)
{
    unsigned char *buffer;
    int i;
    dsreq_t req;
    unsigned char cdb[10];

    printf("=== Single Block Write Test (LBA %u) ===\n", lba);

    buffer = malloc(BLOCK_SIZE);
    if (!buffer) {
        perror("malloc");
        return;
    }

    /* Fill write buffer with a pattern */
    for (i = 0; i < BLOCK_SIZE; i++) {
        buffer[i] = (unsigned char)(0xBB ^ (i & 0xFF));
    }

    printf("Write pattern: First 64 bytes:\n");
    dump_hex(buffer, 64, 0);

    /* Send WRITE(10) command */
    cdb[0] = 0x2A; cdb[1] = 0;
    cdb[6] = 0; cdb[7] = 0; cdb[8] = 1; cdb[9] = 0;

    /* Fill in LBA (big-endian) */
    cdb[2] = (lba >> 24) & 0xFF;
    cdb[3] = (lba >> 16) & 0xFF;
    cdb[4] = (lba >> 8) & 0xFF;
    cdb[5] = lba & 0xFF;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_WRITE | DSRQ_SENSE;
    req.ds_time = 10000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)buffer;
    req.ds_datalen = BLOCK_SIZE;

    printf("Issuing WRITE command...\n");

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (WRITE)");
        free(buffer);
        return;
    }

    if (req.ds_status != 0) {
        printf("WRITE failed: status=0x%02x\n", req.ds_status);
        free(buffer);
        return;
    }

    printf("Write successful, %d bytes transferred\n", req.ds_datasent);

    free(buffer);
    printf("\n");
}

void test_write_sequential(int fd, unsigned int start_lba, unsigned int num_blocks)
{
    unsigned char *buffer;
    size_t buffer_size;
    size_t i;
    dsreq_t req;
    unsigned char cdb[10];

    buffer_size = num_blocks * BLOCK_SIZE;

    printf("=== Sequential Write Test (LBA %u, %u blocks = %lu bytes) ===\n",
           start_lba, num_blocks, (unsigned long)buffer_size);

    buffer = malloc(buffer_size);
    if (!buffer) {
        perror("malloc");
        return;
    }

    /* Fill write buffer with a pattern based on position */
    for (i = 0; i < buffer_size; i++) {
        buffer[i] = (unsigned char)(0xCC ^ ((i >> 8) & 0xFF) ^ (i & 0xFF));
    }

    printf("Write pattern: First 64 bytes:\n");
    dump_hex(buffer, 64, 0);

    /* Send WRITE(10) command */
    cdb[0] = 0x2A; cdb[1] = 0;
    cdb[6] = 0; cdb[9] = 0;

    /* Fill in LBA (big-endian) */
    cdb[2] = (start_lba >> 24) & 0xFF;
    cdb[3] = (start_lba >> 16) & 0xFF;
    cdb[4] = (start_lba >> 8) & 0xFF;
    cdb[5] = start_lba & 0xFF;

    /* Fill in transfer length (big-endian) */
    cdb[7] = (num_blocks >> 8) & 0xFF;
    cdb[8] = num_blocks & 0xFF;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_WRITE | DSRQ_SENSE;
    req.ds_time = 30000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)buffer;
    req.ds_datalen = buffer_size;

    printf("Issuing WRITE command...\n");

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (WRITE)");
        free(buffer);
        return;
    }

    if (req.ds_status != 0) {
        printf("WRITE failed: status=0x%02x\n", req.ds_status);
        free(buffer);
        return;
    }

    printf("Write successful, %d bytes transferred\n", req.ds_datasent);

    free(buffer);
    printf("\n");
}

void test_write_read(int fd, unsigned int lba)
{
    unsigned char *write_buf, *read_buf;
    int i;
    dsreq_t req;
    unsigned char cdb[10];

    printf("=== Write/Read Verification Test (LBA %u) ===\n", lba);

    write_buf = malloc(BLOCK_SIZE);
    read_buf = malloc(BLOCK_SIZE);
    if (!write_buf || !read_buf) {
        perror("malloc");
        if (write_buf) free(write_buf);
        if (read_buf) free(read_buf);
        return;
    }

    /* Fill write buffer with pattern */
    for (i = 0; i < BLOCK_SIZE; i++) {
        write_buf[i] = (unsigned char)(0xAA ^ i);
    }

    /* Send WRITE(10) command */
    cdb[0] = 0x2A; cdb[1] = 0;
    cdb[6] = 0; cdb[7] = 0; cdb[8] = 1; cdb[9] = 0;

    /* Fill in LBA (big-endian) */
    cdb[2] = (lba >> 24) & 0xFF;
    cdb[3] = (lba >> 16) & 0xFF;
    cdb[4] = (lba >> 8) & 0xFF;
    cdb[5] = lba & 0xFF;

    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_WRITE | DSRQ_SENSE;
    req.ds_time = 10000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)write_buf;
    req.ds_datalen = BLOCK_SIZE;

    printf("Writing test pattern...\n");

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (WRITE)");
        free(write_buf);
        free(read_buf);
        return;
    }

    if (req.ds_status != 0) {
        printf("WRITE failed: status=0x%02x\n", req.ds_status);
        free(write_buf);
        free(read_buf);
        return;
    }

    printf("Write successful, %d bytes transferred\n", req.ds_datasent);

    /* Now read it back */
    cdb[0] = 0x28; /* READ(10) */
    bzero(&req, sizeof(req));
    req.ds_flags = DSRQ_READ | DSRQ_SENSE;
    req.ds_time = 10000;
    req.ds_cmdbuf = (caddr_t)cdb;
    req.ds_cmdlen = 10;
    req.ds_databuf = (caddr_t)read_buf;
    req.ds_datalen = BLOCK_SIZE;
    bzero(read_buf, BLOCK_SIZE);

    printf("Reading back...\n");

    if (ioctl(fd, DS_ENTER, &req) < 0) {
        perror("DS_ENTER (READ)");
        free(write_buf);
        free(read_buf);
        return;
    }

    if (req.ds_status != 0) {
        printf("READ failed: status=0x%02x\n", req.ds_status);
        free(write_buf);
        free(read_buf);
        return;
    }

    printf("Read successful, %d bytes transferred\n", req.ds_datasent);

    /* Compare */
    if (memcmp(write_buf, read_buf, BLOCK_SIZE) == 0) {
        printf("VERIFY OK: Write and read data match!\n");
    } else {
        printf("VERIFY FAILED: Data mismatch!\n");
        for (i = 0; i < BLOCK_SIZE; i++) {
            if (write_buf[i] != read_buf[i]) {
                printf("  Offset %d: wrote 0x%02x, read 0x%02x\n",
                       i, write_buf[i], read_buf[i]);
                if (i > 10) {
                    printf("  (showing first 10 mismatches only)\n");
                    break;
                }
            }
        }
    }

    free(write_buf);
    free(read_buf);
    printf("\n");
}

void dump_hex(unsigned char *data, size_t len, size_t offset)
{
    size_t i;
    for (i = 0; i < len; i++) {
        if (i % 16 == 0) {
            printf("%08lx: ", (unsigned long)(offset + i));
        }
        printf("%02x ", data[i]);
        if (i % 16 == 15) {
            printf("\n");
        }
    }
    if (len % 16 != 0) {
        printf("\n");
    }
}

/* Simple LCG random number generator */
static unsigned int rand_lcg(void)
{
    rng_seed = rng_seed * 1103515245 + 12345;
    return rng_seed;
}

/* Random number in range [min, max] */
static unsigned int rand_range(unsigned int min, unsigned int max)
{
    if (max <= min) return min;
    return min + (rand_lcg() % (max - min + 1));
}

/* Alignment options: 4K page, 512B block, or random 4-byte alignment */
typedef enum {
    ALIGN_4K = 0,      /* 4096-byte alignment */
    ALIGN_512B = 1,    /* 512-byte alignment */
    ALIGN_RANDOM = 2   /* Random 4-byte alignment */
} alignment_t;

/* Get alignment in bytes */
static size_t get_alignment(alignment_t align_type)
{
    switch (align_type) {
    case ALIGN_4K:
        return 4096;
    case ALIGN_512B:
        return 512;
    case ALIGN_RANDOM:
        /* Random 4-byte alignment (4, 8, 12, 16, ... up to 512) */
        return (rand_range(1, 128) * 4);
    default:
        return 4;
    }
}

/* Allocate aligned buffer */
static void *alloc_aligned(size_t size, size_t alignment)
{
    void *ptr;
    void *aligned_ptr;
    size_t total_size;
    size_t addr;

    /* Allocate extra space for alignment + pointer storage */
    total_size = size + alignment + sizeof(void *);
    ptr = malloc(total_size);
    if (!ptr) return NULL;

    /* Align the pointer */
    addr = (size_t)ptr + sizeof(void *) + alignment - 1;
    addr = addr & ~(alignment - 1);
    aligned_ptr = (void *)addr;

    /* Store original pointer before aligned pointer for free() */
    *((void **)aligned_ptr - 1) = ptr;

    return aligned_ptr;
}

/* Free aligned buffer */
static void free_aligned(void *aligned_ptr)
{
    void *ptr;

    if (!aligned_ptr) return;
    /* Retrieve original pointer */
    ptr = *((void **)aligned_ptr - 1);
    free(ptr);
}

/* Fill buffer with random data and compute simple checksum */
static unsigned int fill_random_data(unsigned char *buf, size_t size, unsigned int seed)
{
    size_t i;
    unsigned int checksum = 0;
    unsigned int val;

    /* Use seed to initialize local RNG state */
    for (i = 0; i < size; i++) {
        seed = seed * 1103515245 + 12345;
        val = (seed >> 16) & 0xFF;
        buf[i] = (unsigned char)val;
        checksum += val;
    }

    return checksum;
}

/* Comprehensive random write/read test */
void test_random_write_read(int fd, unsigned int max_lba, unsigned int num_iterations)
{
    unsigned int iter;
    unsigned int write_lba, write_blocks;
    unsigned int read_lba, read_blocks;
    unsigned int max_blocks_per_transfer;
    size_t write_size, read_size;
    unsigned char *write_buf, *read_buf;
    alignment_t write_align, read_align;
    size_t write_alignment, read_alignment;
    dsreq_t req;
    unsigned char cdb[10];
    unsigned int write_seed, write_checksum, read_checksum;
    int num_reads;
    int read_idx;
    int errors = 0;
    int total_writes = 0;
    int total_reads = 0;
    size_t i;

    printf("=== Random Write/Read Stress Test ===\n");
    printf("Max LBA range: 0-%u\n", max_lba);
    printf("Iterations: %u\n", num_iterations);
    printf("\n");

    /*
     * Max transfer size: assume v.v_maxdmasz is around 256 pages = 1MB
     * Conservative limit: 128 pages = 512KB = 1024 blocks
     */
    max_blocks_per_transfer = 1024;

    for (iter = 0; iter < num_iterations; iter++) {
        printf("--- Iteration %u/%u ---\n", iter + 1, num_iterations);

        /* Generate random write parameters */
        write_lba = rand_range(0, max_lba - 1);
        write_blocks = rand_range(1, max_blocks_per_transfer);

        /* Ensure we don't exceed max_lba */
        if (write_lba + write_blocks > max_lba) {
            write_blocks = max_lba - write_lba;
        }

        write_size = write_blocks * BLOCK_SIZE;
        write_align = (alignment_t)rand_range(0, 2);
        write_alignment = get_alignment(write_align);

        printf("WRITE: LBA %u, blocks %u (%lu bytes), align %lu bytes\n",
               write_lba, write_blocks, (unsigned long)write_size,
               (unsigned long)write_alignment);

        /* Allocate aligned write buffer */
        write_buf = (unsigned char *)alloc_aligned(write_size, write_alignment);
        if (!write_buf) {
            printf("ERROR: Failed to allocate write buffer\n");
            errors++;
            continue;
        }

        /* Fill with random data */
        write_seed = rand_lcg();
        write_checksum = fill_random_data(write_buf, write_size, write_seed);

        /* Send WRITE(10) command */
        cdb[0] = 0x2A; /* WRITE(10) */
        cdb[1] = 0;
        cdb[2] = (write_lba >> 24) & 0xFF;
        cdb[3] = (write_lba >> 16) & 0xFF;
        cdb[4] = (write_lba >> 8) & 0xFF;
        cdb[5] = write_lba & 0xFF;
        cdb[6] = 0;
        cdb[7] = (write_blocks >> 8) & 0xFF;
        cdb[8] = write_blocks & 0xFF;
        cdb[9] = 0;

        bzero(&req, sizeof(req));
        req.ds_flags = DSRQ_WRITE | DSRQ_SENSE;
        req.ds_time = 30000;
        req.ds_cmdbuf = (caddr_t)cdb;
        req.ds_cmdlen = 10;
        req.ds_databuf = (caddr_t)write_buf;
        req.ds_datalen = write_size;

        if (ioctl(fd, DS_ENTER, &req) < 0) {
            perror("DS_ENTER (WRITE)");
            free_aligned(write_buf);
            errors++;
            continue;
        }

        if (req.ds_status != 0) {
            printf("ERROR: WRITE failed: status=0x%02x\n", req.ds_status);
            free_aligned(write_buf);
            errors++;
            continue;
        }

        total_writes++;
        printf("  Write OK (checksum=0x%08x)\n", write_checksum);

        /* Now perform multiple random reads that overlap or are contained within write range */
        num_reads = rand_range(2, 5);

        for (read_idx = 0; read_idx < num_reads; read_idx++) {
            /* Generate read that overlaps with written range */
            if (rand_range(0, 1) == 0) {
                /* Read fully contained within write range */
                if (write_blocks > 1) {
                    unsigned int offset_blocks = rand_range(0, write_blocks - 1);
                    unsigned int max_read_blocks = write_blocks - offset_blocks;
                    read_blocks = rand_range(1, max_read_blocks);
                    read_lba = write_lba + offset_blocks;
                } else {
                    read_blocks = 1;
                    read_lba = write_lba;
                }
            } else {
                /* Read overlapping with write range */
                if (write_lba > 0 && rand_range(0, 1) == 0) {
                    /* Start before write range */
                    read_lba = write_lba - rand_range(1, (write_lba < 10 ? write_lba : 10));
                    read_blocks = rand_range(1, write_lba - read_lba + write_blocks);
                } else {
                    /* Start within write range */
                    unsigned int offset = rand_range(0, write_blocks - 1);
                    read_lba = write_lba + offset;
                    read_blocks = rand_range(1, (max_lba - read_lba < 100 ? max_lba - read_lba : 100));
                }
            }

            /* Ensure we don't exceed max_lba */
            if (read_lba + read_blocks > max_lba) {
                read_blocks = max_lba - read_lba;
            }

            /* Limit read blocks to reasonable size */
            if (read_blocks > max_blocks_per_transfer) {
                read_blocks = max_blocks_per_transfer;
            }

            read_size = read_blocks * BLOCK_SIZE;
            read_align = (alignment_t)rand_range(0, 2);
            read_alignment = get_alignment(read_align);

            printf("  READ %d/%d: LBA %u, blocks %u (%lu bytes), align %lu bytes\n",
                   read_idx + 1, num_reads, read_lba, read_blocks,
                   (unsigned long)read_size, (unsigned long)read_alignment);

            /* Allocate aligned read buffer */
            read_buf = (unsigned char *)alloc_aligned(read_size, read_alignment);
            if (!read_buf) {
                printf("  ERROR: Failed to allocate read buffer\n");
                errors++;
                continue;
            }

            bzero(read_buf, read_size);

            /* Send READ(10) command */
            cdb[0] = 0x28; /* READ(10) */
            cdb[1] = 0;
            cdb[2] = (read_lba >> 24) & 0xFF;
            cdb[3] = (read_lba >> 16) & 0xFF;
            cdb[4] = (read_lba >> 8) & 0xFF;
            cdb[5] = read_lba & 0xFF;
            cdb[6] = 0;
            cdb[7] = (read_blocks >> 8) & 0xFF;
            cdb[8] = read_blocks & 0xFF;
            cdb[9] = 0;

            bzero(&req, sizeof(req));
            req.ds_flags = DSRQ_READ | DSRQ_SENSE;
            req.ds_time = 30000;
            req.ds_cmdbuf = (caddr_t)cdb;
            req.ds_cmdlen = 10;
            req.ds_databuf = (caddr_t)read_buf;
            req.ds_datalen = read_size;

            if (ioctl(fd, DS_ENTER, &req) < 0) {
                perror("DS_ENTER (READ)");
                free_aligned(read_buf);
                errors++;
                continue;
            }

            if (req.ds_status != 0) {
                printf("  ERROR: READ failed: status=0x%02x\n", req.ds_status);
                free_aligned(read_buf);
                errors++;
                continue;
            }

            total_reads++;

            /* Verify data for blocks that overlap with write range */
            if (read_lba < write_lba + write_blocks && read_lba + read_blocks > write_lba) {
                unsigned int overlap_start_lba;
                unsigned int overlap_end_lba;
                unsigned int overlap_blocks;
                size_t read_offset;
                size_t write_offset;
                size_t verify_size;
                unsigned int expected_checksum;
                int mismatch_count;

                /* Compute overlap range */
                overlap_start_lba = (read_lba > write_lba) ? read_lba : write_lba;
                overlap_end_lba = ((read_lba + read_blocks) < (write_lba + write_blocks)) ?
                                               (read_lba + read_blocks) : (write_lba + write_blocks);
                overlap_blocks = overlap_end_lba - overlap_start_lba;

                /* Offset into buffers */
                read_offset = (overlap_start_lba - read_lba) * BLOCK_SIZE;
                write_offset = (overlap_start_lba - write_lba) * BLOCK_SIZE;
                verify_size = overlap_blocks * BLOCK_SIZE;

                /* Compute checksum of overlapping read data */
                read_checksum = 0;
                for (i = 0; i < verify_size; i++) {
                    read_checksum += read_buf[read_offset + i];
                }

                /* Compute expected checksum from write data */
                expected_checksum = 0;
                for (i = 0; i < verify_size; i++) {
                    expected_checksum += write_buf[write_offset + i];
                }

                /* Compare */
                if (memcmp(read_buf + read_offset, write_buf + write_offset, verify_size) == 0) {
                    printf("    VERIFY OK: overlap LBA %u-%u (checksum=0x%08x)\n",
                           overlap_start_lba, overlap_end_lba - 1, read_checksum);
                } else {
                    printf("    VERIFY FAILED: overlap LBA %u-%u\n", overlap_start_lba, overlap_end_lba - 1);
                    printf("      Expected checksum: 0x%08x, got: 0x%08x\n",
                           expected_checksum, read_checksum);

                    /* Show first few mismatches */
                    mismatch_count = 0;
                    for (i = 0; i < verify_size && mismatch_count < 10; i++) {
                        if (read_buf[read_offset + i] != write_buf[write_offset + i]) {
                            printf("      Offset %lu: expected 0x%02x, got 0x%02x\n",
                                   (unsigned long)i,
                                   write_buf[write_offset + i],
                                   read_buf[read_offset + i]);
                            mismatch_count++;
                        }
                    }
                    errors++;
                }
            } else {
                printf("    Read OK (no overlap, skipped verification)\n");
            }

            free_aligned(read_buf);
        }

        free_aligned(write_buf);
        printf("\n");
    }

    printf("=== Random Write/Read Test Summary ===\n");
    printf("Iterations: %u\n", num_iterations);
    printf("Total writes: %d\n", total_writes);
    printf("Total reads: %d\n", total_reads);
    printf("Errors: %d\n", errors);

    if (errors == 0) {
        printf("SUCCESS: All tests passed!\n");
    } else {
        printf("FAILURE: %d errors detected\n", errors);
    }
    printf("\n");
}
