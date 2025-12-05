# NDP - NVMe Disk Performance Benchmark

A modern disk benchmark tool inspired by IRIX's `diskperf`, with multithreading and asynchronous I/O support.

## Features

- **Multithreaded I/O**: Uses multiple threads to saturate multiple CPUs and I/O queues
- **Asynchronous I/O**: Uses POSIX AIO for efficient I/O operations
- **Multiple test patterns**:
  - Forward sequential read/write
  - Backward sequential read/write
  - Random read/write
- **Cross-platform**: Compiles on both IRIX 6.5 (MIPSpro 7.4) and modern Linux

## Building

### On IRIX:
```bash
make -f Makefile.ndp
```

### On Linux:
Edit [Makefile.ndp](Makefile.ndp) and change line 5 to:
```make
UNAME = Linux
```
Then run:
```bash
make -f Makefile.ndp
```

Or compile directly:
```bash
gcc -O2 -Wall -D_GNU_SOURCE -o ndp ndp.c -lpthread -lrt
```

## Usage

```bash
ndp [options] <testpath>
```

### Options

- `-W` - Enable write tests (default: read-only, non-destructive)
- `-D` - Use direct I/O (bypasses filesystem cache)
- `-t <seconds>` - Duration in seconds for each test (default: 5)
- `-r <size>` - Minimum block size (default: 4k)
- `-m <size>` - Maximum block size (default: 1m)
- `-c <size>` - Test file size (default: 1g)
- `-j <threads>` - Number of parallel threads (default: 1)
- `-h` - Print help message

Size suffixes: `k` (KB), `m` (MB), `g` (GB)

### Examples

Read-only test on existing file:
```bash
ndp /disk/testfile
```

Read and write test with 4 threads:
```bash
ndp -W -j 4 -c 1g /disk/testfile
```

Direct I/O test with custom block sizes:
```bash
ndp -D -r 4k -m 4m -t 10 -j 8 /disk/testfile
```

## Output

The benchmark displays throughput (MB/s) for each combination of block size and test pattern:

```
block_sz   | fwd-rd     | fwd-wr     | bwd-rd     | bwd-wr     | rand-rd    | rand-wr
-----------+------------+------------+------------+------------+------------+------------
4096       | 245.32     | ---        | 243.11     | ---        | 178.45     | ---
8192       | 312.67     | ---        | 309.22     | ---        | 201.33     | ---
...
```

## Implementation Notes

- On IRIX, uses `aio_sgi_init()` to configure the AIO subsystem
- Each thread maintains its own I/O buffer and statistics
- Random number generator uses simple LCG for thread-safety
- Threads are staggered to avoid initial offset collisions
- Statistics are aggregated using mutex-protected counters

## Testing NVMe Driver

This tool is perfect for testing the IRIX NVMe driver's multiple I/O queue support:

```bash
# Single-threaded baseline
ndp -W -j 1 -c 1g /disk1s0/testfile

# Multi-threaded to exercise multiple queues
ndp -W -j 4 -c 1g /disk1s0/testfile
```

If the NVMe driver supports multiple I/O queues properly, you should see significantly higher throughput with more threads.
