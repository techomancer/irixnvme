# IRIX NVMe Driver

An NVMe driver for SGI IRIX 6.5, providing SCSI emulation to allow legacy IRIX disk drivers to work with modern NVMe SSDs.

## Building
Install all the dev env and mipspro 7.4.4.
Set CPUBOARD in Makefile to IP30, IP32 or IP35
make
make load
make ioc
hinv -v
fx
enjoy

## Tested with
- PLX and PERICOM PCI-PCIe bridges, (Sedna and Startech on Amazon), the PLX bridges are cheaper on eBay
- your favorite no name brand passive PCIe-M.2 adapter
- cheapo Liteon and Patriot NVME ssds. Samsung 990 EVO didnt show up on the bus though.
- R12K/300 O2 and Dual R14k/600 Octane with shoebox.

## Project Structure

```
irixnvme/
├── nvme.h          - NVMe specification definitions (registers, commands, structures)
├── nvmedrv.h       - Driver-specific structures and function prototypes
├── nvmedrv.c       - Main driver (PCI, initialization, SCSI registration)
├── nvme_scsi.c     - SCSI to NVMe translation layer
├── nvme_cmd.c      - NVMe command construction and submission
├── nvme_cpl.c      - NVMe completion queue handling
├── Makefile        - IRIX make build file
└── README.md       - This file
```

## Architecture

### Component Responsibilities

**nvme.h**
- Pure NVMe spec definitions from NVMe 1.0e
- Register offsets, bit definitions
- Command/completion structures
- Status codes
- Portable across different OS implementations

**nvmedrv.h**
- IRIX-specific driver structures
- `nvme_soft_t` - Controller state and namespace information
- `nvme_queue_t` - Queue pair management
- Function prototypes for all modules

**nvmedrv.c**
- PCI driver registration (wildcard matching on class code 0x010802)
- Controller initialization and shutdown
- SCSI controller registration
- Hardware graph integration
- Interrupt handling

**nvme_scsi.c**
- SCSI CDB to NVMe command translation
- Implements standard SCSI commands:
  - INQUIRY
  - READ CAPACITY
  - TEST UNIT READY
  - READ (6/10/16)
  - WRITE (6/10/16)
  - MODE SENSE
  - SYNC CACHE
- Provides SCSI interface functions for dksc driver

**nvme_cmd.c**
- NVMe command construction
- Admin commands (Identify, Create/Delete Queues)
- I/O commands (Read, Write)
- Command submission to queues
- Doorbell ringing

**nvme_cpl.c**
- Completion queue processing
- Phase bit tracking
- Completion handler dispatch
- Doorbell updates

## How It Works

### PCI Discovery
1. Driver registers with wildcard vendor/device IDs (-1, -1)
2. Attach function checks PCI class code for NVMe (0x010802)
3. Maps BAR0 (controller registers)
4. Reads controller capabilities

### SCSI Emulation
1. Creates SCSI controller in hardware graph: `/hw/scsi_ctlr/N`
2. NVMe namespace 1 appears as SCSI target 0
3. IRIX `dksc` driver attaches to SCSI devices
4. SCSI commands are translated to NVMe commands

### Device Paths

```
May be somewhat different on Octane
/hw/node/io/pci/9
├── scsi_ctlr/(probably)2/    <- NVMe driver creates this
    └── target/0/             <- Namespace 1
        └── lun/0/
            └── disk/         <- dksc attaches here
                └── dks2d1s0  <- Usable block device

```

### SCSI to NVMe Translation Examples

| SCSI Command | NVMe Translation |
|--------------|------------------|
| INQUIRY | Synthesize from Identify Controller |
| READ CAPACITY(10) | Get from Identify Namespace (NSZE field) |
| READ(10) | NVMe Read command with LBA/count from CDB |
| WRITE(10) | NVMe Write command with LBA/count from CDB |
| TEST UNIT READY | Check controller ready bit (CSTS.RDY) |

## Building

On an IRIX system with kernel build tools:

```from your shell
make           # Compile all modules
make load      # Load driver into kernel
make unload    # Unload driver (doesnt quite work, make reboot does)
make clean     # Remove build artifacts
make nt        # build test program

```

## Testing

The `nvmetest.c` program provides comprehensive testing of the driver via the SCSI layer:

```bash
# Compile test program on IRIX
cc -o nvmetest nvmetest.c

# Basic tests
./nvmetest -a                    # Run all basic tests (INQUIRY, READ CAPACITY, single read)

# Single operations
./nvmetest -i                    # INQUIRY test
./nvmetest -c                    # READ CAPACITY test
./nvmetest -r 0                  # Read single block at LBA 0
./nvmetest -w 100                # Write/read verification at LBA 100

# Sequential reads (tests PRP chaining)
./nvmetest -s 0 8192            # Read 8192 blocks (4MB) starting at LBA 0

# Random write/read stress test (DESTRUCTIVE!)
./nvmetest -R 100               # 100 iterations of random write/read tests
./nvmetest -R 1000              # 1000 iterations for extended testing
```

### Random Write/Read Test Features

The `-R` option runs a comprehensive stress test that:

- **Random LBA selection**: Writes to random locations within first 10,000 blocks
- **Random transfer sizes**: 1 block to 1024 blocks (512KB) per operation
- **Random buffer alignments**: Tests 4KB page-aligned, 512B block-aligned, and random 4-byte alignments
- **Data verification**: Each write is followed by 2-5 overlapping reads with checksum verification
- **Pattern generation**: Uses reproducible random data patterns for verification

This test is **DESTRUCTIVE** and will overwrite data on the drive. Only use on test systems with no important data.

Example output:
```
=== Random Write/Read Stress Test ===
Max LBA range: 0-10000
Iterations: 100

--- Iteration 1/100 ---
WRITE: LBA 4523, blocks 128 (65536 bytes), align 4096 bytes
  Write OK (checksum=0x12345678)
  READ 1/3: LBA 4523, blocks 64 (32768 bytes), align 512 bytes
    VERIFY OK: overlap LBA 4523-4586 (checksum=0x0abcdef0)
  READ 2/3: LBA 4600, blocks 50 (25600 bytes), align 4096 bytes
    VERIFY OK: overlap LBA 4600-4650 (checksum=0x0fedcba0)
...
```

## Hardware Requirements

- SGI system running IRIX 6.5
- PCI to PCIe bridge
- NVMe SSD
- Tested platforms: IP32 (O2), IP30 (Octane), IP35 (Fuel), others could work

## Inspiration

This driver is equally stupid as the Windows 2000 NVMe driver (`nvme2k`), attempting to bring modern NVMe storage to a legacy OS that predates the NVMe specification by over a decade.

## References

- NVMe 1.0e Specification
- IRIX Device Driver Programmer's Guide (useless)
- simple_irix_driver (basic loadable module example)

## License

BSD 3 clause.
Use at your risk, don't cry to me if it kills your cat or something.

## Author

Bringing NVMe to IRIX, one ridiculous project at a time.
