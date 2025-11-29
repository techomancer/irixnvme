#!smake
#
# Makefile for IRIX NVMe Driver
#
# This builds a loadable kernel module for NVMe support on IRIX 6.5
#
# Uses smake (SGI's parallel make) and follows IRIX loadable module conventions
#

# Target CPU board - change this based on your system
# Supportted values IP32 (O2) IP30 (Octane) IP35 (Fuel, Tezro, Origin 350)
CPUBOARD=IP30
BUILTIN=0
# Include the IRIX kernel loadable I/O module makefile
# This provides $(CC), $(LD), $(CFLAGS), $(LDFLAGS), $(ML), etc.

#if $(BUILTIN) == "1"
include /var/sysgen/Makefile.kernio
BUILTIN_CFLAGS=-DNVME_BUILTIN
#else
include /var/sysgen/Makefile.kernloadio
BUILTIN_CFLAGS=-DNVME_MODULE
#endif

COMMON_FLAGS=
COMMON_LDFLAGS=-v
COMMON_CFLAGS=$(BUILTIN_CFLAGS)

LDFLAGS_IP35=-nostdlib -64 -mips4
LDFLAGS_IP30=-nostdlib -64 -mips4
LDFLAGS_IP32=-nostdlib -n32 -mips3
MYCFLAGS_IP35=-mips4 -DPTE_64BIT
MYCFLAGS_IP30=-mips4 -DPTE_64BIT -DHEART_INVALIDATE_WAR
MYCFLAGS_IP32=-mips3

#if $(CPUBOARD) == "IP30"
MYCFLAGS=$(MYCFLAGS_IP30) $(COMMON_FLAGS) $(COMMON_CFLAGS)
LDFLAGS=$(LDFLAGS_IP30) $(COMMON_FLAGS) $(COMMON_LDFLAGS)
#elif $(CPUBOARD) == "IP32"
MYCFLAGS=$(MYCFLAGS_IP32) $(COMMON_FLAGS) $(COMMON_CFLAGS)
LDFLAGS=$(LDFLAGS_IP32) $(COMMON_FLAGS) $(COMMON_LDFLAGS)
#elif $(CPUBOARD) == "IP35"
MYCFLAGS=$(MYCFLAGS_IP35) $(COMMON_FLAGS) $(COMMON_CFLAGS)
LDFLAGS=$(LDFLAGS_IP35) $(COMMON_FLAGS) $(COMMON_LDFLAGS)
#else
#endif
# Define module loader tool
ML=ml

# Source files
SRCS = nvmedrv.c nvme_scsi.c nvme_cmd.c nvme_cpl.c
OBJS = $(SRCS:.c=.o)

# Header dependencies
HDRS = nvme.h nvmedrv.h

# Module name
MODULE = nvme.o

# Default target
all: $(MODULE)

# Link all object files into a single loadable module
# Use ld with -r flag to create relocatable object (kernel module)
$(MODULE): $(OBJS)
	$(LD) $(LDFLAGS) -r $(OBJS) -o $(MODULE)

# Compile C files to object files
# Makefile.kernloadio should provide proper CFLAGS for loadable modules
.c.o:
	$(CC) $(CFLAGS) $(MYCFLAGS) -c $<

# Dependencies
nvmedrv.o: nvmedrv.c $(HDRS)
nvme_scsi.o: nvme_scsi.c $(HDRS)
nvme_cmd.o: nvme_cmd.c $(HDRS)
nvme_cpl.o: nvme_cpl.c $(HDRS)

# Load the driver into the running kernel using ml (module loader)
# Register as character device to allow ml loading (even though we don't use it)
# -c = character device
# -p nvme_ = driver prefix
# -s 13 = major device number (arbitrary, change if conflicts)
# -v = verbose output
load: $(MODULE)
	@echo "Loading NVMe driver module..."
	$(ML) ld -v -c $(MODULE) -p nvme_ -s 13

# Unload the driver from the kernel
# ml unld = unload module
# -v = verbose
# -p nvme_ = driver prefix to unload
unload:
	$(ML) unld -v -p nvme_

# Reload the driver (unload then load)
reload: unload load

# List loaded modules
list:
	$(ML) list

# Clean build artifacts
clean:
	rm -f $(OBJS) $(MODULE)

# Install target (for permanent installation, not implemented yet)
install: $(MODULE)
	cp master.d/nvme /var/sysgen/master.d/
	cp nvme.o /var/sysgen/boot/
	@echo add "USE: nvme" at the end of /var/sysgen/system/irix.sm
	@echo and run autoconfig

reboot:
	shutdown -y -g0 -i6

ioc:
	ioconfig -d -f /hw

nt:
	cc nvmetest.c -o nt

dp:
	diskperf -D -W -c100m -r4k -m4m /nvme/lol


# Help target
help:
	@echo "IRIX NVMe Driver Makefile"
	@echo ""
	@echo "Build using smake (SGI's parallel make)"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build nvme.o loadable module (default)"
	@echo "  load     - Load the driver into the running kernel"
	@echo "  unload   - Unload the driver from the kernel"
	@echo "  reload   - Unload then reload the driver"
	@echo "  list     - List loaded modules"
	@echo "  clean    - Remove build artifacts"
	@echo "  help     - Show this help message"
	@echo ""
	@echo "Variables:"
	@echo "  CPUBOARD - Target CPU board (currently: $(CPUBOARD))"
	@echo ""
	@echo "Usage:"
	@echo "  smake            # Build module"
	@echo "  smake load       # Load into kernel"
	@echo "  smake unload     # Unload from kernel"
	@echo "  smake reload     # Reload (unload + load)"
	@echo "  smake list       # Show loaded modules"
	@echo ""
	@echo "Module Loader Commands:"
	@echo "  ml ld -p nvme_ nvme.o   # Load module"
	@echo "  ml unld -p nvme_        # Unload by prefix"
	@echo "  ml list                 # List modules"

.PHONY: all load unload reload list clean install help reboot ioc nt dp
