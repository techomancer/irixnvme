#!smake
# Makefile for ndp (NVMe Disk Performance benchmark)
# Supports both IRIX 6.5 (MIPSpro 7.4) and Linux (gcc)

# Set this to IRIX64 or Linux depending on your platform
ISIRIX=1

#if $(ISIRIX) == "1"
CC = cc
CFLAGS = -O2 -n32 -mips4 -woff 1174,1552,3201
LDFLAGS = -lpthread
TARGET = ndp
#else
CC = gcc
CFLAGS = -O2 -Wall -D_GNU_SOURCE
LDFLAGS = -lpthread -lrt
TARGET = ndp
#endif

# Source files
SRCS = ndp.c
OBJS = $(SRCS:.c=.o)

# Default target
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
