# Heavy-write stability fixes

Symptom being addressed: on an O2 (IP32) used as a capture/render scratch
disk (ZapIt, Jaleo, Discreet), sustained large writes end in

    ALERT: I/O error in filesystem ("/NVME") meta-data dev 0xb9 block ...

followed by XFS forcing the filesystem offline.

XFS prints that message when a *metadata* buffer write returns an error
from the block layer, and it shuts the filesystem down immediately.  So the
question is: how does a write get an error back from this driver under
load?  Reading the code, there were several ways, and heavy sequential
writes trigger all of them at once.

## Root causes found

### 1. Any request the driver could not resource was returned as an error

`nvme_scsi_read_write()` needed, for every request, one CID per NVMe
command plus a PRP-list page per command larger than two pages.  The pool
was 64 PRP pages and 512 CIDs.  A typical cheap drive reports MDTS = 128 KB,
so a 4 MB direct-I/O write from a video app becomes 32 commands needing 32
CIDs and 32 PRP pages.  Two such requests in flight exhaust the PRP pool;
sixteen exhaust the CIDs.

When that happened the driver completed the request with
`sr_status = SC_REQUEST, sr_scsi_status = ST_BUSY`.  `SC_REQUEST` tells
dksc "the adapter rejected this request", which it treats as a hard error,
not a retry.  Under load the victims are the small XFS metadata writes that
arrive while the big data writes hold all the resources - exactly the
message you see.

PRP pages were also allocated *while* building commands, so a request could
have half its commands already submitted when it ran out, and the partially
submitted request was then completed with BUSY.

**Fix:** all READ/WRITE/SYNC CACHE requests now go through a deferred-request
ring (`nvme_defer_*`, `nvme_dispatch_deferred`).  A request reserves *all*
of its CIDs and PRP pages atomically in `nvme_io_reserve()` before anything
is submitted; if the reservation cannot be satisfied the request simply
waits in the ring and is started by the next completion.  Nothing is ever
bounced back to dksc for lack of driver resources.  The PRP pool was also
doubled to 128 pages.  A raw user-address request (dsreq-style ioctl) cannot
be translated from an interrupt thread, so it is started synchronously or
gets a plain `ST_BUSY` (which dksc *does* retry).

### 2. A 2-second command timeout with abort

`nvme_check_timeouts()` aborted any command older than 2 seconds.  Consumer
SSDs routinely stall for several seconds once their SLC cache fills during
sustained writes.  Every in-flight command then got aborted, came back as
`SC_CMDTIME`, and dksc retried it - into a drive that was still stalled.

**Fix:** per-request timeouts.  The driver now honours the timeout dksc asks
for (`req->sr_timeout`) with a floor of 30 s (`NVME_IO_TIMEOUT_MIN_SEC`) and
a 60 s default.  Aborts are rate-limited to the controller's Abort Command
Limit (ACL, now parsed from Identify Controller) so a stall no longer floods
the admin queue with aborts that the drive rejects anyway.  Abort
completions are now actually accounted (the old handler returned early on
any admin failure, so rejected aborts were never seen).  A controller-fatal
status (CSTS.CFS) is now logged when it occurs.

### 3. Retries were split into one 512-byte command per block

When a request matched the "aborted command FIFO", the retry was issued
with `max_transfer_blocks = 1`: a 1 MB retry became 2048 NVMe commands.
Anything over 256 KB failed outright with "not enough CIDs" (-> SC_REQUEST
again), and smaller ones were slow enough to time out a second time.

**Fix:** removed.  The FIFO is kept for a diagnostic message only.

### 4. Race between CID allocation and the timeout scanner (crash)

`nvme_io_cid_alloc()` set the CID bit under the lock but filled in `.req`
and `.start_time` *after* unlocking.  The 10 Hz timeout scanner walks the
bitmap under the same lock: it could see a set bit with a NULL `req` and a
stale `start_time` from a previous use, decide the command had timed out,
and call `nvme_aborted_fifo_add(soft, NULL)` -> NULL dereference in
interrupt context.  Rare, but "any heavy write" is a lot of tries.

**Fix:** slots are initialised inside the lock (`nvme_io_reserve`), and the
scanner tolerates a NULL anyway.

### 5. Unaligned multi-command buffers built wrong PRPs (data corruption)

`nvme_get_translated_addr()` only corrected the "PRP1 straddles a page"
case for the very first segment of a request.  For a request split into
several NVMe commands whose buffer was not page aligned, commands 2..N were
given a PRP1 that crossed a page boundary; the drive transfers from the
wrong pages.  Consumer apps mostly use page-aligned buffers so this is
uncommon, but it is silent corruption when it happens.

**Fix:** every fetched segment is checked; if it straddles an NVMe page the
cursor is rewound and the segment is refetched with the exact in-page
length.

### 6. MDTS converted assuming 512-byte blocks

`max_transfer_blocks` was computed from MDTS as `bytes / 512` even for
4Kn-formatted namespaces, producing commands 8x larger than the drive
permits.  Now `nvme_compute_limits()` derives the limit from MDTS, the real
block size, and how much the PRP chain can describe, and caps "unlimited"
(MDTS = 0) drives at 4 MB per command.

### 7. Ordered / head-of-queue tags did nothing useful

An NVMe Flush was submitted "first" but nothing waited for it, and NVMe does
not order commands within a queue, so the flush provided no ordering.  It
also consumed an SQ slot that was not tracked against the CID count, which
could make `nvme_submit_cmd()` fail with a full queue (-> BUSY) even though
CIDs were available.

**Fix:** ordered/head requests are real barriers now: they are not started
until the I/O queue is empty and nothing behind them starts until they
complete.  SYNCHRONIZE CACHE also waits for the queue to drain before the
Flush is issued (an NVMe Flush only covers *completed* writes).  CIDs
handed to the SCSI path are capped at `SQ size - 1 - NVME_IO_CID_RESERVE`
so a reserved command can always be submitted.

### 8. `nvme_prp_entries` overflowed to zero

`soft->nvme_prp_entries` was a `u_int8_t`; 4096/8 = 512 wraps to 0 (and
16384/8 = 2048 on IP30/IP35 too).  The old PRP builder happened to survive
this because a 512 KB command never needs a second list page, but the new
reservation code computed "0 PRP pages needed" for every request and then
correctly refused to build it ("needs more than the 0 reserved PRP pages").
Found from the SYSLOG of the first test build; now a `uint_t`.

### 9. Submission-queue "full" decided by the controller's SQHD (wedge)

`nvme_submit_cmd()` declared the SQ full when `tail + 1 == sq_head`, where
`sq_head` came from the SQHD field of the last completion entry.  The
second test run showed the ring "full" while nothing was outstanding, and
staying that way: one bad SQHD from the drive (MAXIO MAP1202 in the
Patriot P300) set `sq_head = tail + 1`; since nothing could be submitted,
no completion ever arrived to correct it.  The original driver had the
same check.

**Fix:** flow control now uses the driver's own `outstanding` counter
(full iff `outstanding >= size - 1`), which cannot wedge.  SQHD is only
used for diagnostics and a sanity-checked, rate-limited warning is logged
if the controller reports a head outside the range it could legally be in.
If the SQ nevertheless looks full before a request's first command, the
request is returned to the ring instead of being failed.

### 10. Completion entries read in the wrong order (source of the bogus SQHD)

`nvme_read_completion()` read DW0, DW1, DW2, DW3 and then checked the
phase bit in DW3.  The NVMe controller writes DW3 *last* precisely so that
a reader that sees a new phase bit can trust DW0..DW2 - but only if DW3 is
read *first*.  Reading DW2 (SQ head) before DW3 means an entry landing
between the two reads yields "new phase, stale SQ head", which is exactly
what produced the permanent "queue full" wedge in item 9.  With the
watchdog polling the CQ every 2 ms under load, the window gets hit.
Now DW3 is read first.

## Verified against SGI documentation

The interfaces relied on were checked against the IRIX 6.5 *Device Driver
Programmer's Guide* (007-0911-210), chapters 16 (SCSI), 20 (PCI device
attachment, "PCI Implementation in O2 Workstations") and 21 (Services for
PCI drivers):

* `sr_timeout` is documented as "number of ticks (HZ units) to wait for a
  response before timing out; the host adapter driver supplies a minimum
  value if this field is zero or too small" - which is exactly what
  `nvme_io_reserve()` now does.
* `SC_REQUEST` is documented as "an error was detected in the input values;
  the command was not attempted", and its log string is "Driver protocol
  error" - the line preceding every XFS alert in your SYSLOG.  It is not a
  retry status.  `ST_BUSY`, by contrast, is documented as "the driver will
  normally delay and then request the command again", so that is what the
  ring-full fallback returns.
* PCI interrupt handlers "run as an independent thread in the kernel", so
  taking mutexes and building commands from the completion path (the
  dispatcher) is legitimate on 6.5.
* O2 specifics: 32-bit PCI, no 64-bit addressing (the driver already sets
  no `PCIIO_DMA_A64` on IP32), cache snooping not supported so "cache
  coherency must be ensured by the driver" (queues and PRP pool are
  `VM_UNCACHED`, data buffers are flushed with `bp_dcache_wbinval` /
  `dki_dcache_*`), and BARs are placed above 0x80000000 (matches the
  `bar0 size is 0, overriding` workaround).
* The guide states that on O2 all PIO must use `pciio_pio_read32/write32`
  built with `-DUSE_PCI_PIO`.  The driver uses raw volatile loads/stores
  and has worked that way, so this is left as an opt-in: `USE_PCI_PIO=1`
  on the make line.  If you ever see doorbell writes appearing lost, try
  that first.

## Other changes

* `nvmectl <N> stats` prints counters and high-water marks (CIDs, PRP
  pages, defer-ring depth, timeouts, aborts, errors).  Run it during a
  capture to see how close you are to the limits.
* Clean shutdown / detach fails any parked requests instead of leaking
  them, and refuses new I/O once shutdown starts.
* Completions for CIDs that are not in flight are logged instead of
  silently ignored.

## Things deliberately not done (yet)

* **Controller reset on a dead command.**  If the drive never completes a
  command and rejects the abort, the request stays pending forever.  The
  correct answer is CC.EN=0 / re-create queues / fail everything
  outstanding, like every mainstream NVMe driver does.  That is a larger
  change and needs hardware to test against; the CFS log message at least
  tells you when you are in that state.
* `si_maxq` (dksc queue depth) is still 32.  With deferral it is safe to
  raise; try 64 if you want more concurrency.

## Testing notes

This was reviewed and syntax-checked against stub headers but **not
compiled with MIPSPro or run on hardware** - I do not have an IRIX box.
Please:

1. Build with `CPUBOARD=IP32`, load, and run `nvmetest -a` and
   `nvmetest -R 200` on a scratch partition first (destructive).
2. Reproduce your capture workload; watch `SYSLOG` for `nvme:` lines and
   run `nvmectl 2 stats` afterwards.  `requests deferred` > 0 with
   `requests bounced BUSY` = 0 and `command timeouts` = 0 is the goal.
3. If you still see timeouts, the drive itself is stalling for >30 s; the
   floor is `NVME_IO_TIMEOUT_MIN_SEC` in `nvmedrv.h`.
