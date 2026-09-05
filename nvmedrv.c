/*
 * nvmedrv.c - NVMe driver for SGI IRIX
 *
 * A PCI driver for NVM Express (NVMe) storage devices on IRIX 6.5
 * Based on the NVMe 1.0 specification
 */

#include "nvmedrv.h"

/*
 * Module version information, required for loadable modules.
 */
char *nvme_mversion = M_VERSION;

/*
 * Device flags - D_MP indicates multiprocessor safe
 */
int nvme_devflag = D_MP;

/*
 * Global soft-state table indexed by adapter number.
 * Populated in nvme_attach(), cleared in nvme_detach_attached_soft().
 * Used by nvme_ioctl() to map minor device number → soft state.
 */
static nvme_soft_t *nvme_ctlr_softmap[NVME_MAX_CTLR];

/* NVMe PCI Class Codes */
#define PC_CLASS_STORAGE       0x01        /* Mass Storage Controller */
#define PCI_SUBCLASS_NVM        0x08        /* Non-Volatile Memory */
#define PCI_PROGIF_NVME         0x02        /* NVMe */
#define NVME_CLASS_CODE         0x010802    /* Full class code: Storage/NVM/NVMe */

/* PCI Bridge Class Codes */
#define PCI_CLASS_BRIDGE        0x06        /* Bridge Device */
#define PCI_SUBCLASS_PCI_BRIDGE 0x04        /* PCI-to-PCI Bridge */
#define PCI_BRIDGE_CLASS_CODE   0x060400    /* Full class code: Bridge/PCI-to-PCI */

/* PCI Command Register Bits (not all in PCI_defs.h) */
#define PCI_CMD_INTERRUPT_DISABLE 0x400     /* Interrupt Disable (bit 10) */

/* PCI Bridge Config Space Offsets (Type 1 Header) */
#define PCI_BRIDGE_PRIMARY_BUS      0x18    /* Primary Bus Number */
#define PCI_BRIDGE_SECONDARY_BUS    0x19    /* Secondary Bus Number */
#define PCI_BRIDGE_SUBORDINATE_BUS  0x1A    /* Subordinate Bus Number */
#define PCI_BRIDGE_SEC_LATENCY      0x1B    /* Secondary Latency Timer */
#define PCI_BRIDGE_IO_BASE          0x1C    /* I/O Base (lower byte) */
#define PCI_BRIDGE_IO_LIMIT         0x1D    /* I/O Limit (lower byte) */
#define PCI_BRIDGE_SEC_STATUS       0x1E    /* Secondary Status (2 bytes) */
#define PCI_BRIDGE_MEM_BASE         0x20    /* Memory Base (2 bytes) */
#define PCI_BRIDGE_MEM_LIMIT        0x22    /* Memory Limit (2 bytes) */
#define PCI_BRIDGE_PREFETCH_BASE    0x24    /* Prefetchable Memory Base (2 bytes) */
#define PCI_BRIDGE_PREFETCH_LIMIT   0x26    /* Prefetchable Memory Limit (2 bytes) */
#define PCI_BRIDGE_PREFETCH_BASE_HI 0x28    /* Prefetchable Base Upper 32 bits */
#define PCI_BRIDGE_PREFETCH_LIMIT_HI 0x2C   /* Prefetchable Limit Upper 32 bits */
#define PCI_BRIDGE_IO_BASE_HI       0x30    /* I/O Base Upper 16 bits */
#define PCI_BRIDGE_IO_LIMIT_HI      0x32    /* I/O Limit Upper 16 bits */
#define PCI_BRIDGE_CONTROL          0x3E    /* Bridge Control (2 bytes) */

/* Bridge Control Register Bits */
#define PCI_BRIDGE_CTL_PARITY       0x0001  /* Parity Error Response Enable */
#define PCI_BRIDGE_CTL_SERR         0x0002  /* SERR# Enable */
#define PCI_BRIDGE_CTL_ISA          0x0004  /* ISA Enable */
#define PCI_BRIDGE_CTL_VGA          0x0008  /* VGA Enable */
#define PCI_BRIDGE_CTL_MASTER_ABORT 0x0020  /* Master Abort Mode */
#define PCI_BRIDGE_CTL_BUS_RESET    0x0040  /* Secondary Bus Reset */
#define PCI_BRIDGE_CTL_FAST_BACK    0x0080  /* Fast Back-to-Back Enable */

/* Useful macros */
#define NEW(ptr)    (ptr = kmem_zalloc(sizeof (*(ptr)), KM_SLEEP))
#define DEL(ptr)    (kmem_free(ptr, sizeof (*(ptr))))

/*
 * nvme_log2 - Calculate log2 of a power-of-2 value
 * Returns the number of bits needed to represent the value
 */
static __inline uint_t
nvme_log2(uint_t val)
{
    uint_t shift = 0;
    while (val > 1) {
        val >>= 1;
        shift++;
    }
    return shift;
}

/*
 * nvme_scan_ctlr_callback - Callback for scaninvent() to find max controller number
 */
static int
nvme_scan_ctlr_callback(inventory_t *inv, void *arg)
{
    int *max_ctlr = (int *)arg;

    /* Check if this is a SCSI controller */
    if (inv->inv_class == INV_DISK &&
        (inv->inv_type == INV_SCSICONTROL || inv->inv_type == INV_PCI_SCSICONTROL)) {
        /* Track the maximum controller number we've seen
         * Note: inv_controller is unsigned (major_t), so we cast to int for comparison
         * We also need to handle the case where max_ctlr is -1 initially */
        if (*max_ctlr < 0 || (int)inv->inv_controller > *max_ctlr) {
            *max_ctlr = (int)inv->inv_controller;
        }
    }

    return 0;  /* Continue scanning */
}

/*
 * nvme_get_next_adapter_num - Find next available SCSI adapter number
 *
 * Scans the system inventory to find all existing SCSI controllers
 * and returns the next available adapter number.
 *
 * Returns: Next available adapter number
 */
static int
nvme_get_next_adapter_num(void)
{
    int max_ctlr = -1;

    /* Scan inventory to find highest existing SCSI controller number */
    scaninvent(nvme_scan_ctlr_callback, &max_ctlr);

    return max_ctlr + 1;
}

#if defined(IP32)
#define NVME_IP32_ALIAS_SLOTS 64
#define NVME_IP32_CANDIDATE_SLOTS 64
#define NVME_IP32_FINALIZE_DELAY_US 500000 /* Wait 500ms of quiet before final attach pass */

typedef struct nvme_ip32_alias_entry_s {
    int             in_use;
    char            serial[21];  /* Trimmed ASCII serial key */
    vertex_hdl_t    conn;        /* Currently attached alias for this serial */
    nvme_soft_t    *soft;        /* Attached soft state for this alias */
    int             slot;        /* PCI slot of attached alias */
} nvme_ip32_alias_entry_t;

static nvme_ip32_alias_entry_t nvme_ip32_alias_table[NVME_IP32_ALIAS_SLOTS];

typedef struct nvme_ip32_candidate_entry_s {
    int             in_use;
    char            serial[21];  /* Trimmed ASCII serial key */
    vertex_hdl_t    conn;        /* Latest enumerated alias for this serial */
    int             slot;        /* Slot for latest alias */
    nvme_soft_t    *soft;        /* Pre-initialized soft state saved from discovery phase */
} nvme_ip32_candidate_entry_t;

static nvme_ip32_candidate_entry_t nvme_ip32_candidate_table[NVME_IP32_CANDIDATE_SLOTS];
static mutex_t         nvme_ip32_candidate_lock;
static int             nvme_ip32_candidate_lock_ready = 0;
static sema_t          nvme_ip32_finalize_sema;
static volatile int    nvme_ip32_finalize_shutdown = 0;
static volatile int    nvme_ip32_finalize_thread_running = 0;
static volatile uint_t nvme_ip32_candidate_gen = 0;
static volatile int    nvme_ip32_finalize_active = 0;
static volatile vertex_hdl_t nvme_ip32_finalize_conn = (vertex_hdl_t)0;
static volatile nvme_soft_t *nvme_ip32_finalize_soft = NULL; /* Pre-init soft for current finalize conn */

static void
nvme_ip32_candidate_lock_init_once(void)
{
    if (!nvme_ip32_candidate_lock_ready) {
        init_mutex(&nvme_ip32_candidate_lock, MUTEX_DEFAULT, "nvme_ip32cand", 0);
        nvme_ip32_candidate_lock_ready = 1;
    }
}

static void
nvme_ip32_build_serial_key(const uchar_t *serial_in, char *serial_out, size_t serial_out_len)
{
    int end;
    int i;

    if (!serial_out || serial_out_len == 0) {
        return;
    }

    serial_out[0] = '\0';
    if (!serial_in) {
        return;
    }

    end = 20;
    while (end > 0) {
        uchar_t c = serial_in[end - 1];
        if (c != ' ' && c != '\0') {
            break;
        }
        end--;
    }

    if (end <= 0) {
        return;
    }

    if (end >= (int)serial_out_len) {
        end = (int)serial_out_len - 1;
    }

    for (i = 0; i < end; i++) {
        serial_out[i] = (char)serial_in[i];
    }
    serial_out[end] = '\0';
}

static int
nvme_ip32_alias_find(const char *serial_key)
{
    int i;

    if (!serial_key || serial_key[0] == '\0') {
        return -1;
    }

    for (i = 0; i < NVME_IP32_ALIAS_SLOTS; i++) {
        if (nvme_ip32_alias_table[i].in_use &&
            strcmp(nvme_ip32_alias_table[i].serial, serial_key) == 0) {
            return i;
        }
    }

    return -1;
}

static void
nvme_ip32_alias_set_conn(const char *serial_key, vertex_hdl_t conn, nvme_soft_t *soft, int slot)
{
    int idx;
    int i;
    size_t len;

    if (!serial_key || serial_key[0] == '\0') {
        return;
    }

    idx = nvme_ip32_alias_find(serial_key);
    if (idx >= 0) {
        nvme_ip32_alias_table[idx].conn = conn;
        nvme_ip32_alias_table[idx].soft = soft;
        nvme_ip32_alias_table[idx].slot = slot;
        return;
    }

    for (i = 0; i < NVME_IP32_ALIAS_SLOTS; i++) {
        if (!nvme_ip32_alias_table[i].in_use) {
            nvme_ip32_alias_table[i].in_use = 1;
            nvme_ip32_alias_table[i].conn = conn;
            nvme_ip32_alias_table[i].soft = soft;
            nvme_ip32_alias_table[i].slot = slot;
            bzero(nvme_ip32_alias_table[i].serial, sizeof(nvme_ip32_alias_table[i].serial));
            len = strlen(serial_key);
            if (len >= sizeof(nvme_ip32_alias_table[i].serial)) {
                len = sizeof(nvme_ip32_alias_table[i].serial) - 1;
            }
            bcopy(serial_key, nvme_ip32_alias_table[i].serial, len);
            nvme_ip32_alias_table[i].serial[len] = '\0';
            return;
        }
    }

    cmn_err(CE_WARN, "nvme_attach: IP32 serial alias table full, cannot track SN=%s",
            serial_key);
}

static void
nvme_ip32_alias_clear_conn(const char *serial_key, vertex_hdl_t conn)
{
    int idx;

    idx = nvme_ip32_alias_find(serial_key);
    if (idx < 0) {
        return;
    }

    if (nvme_ip32_alias_table[idx].conn != conn) {
        return;
    }

    bzero(&nvme_ip32_alias_table[idx], sizeof(nvme_ip32_alias_table[idx]));
}

static int
nvme_ip32_candidate_find_locked(const char *serial_key)
{
    int i;

    if (!serial_key || serial_key[0] == '\0') {
        return -1;
    }

    for (i = 0; i < NVME_IP32_CANDIDATE_SLOTS; i++) {
        if (nvme_ip32_candidate_table[i].in_use &&
            strcmp(nvme_ip32_candidate_table[i].serial, serial_key) == 0) {
            return i;
        }
    }

    return -1;
}

/* Forward declaration needed by nvme_ip32_candidate_set_latest */
static void nvme_ip32_cleanup_staged_soft(nvme_soft_t *soft);

static int
nvme_ip32_candidate_set_latest(const char *serial_key, vertex_hdl_t conn, int slot,
                                nvme_soft_t *soft)
{
    int idx;
    int i;
    size_t len;
    nvme_soft_t *old_soft;

    if (!serial_key || serial_key[0] == '\0') {
        return -1;
    }

    if (!nvme_ip32_candidate_lock_ready) {
        nvme_ip32_candidate_lock_init_once();
    }

    mutex_lock(&nvme_ip32_candidate_lock, PZERO);

    idx = nvme_ip32_candidate_find_locked(serial_key);
    if (idx < 0) {
        for (i = 0; i < NVME_IP32_CANDIDATE_SLOTS; i++) {
            if (!nvme_ip32_candidate_table[i].in_use) {
                idx = i;
                bzero(&nvme_ip32_candidate_table[idx], sizeof(nvme_ip32_candidate_table[idx]));
                nvme_ip32_candidate_table[idx].in_use = 1;
                len = strlen(serial_key);
                if (len >= sizeof(nvme_ip32_candidate_table[idx].serial)) {
                    len = sizeof(nvme_ip32_candidate_table[idx].serial) - 1;
                }
                bcopy(serial_key, nvme_ip32_candidate_table[idx].serial, len);
                nvme_ip32_candidate_table[idx].serial[len] = '\0';
                break;
            }
        }
    }

    if (idx < 0) {
        mutex_unlock(&nvme_ip32_candidate_lock);
        cmn_err(CE_WARN, "nvme_attach: IP32 candidate table full, cannot stage SN=%s",
                serial_key);
        return -1;
    }

    old_soft = NULL;
    if (nvme_ip32_candidate_table[idx].conn != (vertex_hdl_t)0 &&
        nvme_ip32_candidate_table[idx].conn != conn) {
        cmn_err(CE_NOTE,
                "nvme_attach: IP32 duplicate candidate for SN=%s, replacing conn 0x%x (slot %d) with 0x%x (slot %d)",
                serial_key,
                (uint_t)nvme_ip32_candidate_table[idx].conn,
                nvme_ip32_candidate_table[idx].slot,
                (uint_t)conn,
                slot);
        /* Capture old soft to free after releasing the lock */
        old_soft = nvme_ip32_candidate_table[idx].soft;
        nvme_ip32_candidate_table[idx].soft = NULL;
    }

    nvme_ip32_candidate_table[idx].conn = conn;
    nvme_ip32_candidate_table[idx].slot = slot;
    nvme_ip32_candidate_table[idx].soft = soft;
    nvme_ip32_candidate_gen++;

    mutex_unlock(&nvme_ip32_candidate_lock);

    /* Free displaced soft outside the lock to avoid holding it during shutdown */
    if (old_soft != NULL) {
        nvme_ip32_cleanup_staged_soft(old_soft);
    }

    return 0;
}

static int
nvme_ip32_candidate_collect(vertex_hdl_t *conns, nvme_soft_t **softs, int max_conns)
{
    int i;
    int count = 0;

    if (!conns || !softs || max_conns <= 0) {
        return 0;
    }

    if (!nvme_ip32_candidate_lock_ready) {
        return 0;
    }

    mutex_lock(&nvme_ip32_candidate_lock, PZERO);
    for (i = 0; i < NVME_IP32_CANDIDATE_SLOTS && count < max_conns; i++) {
        if (!nvme_ip32_candidate_table[i].in_use) {
            continue;
        }
        if (nvme_ip32_candidate_table[i].conn == (vertex_hdl_t)0) {
            continue;
        }
        conns[count] = nvme_ip32_candidate_table[i].conn;
        softs[count] = nvme_ip32_candidate_table[i].soft; /* Transfer ownership */
        nvme_ip32_candidate_table[i].soft = NULL;
        count++;
        bzero(&nvme_ip32_candidate_table[i], sizeof(nvme_ip32_candidate_table[i]));
    }
    mutex_unlock(&nvme_ip32_candidate_lock);

    return count;
}
#endif

/* =====================================================================
 *    FUNCTION TABLE OF CONTENTS
 */

void        nvme_init(void);
int         nvme_unload(void);
int         nvme_reg(void);
int         nvme_unreg(void);

int         nvme_attach(vertex_hdl_t conn);
int         nvme_detach(vertex_hdl_t conn);
static int  nvme_detach_attached_soft(nvme_soft_t *soft, vertex_hdl_t conn, int fast_alias);
static int  nvme_shutdown_ex(nvme_soft_t *soft, int fast_alias);
#if defined(IP32)
/* nvme_ip32_cleanup_staged_soft forward-declared earlier */
static void nvme_ip32_free_replaced_soft(nvme_soft_t *soft);
static void nvme_ip32_do_soft_swap(int alias_idx, vertex_hdl_t new_conn, nvme_soft_t *new_soft);
static void nvme_ip32_finalize_candidates(void);
static void nvme_ip32_finalize_thread(void *arg);
static void nvme_ip32_start_finalize_thread(void);
static void nvme_ip32_stop_finalize_thread(void);
#endif

static pciio_iter_f nvme_reloadme;
static pciio_iter_f nvme_unloadme;

/* =====================================================================
 *    Error Handler
 */

/*
 * nvme_error_handler: PCI error handler callback
 *
 * Called by the PCI infrastructure when a hardware error is detected
 * involving this device (e.g., PCI parity errors, target aborts, etc.)
 *
 * Arguments:
 *   einfo      - Driver private data (nvme_soft_t) passed to pciio_error_register
 *   error_code - Error code indicating the type of error
 *   mode       - Error handling mode (probe, register, etc.)
 *   ioerror    - Detailed error information structure
 *
 * Returns:
 *   IOERROR_HANDLED if error was handled
 *   Other error codes to allow error to propagate
 */
static int
nvme_error_handler(void *einfo, int error_code, ioerror_mode_t mode, ioerror_t *ioerror)
{
    nvme_soft_t *soft = (nvme_soft_t *)einfo;

    cmn_err(CE_ALERT, "nvme_error_handler: PCI error detected (code=%d, mode=%d)",
                error_code, mode);

    /* Dump detailed error information using system utility */
    ioerror_dump("nvme", error_code, mode, ioerror);

    /*
     * For production, we mark the error as handled to prevent system panic.
     * In a more sophisticated implementation, we might:
     * - Attempt controller reset if the error is recoverable
     * - Mark the controller as failed and fail all outstanding I/O
     * - Set a flag to prevent new I/O from being submitted
     * - Notify the SCSI layer that the device is unavailable
     */

    return IOERROR_HANDLED;
}

/* =====================================================================
 *    Driver Initialization
 */

/*
 * nvme_init: called once during system startup or when a loadable
 * driver is loaded.
 */
void
nvme_init(void)
{
    cmn_err(CE_NOTE, "nvme_init: NVMe driver for IRIX initializing");

    /* If we are already registered, this is a reload */
    pciio_iterate("nvme_", nvme_reloadme);
#ifdef NVME_BUILTIN
    /*
     * Register with wildcards (-1, -1) to get called for ALL PCI devices.
     * We'll filter by class code in nvme_attach().
     * IRIX CDL (Connection/Driver List) supports -1 wildcarding.
     */
    pciio_driver_register(PCIIO_VENDOR_ID_NONE,    /* -1 = wildcard vendor */
                          PCIIO_DEVICE_ID_NONE,    /* -1 = wildcard device */
                          "nvme_",
                          0);
#endif    
}

/*
 * nvme_unload: unload the driver if not in use
 */
int
nvme_unload(void)
{
    cmn_err(CE_NOTE, "nvme_unload: unloading NVMe driver");

#if defined(IP32)
    nvme_ip32_stop_finalize_thread();
#endif

    pciio_iterate("nvme_", nvme_unloadme);

    return 0;
}

#if defined(IP32)
static void
nvme_ip32_cleanup_staged_soft(nvme_soft_t *soft)
{
    if (!soft) {
        return;
    }

    nvme_shutdown_ex(soft, 1);
    if (soft->bar0_map) {
        pciio_piomap_free(soft->bar0_map);
        soft->bar0_map = 0;
    }
    DEL(soft);
}

/*
 * nvme_ip32_free_replaced_soft: Release kernel resources for a soft state that
 * has been superseded by a soft swap.
 *
 * Unlike nvme_ip32_cleanup_staged_soft this intentionally does NOT write any
 * NVMe MMIO registers.  The controller hardware has already been reset and
 * reconfigured by the replacement soft's nvme_sanitize()+nvme_initialize(), so
 * touching CC/CSTS here would only disturb the live controller.
 */
static void
nvme_ip32_free_replaced_soft(nvme_soft_t *soft)
{
    if (!soft) {
        return;
    }

    /* Cancel kernel timer callbacks - no hardware access */
    nvme_timeout_watchdog_stop(soft);
    nvme_watchdog_stop(&soft->io_queue);
    nvme_watchdog_stop(&soft->admin_queue);

    /* Free utility buffer */
    if (soft->utility_buffer) {
        kvpfree(soft->utility_buffer, 1);
        soft->utility_buffer = NULL;
    }

    /* Free alenlist */
    if (soft->alenlist) {
        mutex_destroy(&soft->alenlist_lock);
        alenlist_destroy(soft->alenlist);
        soft->alenlist = NULL;
    }

    /* Free PRP pool */
    nvme_prp_pool_done(soft);

    /* Destroy command tracking locks and defer ring */
    soft->shutting_down = 1;
    nvme_defer_fail_all(soft);
    nvme_defer_destroy(soft);
    mutex_destroy(&soft->io_requests_lock);
    mutex_destroy(&soft->aborted_lock);

#ifdef NVME_UTILBUF_USEDMAP
    pciio_dmamap_free(soft->utility_buffer_dmamap);
#endif

    /* Free I/O queue descriptor ring memory */
    if (soft->io_queue.sq) {
        kvpfree(soft->io_queue.sq, (uint)btoc(soft->io_queue.size * NVME_SQ_ENTRY_SIZE));
        soft->io_queue.sq = NULL;
    }
    if (soft->io_queue.cq) {
        kvpfree(soft->io_queue.cq, (uint)btoc(soft->io_queue.size * NVME_CQ_ENTRY_SIZE));
        soft->io_queue.cq = NULL;
    }
    mutex_destroy(&soft->io_queue.lock);

    /* Free admin queue descriptor ring memory */
    if (soft->admin_queue.sq) {
        kvpfree(soft->admin_queue.sq, (uint)btoc(soft->admin_queue.size * NVME_SQ_ENTRY_SIZE));
        soft->admin_queue.sq = NULL;
    }
    if (soft->admin_queue.cq) {
        kvpfree(soft->admin_queue.cq, (uint)btoc(soft->admin_queue.size * NVME_CQ_ENTRY_SIZE));
        soft->admin_queue.cq = NULL;
    }
    mutex_destroy(&soft->admin_queue.lock);

    /* Release the replaced conn's BAR0 PIO mapping */
    if (soft->bar0_map) {
        pciio_piomap_free(soft->bar0_map);
        soft->bar0_map = 0;
    }

    /*
     * INTENTIONALLY do not DEL(soft) here.
     *
     * The IRIX SCSI layer (dksc) stores the scsi_target_info_t pointer
     * returned by nvme_scsi_info() — which is &soft->tinfo, an address
     * inside this struct — and continues to dereference it (for si_inq,
     * si_sense, etc.) long after attachment.  Freeing the struct would
     * leave dksc with a dangling pointer and cause a tlbmiss panic.
     *
     * The struct is therefore intentionally left allocated as a "zombie":
     * all expensive NVMe sub-resources above have been freed, the
     * watchdog is stopped, and bar0_map is gone, so no hardware activity
     * can occur through this soft again.  The struct body (~16 KB with the
     * embedded io_requests[512] array) stays mapped so that any cached
     * pointer into it (tinfo, inq_data, sense_data) remains valid.
     *
     * With at most ~12 alias appearances across 4 drives, the total
     * zombie overhead is under 200 KB — acceptable for a boot driver.
     */
    soft->initialized = 0;
    soft->bar0 = NULL;
}

/*
 * nvme_ip32_do_soft_swap: Replace the live soft state for an already-registered
 * controller with a freshly initialised one from the latest PCI alias.
 *
 * The SCSI controller vertex, adapter number, and all hwgraph paths remain
 * unchanged; only the nvme_soft_t pointer stored in SCI_INFO is updated so
 * that subsequent SCSI commands use the new conn's hardware queues.
 *
 * The old soft's hardware was already reset by the new soft's
 * nvme_sanitize()+nvme_initialize(), so we release old soft's resources
 * without writing to NVMe MMIO.
 */
static void
nvme_ip32_do_soft_swap(int alias_idx, vertex_hdl_t new_conn, nvme_soft_t *new_soft)
{
    nvme_soft_t      *old_soft;
    scsi_ctlr_info_t *scsi_info;

    if (!new_soft) {
        cmn_err(CE_WARN, "nvme: IP32 soft swap: new_soft is NULL for conn 0x%x",
                (uint_t)new_conn);
        return;
    }

    old_soft = nvme_ip32_alias_table[alias_idx].soft;
    if (!old_soft) {
        /* No live old soft to swap; release new_soft to avoid a leak */
        nvme_ip32_cleanup_staged_soft(new_soft);
        return;
    }

    /* Inherit the SCSI registration from the old soft */
    new_soft->scsi_vhdl = old_soft->scsi_vhdl;
    new_soft->adap      = old_soft->adap;

    /* Transfer character control device ownership to the new soft.
     * old_soft->ctrl_vhdl's device_info still points to old_soft; update it
     * now so that nvme_open/nvme_ioctl resolve to the live soft after the swap.
     * Do this before nvme_ip32_free_replaced_soft() zeroes old_soft->initialized. */
    new_soft->ctrl_vhdl = old_soft->ctrl_vhdl;
    old_soft->ctrl_vhdl = GRAPH_VERTEX_NONE;
    if (new_soft->ctrl_vhdl != GRAPH_VERTEX_NONE)
        device_info_set(new_soft->ctrl_vhdl, new_soft);

    /* Keep global softmap in sync */
    if (new_soft->adap < NVME_MAX_CTLR)
        nvme_ctlr_softmap[new_soft->adap] = new_soft;

    /* Redirect the SCSI layer to the new hardware state */
    scsi_info = scsi_ctlr_info_get(new_soft->scsi_vhdl);
    if (scsi_info) {
        SCI_INFO(scsi_info) = new_soft;
    } else {
        cmn_err(CE_WARN,
                "nvme: IP32 soft swap: scsi_ctlr_info_get returned NULL "
                "(adap=%d vhdl=0x%x)",
                new_soft->adap, (uint_t)new_soft->scsi_vhdl);
    }

    /* Register PCI error handler for the new conn */
    pciio_error_register(new_conn, nvme_error_handler, new_soft);

    /* Arm watchdog and mark not quiesced for new soft */
    nvme_timeout_watchdog_start(new_soft);
    new_soft->quiesce_state = NO_QUIESCE_IN_PROGRESS;

    cmn_err(CE_NOTE,
            "nvme: IP32 soft swap: adapter %d updated to conn 0x%x",
            new_soft->adap, (uint_t)new_conn);

    /* Release old soft resources without touching hardware */
    nvme_ip32_free_replaced_soft(old_soft);

    /* Update alias table */
    nvme_ip32_alias_table[alias_idx].soft = new_soft;
    nvme_ip32_alias_table[alias_idx].conn = new_conn;
}

static void
nvme_ip32_finalize_candidates(void)
{
    vertex_hdl_t conns[NVME_IP32_CANDIDATE_SLOTS];
    nvme_soft_t *softs[NVME_IP32_CANDIDATE_SLOTS];
    int count;
    int i;

    count = nvme_ip32_candidate_collect(conns, softs, NVME_IP32_CANDIDATE_SLOTS);
    if (count <= 0) {
        return;
    }

    cmn_err(CE_NOTE, "nvme_attach: IP32 finalize pass attaching %d controller candidate(s)",
            count);

    nvme_ip32_finalize_active = 1;
    for (i = 0; i < count; i++) {
        char serial_key[21];
        int alias_idx;

        /* Derive serial from the staged soft so we can check the alias table */
        if (softs[i] != NULL) {
            nvme_ip32_build_serial_key(softs[i]->serial, serial_key, sizeof(serial_key));
        } else {
            serial_key[0] = '\0';
        }

        alias_idx = (serial_key[0] != '\0') ? nvme_ip32_alias_find(serial_key) : -1;

        if (alias_idx < 0) {
            /*
             * Controller not yet registered: do full SCSI attach via the
             * finalize fast-path (skips sanitize+initialize since soft is
             * pre-initialised from the discovery phase).
             */
            nvme_ip32_finalize_conn = conns[i];
            nvme_ip32_finalize_soft = softs[i]; /* May be NULL if pre-init unavailable */
            (void)nvme_attach(conns[i]);
            nvme_ip32_finalize_conn = (vertex_hdl_t)0;
            /* If nvme_attach didn't consume the pre-init soft (early failure),
             * clean it up to avoid leaking. */
            if (nvme_ip32_finalize_soft != NULL) {
                nvme_ip32_cleanup_staged_soft((nvme_soft_t *)nvme_ip32_finalize_soft);
                nvme_ip32_finalize_soft = NULL;
            }
        } else {
            /*
             * Controller already registered under an earlier alias: swap the
             * soft state so the newest conn's hardware is used for I/O.
             * The SCSI/hwgraph registration stays intact.
             */
            nvme_ip32_do_soft_swap(alias_idx, conns[i], softs[i]);
        }
    }
    nvme_ip32_finalize_active = 0;
}

static void
nvme_ip32_finalize_thread(void *arg)
{
    uint_t snapshot_gen;
    (void)arg;

    nvme_ip32_finalize_thread_running = 1;
    while (!nvme_ip32_finalize_shutdown) {
        psema(&nvme_ip32_finalize_sema, PZERO);
        if (nvme_ip32_finalize_shutdown) {
            break;
        }

        for (;;) {
            snapshot_gen = nvme_ip32_candidate_gen;
            delay(drv_usectohz(NVME_IP32_FINALIZE_DELAY_US));
            if (nvme_ip32_finalize_shutdown) {
                break;
            }
            if (snapshot_gen == nvme_ip32_candidate_gen) {
                break;
            }
        }

        if (nvme_ip32_finalize_shutdown) {
            break;
        }

        nvme_ip32_finalize_candidates();

        /* Drain any vsema() signals that accumulated during enumeration so we
         * don't re-enter the finalize loop unnecessarily (each extra wakeup
         * would add one more NVME_IP32_FINALIZE_DELAY_US of idle wait). */
        while (cpsema(&nvme_ip32_finalize_sema))
            ;
    }
    nvme_ip32_finalize_thread_running = 0;
}

static void
nvme_ip32_start_finalize_thread(void)
{
    if (nvme_ip32_finalize_thread_running) {
        return;
    }

    nvme_ip32_candidate_lock_init_once();
    bzero(nvme_ip32_candidate_table, sizeof(nvme_ip32_candidate_table));
    initnsema(&nvme_ip32_finalize_sema, 0, "nvme_ip32f");
    nvme_ip32_candidate_gen = 0;
    nvme_ip32_finalize_shutdown = 0;
    nvme_ip32_finalize_active = 0;
    nvme_ip32_finalize_conn = (vertex_hdl_t)0;
    nvme_ip32_finalize_soft = NULL;
    nvme_ip32_finalize_thread_running = 1;

    sthread_create("nvme_ip32_finalize",
                    NULL, 2 * KTHREAD_DEF_STACKSZ,
                    0,
                    scsi_intr_pri,
                    KT_PS,
                    nvme_ip32_finalize_thread,
                    0,
                    0, 0, 0);
}

static void
nvme_ip32_stop_finalize_thread(void)
{
    int timeout;

    if (!nvme_ip32_finalize_thread_running) {
        return;
    }

    nvme_ip32_finalize_shutdown = 1;
    vsema(&nvme_ip32_finalize_sema);

    timeout = 500; /* 5 seconds max */
    while (nvme_ip32_finalize_thread_running && timeout-- > 0) {
        delay(drv_usectohz(10000));
    }

    freesema(&nvme_ip32_finalize_sema);

    if (nvme_ip32_candidate_lock_ready) {
        int i;
        mutex_lock(&nvme_ip32_candidate_lock, PZERO);
        for (i = 0; i < NVME_IP32_CANDIDATE_SLOTS; i++) {
            if (nvme_ip32_candidate_table[i].soft != NULL) {
                nvme_ip32_cleanup_staged_soft(nvme_ip32_candidate_table[i].soft);
                nvme_ip32_candidate_table[i].soft = NULL;
            }
        }
        bzero(nvme_ip32_candidate_table, sizeof(nvme_ip32_candidate_table));
        mutex_unlock(&nvme_ip32_candidate_lock);
    }
    if (nvme_ip32_finalize_soft != NULL) {
        nvme_ip32_cleanup_staged_soft((nvme_soft_t *)nvme_ip32_finalize_soft);
        nvme_ip32_finalize_soft = NULL;
    }
    nvme_ip32_finalize_active = 0;
    nvme_ip32_finalize_conn = (vertex_hdl_t)0;
}
#endif

#ifndef NVME_BUILTIN
/*
 * nvme_reg: register the driver with the PCI infrastructure
 * NOTE: Should be called from _reg, not _init
 *
 * We use wildcard registration (vendor=-1, device=-1) to match ANY PCI device.
 * Our attach function will check the class code to ensure it's an NVMe controller.
 * This allows us to support NVMe devices from any vendor without hardcoding IDs.
 */
int
nvme_reg(void)
{
    cmn_err(CE_NOTE, "nvme_reg: registering NVMe driver with wildcard matching");

    /*
     * Register with wildcards (-1, -1) to get called for ALL PCI devices.
     * We'll filter by class code in nvme_attach().
     * IRIX CDL (Connection/Driver List) supports -1 wildcarding.
     */
    pciio_driver_register(PCIIO_VENDOR_ID_NONE,    /* -1 = wildcard vendor */
                          PCIIO_DEVICE_ID_NONE,    /* -1 = wildcard device */
                          "nvme_",
                          0);

    return 0;
}

/*
 * nvme_unreg: unregister the driver
 */
int
nvme_unreg(void)
{
    cmn_err(CE_NOTE, "nvme_unreg: unregistering NVMe driver");

#if defined(IP32)
    nvme_ip32_stop_finalize_thread();
#endif

    pciio_driver_unregister("nvme_");

    return 0;
}
#endif

#ifdef NVME_DBG_EXTRA
/*
 * ============================================================================
 * Controller Diagnostics and State Inspection
 * ============================================================================
 */

/*
 * nvme_dump_controller_state: Dump all NVMe controller registers for debugging
 *
 * This function reads and displays all important NVMe controller registers.
 * Very useful for debugging when the controller isn't responding as expected.
 */
void
nvme_dump_controller_state(nvme_soft_t *soft, const char *context)
{
    uint_t cap_lo, cap_hi, vs, intms, intmc, cc, csts, aqa;
    uint_t asq_lo, asq_hi, acq_lo, acq_hi;
    uint_t sq_doorbell, cq_doorbell;
    __uint64_t cap;

    cmn_err(CE_NOTE, "=== NVMe Controller State Dump: %s ===", context);

    /* Read Controller Capabilities (64-bit) */
    cap_lo = NVME_RD(soft, NVME_REG_CAP);
    cap_hi = NVME_RD(soft, NVME_REG_CAP + 4);
    cap = ((__uint64_t)cap_hi << 32) | cap_lo;

    cmn_err(CE_NOTE, "CAP (0x00): 0x%08x%08x", cap_hi, cap_lo);
    cmn_err(CE_NOTE, "  MQES (Max Queue Entries):    %u", (uint_t)(cap & 0xFFFF) + 1);
    cmn_err(CE_NOTE, "  CQR (Contiguous Queues Req): %u", (uint_t)((cap >> 16) & 1));
    cmn_err(CE_NOTE, "  AMS (Arb Mechanisms):        0x%x", (uint_t)((cap >> 17) & 3));
    cmn_err(CE_NOTE, "  TO (Timeout):                %u ms", (uint_t)((cap >> 24) & 0xFF) * 500);
    cmn_err(CE_NOTE, "  DSTRD (Doorbell Stride):     %u bytes", 4 << ((cap >> 32) & 0xF));
    cmn_err(CE_NOTE, "  NSSRS (NVM Subsys Reset):    %u", (uint_t)((cap >> 36) & 1));
    cmn_err(CE_NOTE, "  CSS (Command Sets):          0x%x", (uint_t)((cap >> 37) & 0xFF));
    cmn_err(CE_NOTE, "  MPSMIN:                      %u", (uint_t)((cap >> 48) & 0xF));
    cmn_err(CE_NOTE, "  MPSMAX:                      %u", (uint_t)((cap >> 52) & 0xF));

    /* Read Version */
    vs = NVME_RD(soft, NVME_REG_VS);
    cmn_err(CE_NOTE, "VS  (0x08): 0x%08x (NVMe %u.%u.%u)", vs,
            (vs >> 16) & 0xFFFF, (vs >> 8) & 0xFF, vs & 0xFF);

    /* Read Interrupt Masks */
    intms = NVME_RD(soft, NVME_REG_INTMS);
    intmc = NVME_RD(soft, NVME_REG_INTMC);
    cmn_err(CE_NOTE, "INTMS (0x0C): 0x%08x (Interrupt Mask Set)", intms);
    cmn_err(CE_NOTE, "INTMC (0x10): 0x%08x (Interrupt Mask Clear)", intmc);
    cmn_err(CE_NOTE, "  Effective Mask: 0x%08x", intms);

    /* Read Controller Configuration */
    cc = NVME_RD(soft, NVME_REG_CC);
    cmn_err(CE_NOTE, "CC  (0x14): 0x%08x", cc);
    cmn_err(CE_NOTE, "  EN (Enable):                 %u", cc & 1);
    cmn_err(CE_NOTE, "  CSS (Command Set):           %u", (cc >> 4) & 7);
    cmn_err(CE_NOTE, "  MPS (Memory Page Size):      %u (= %u bytes)",
            (cc >> 7) & 0xF, 4096 << ((cc >> 7) & 0xF));
    cmn_err(CE_NOTE, "  AMS (Arbitration):           %u", (cc >> 11) & 7);
    cmn_err(CE_NOTE, "  SHN (Shutdown):              %u", (cc >> 14) & 3);
    cmn_err(CE_NOTE, "  IOSQES (SQ Entry Size):      %u (= %u bytes)",
            (cc >> 16) & 0xF, 1 << ((cc >> 16) & 0xF));
    cmn_err(CE_NOTE, "  IOCQES (CQ Entry Size):      %u (= %u bytes)",
            (cc >> 20) & 0xF, 1 << ((cc >> 20) & 0xF));

    /* Read Controller Status */
    csts = NVME_RD(soft, NVME_REG_CSTS);
    cmn_err(CE_NOTE, "CSTS (0x1C): 0x%08x", csts);
    cmn_err(CE_NOTE, "  RDY (Ready):                 %u %s",
            csts & 1, (csts & 1) ? "[READY]" : "[NOT READY]");
    cmn_err(CE_NOTE, "  CFS (Controller Fatal):      %u %s",
            (csts >> 1) & 1, ((csts >> 1) & 1) ? "[FATAL ERROR!]" : "[OK]");
    cmn_err(CE_NOTE, "  SHST (Shutdown Status):      %u", (csts >> 2) & 3);
    cmn_err(CE_NOTE, "  NSSRO (NVM Subsys Reset):    %u", (csts >> 4) & 1);

    /* Read Admin Queue Attributes */
    aqa = NVME_RD(soft, NVME_REG_AQA);
    cmn_err(CE_NOTE, "AQA (0x24): 0x%08x", aqa);
    cmn_err(CE_NOTE, "  ASQS (Admin SQ Size):        %u", (aqa & 0xFFF) + 1);
    cmn_err(CE_NOTE, "  ACQS (Admin CQ Size):        %u", ((aqa >> 16) & 0xFFF) + 1);

    /* Read Admin Queue Addresses */
    asq_lo = NVME_RD(soft, NVME_REG_ASQ);
    asq_hi = NVME_RD(soft, NVME_REG_ASQ + 4);
    acq_lo = NVME_RD(soft, NVME_REG_ACQ);
    acq_hi = NVME_RD(soft, NVME_REG_ACQ + 4);
    cmn_err(CE_NOTE, "ASQ (0x28): 0x%08x%08x (Admin SQ Base)", asq_hi, asq_lo);
    cmn_err(CE_NOTE, "ACQ (0x30): 0x%08x%08x (Admin CQ Base)", acq_hi, acq_lo);

    /* Read Admin Queue Doorbells */
    sq_doorbell = NVME_RD(soft, soft->admin_queue.sq_doorbell);
    cq_doorbell = NVME_RD(soft, soft->admin_queue.cq_doorbell);
    cmn_err(CE_NOTE, "Admin SQ Doorbell (0x%04x): 0x%08x",
            soft->admin_queue.sq_doorbell, sq_doorbell);
    cmn_err(CE_NOTE, "Admin CQ Doorbell (0x%04x): 0x%08x",
            soft->admin_queue.cq_doorbell, cq_doorbell);

    /* Print queue state from soft structure */
    cmn_err(CE_NOTE, "Driver Queue State:");
    cmn_err(CE_NOTE, "  Admin SQ: head=%u tail=%u size=%u",
            soft->admin_queue.sq_head, soft->admin_queue.sq_tail,
            soft->admin_queue.size);
    cmn_err(CE_NOTE, "  Admin CQ: head=%u size=%u phase=%u",
            soft->admin_queue.cq_head & soft->admin_queue.size_mask,
            soft->admin_queue.size,
            (soft->admin_queue.cq_head >> soft->admin_queue.size_shift) & 1);

    cmn_err(CE_NOTE, "=== End Controller State Dump ===");
}
#endif /* NVME_DBG_EXTR */

#ifdef NVME_DBG_CMD
/*
 * nvme_dump_sq_entry: Dump a submission queue entry for debugging
 */
void
nvme_dump_sq_entry(nvme_command_t *cmd, const char *context)
{
    uint_t *dwords = (uint_t *)cmd;
    int i;

    cmn_err(CE_NOTE, "=== SQ Entry Dump: %s ===", context);
    cmn_err(CE_NOTE, "Address: %p", cmd);

    /* Dump raw dwords with NVME_MEMRD to show what's actually in memory */
    cmn_err(CE_NOTE, "Raw Memory (with byteswap if enabled):");
    for (i = 0; i < 16; i += 4) {
        cmn_err(CE_NOTE, "  DW%02d-%02d: 0x%08x 0x%08x 0x%08x 0x%08x",
                i, i+3,
                NVME_MEMRD(&dwords[i]),
                NVME_MEMRD(&dwords[i+1]),
                NVME_MEMRD(&dwords[i+2]),
                NVME_MEMRD(&dwords[i+3]));
    }

    /* Decode the command (reading with byteswap) */
    cmn_err(CE_NOTE, "Decoded Fields:");
    cmn_err(CE_NOTE, "  CDW0  (OPC/FLAGS/CID): 0x%08x", NVME_MEMRD(&cmd->cdw0));
    cmn_err(CE_NOTE, "    Opcode:              0x%02x", NVME_MEMRD(&cmd->cdw0) & 0xFF);
    cmn_err(CE_NOTE, "    Flags:               0x%02x", (NVME_MEMRD(&cmd->cdw0) >> 8) & 0xFF);
    cmn_err(CE_NOTE, "    CID:                 0x%04x", (NVME_MEMRD(&cmd->cdw0) >> 16) & 0xFFFF);
    cmn_err(CE_NOTE, "  NSID:                  0x%08x", NVME_MEMRD(&cmd->nsid));
    cmn_err(CE_NOTE, "  PRP1:                  0x%08x%08x",
            NVME_MEMRD(&cmd->prp1_hi), NVME_MEMRD(&cmd->prp1_lo));
    cmn_err(CE_NOTE, "  PRP2:                  0x%08x%08x",
            NVME_MEMRD(&cmd->prp2_hi), NVME_MEMRD(&cmd->prp2_lo));
    cmn_err(CE_NOTE, "  CDW10:                 0x%08x", NVME_MEMRD(&cmd->cdw10));
    cmn_err(CE_NOTE, "  CDW11:                 0x%08x", NVME_MEMRD(&cmd->cdw11));
    cmn_err(CE_NOTE, "=== End SQ Entry Dump ===");
}
#endif /* NVME_DBG_CMD */

#ifdef NVME_DBG_EXTRA
/*
 * nvme_dump_memory: Dump arbitrary memory region for debugging
 */
void
nvme_dump_memory(void *addr, size_t len, const char *context)
{
    uint_t *dwords = (uint_t *)addr;
    size_t dword_count = (len + 3) / 4;  /* Round up to dword boundary */
    size_t i;

    cmn_err(CE_NOTE, "=== Memory Dump: %s ===", context);
    cmn_err(CE_NOTE, "Address: %p, Length: %u bytes", addr, len);

    for (i = 0; i < dword_count && i < 64; i += 4) {  /* Limit to 256 bytes */
        if (i + 3 < dword_count) {
            cmn_err(CE_NOTE, "  +0x%03x: 0x%08x 0x%08x 0x%08x 0x%08x",
                    i * 4,
                    NVME_MEMRD(&dwords[i]),
                    NVME_MEMRD(&dwords[i+1]),
                    NVME_MEMRD(&dwords[i+2]),
                    NVME_MEMRD(&dwords[i+3]));
        } else {
            /* Handle last partial line */
            size_t j;
            cmn_err(CE_NOTE, "  +0x%03x:", i * 4);
            for (j = i; j < dword_count; j++) {
                cmn_err(CE_CONT, " 0x%08x", NVME_MEMRD(&dwords[j]));
            }
        }
    }

    if (dword_count > 64) {
        cmn_err(CE_NOTE, "  ... (truncated, %u more dwords)", dword_count - 64);
    }

    cmn_err(CE_NOTE, "=== End Memory Dump ===");
}

/*
 * nvme_dump_pci_bridge: Dump PCI-to-PCI bridge configuration
 *
 * This function dumps the configuration space of a PCI-to-PCI bridge.
 * Useful for debugging DMA and memory mapping issues.
 */
void
nvme_dump_pci_bridge(vertex_hdl_t conn)
{
    uint_t class_code;
    ushort_t vendor_id, device_id, command, status, bridge_control;
    uchar_t header_type, primary_bus, secondary_bus, subordinate_bus, sec_latency;
    uchar_t io_base, io_limit;
    ushort_t mem_base, mem_limit, prefetch_base, prefetch_limit;
    ushort_t io_base_hi, io_limit_hi, sec_status;
    uint_t prefetch_base_hi, prefetch_limit_hi;
    pciio_info_t pciioinfo;

    /* Read class code to verify this is a bridge */
    class_code = (uint_t)pciio_config_get(conn, PCI_CFG_CLASS_CODE, 4) & 0x00FFFFFF;

    /* Class code format: ClassCode (23:16), SubClass (15:8), ProgIF (7:0) */
    if ((class_code >> 16) != PCI_CLASS_BRIDGE) {
        return;  /* Not a bridge, silently return */
    }

    /* Read basic device info */
    vendor_id = (ushort_t)pciio_config_get(conn, PCI_CFG_VENDOR_ID, 2);
    device_id = (ushort_t)pciio_config_get(conn, PCI_CFG_DEVICE_ID, 2);
    command = (ushort_t)pciio_config_get(conn, PCI_CFG_COMMAND, 2);
    status = (ushort_t)pciio_config_get(conn, PCI_CFG_STATUS, 2);
    header_type = (uchar_t)pciio_config_get(conn, PCI_CFG_HEADER_TYPE, 1);

    pciioinfo = pciio_info_get(conn);
    if (!pciioinfo)
        return;

    cmn_err(CE_NOTE, "=== PCI Bridge Configuration Dump ===");
    cmn_err(CE_NOTE, "Location: Bus %d, Slot %d, Function %d",
            pciio_info_bus_get(pciioinfo),
            pciio_info_slot_get(pciioinfo),
            pciio_info_function_get(pciioinfo));
    cmn_err(CE_NOTE, "Vendor:Device: %04x:%04x", vendor_id, device_id);
    cmn_err(CE_NOTE, "Class Code: 0x%06x (%s)",
            class_code,
            (class_code >> 16) == PCI_CLASS_BRIDGE ? "Bridge" : "Other");
    cmn_err(CE_NOTE, "Header Type: 0x%02x (%s)",
            header_type & 0x7F,
            (header_type & 0x7F) == 1 ? "PCI-to-PCI Bridge" : "Other");

    /* Command register */
    cmn_err(CE_NOTE, "Command: 0x%04x", command);
    cmn_err(CE_NOTE, "  I/O Space:           %s", (command & PCI_CMD_IO_SPACE) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  Memory Space:        %s", (command & PCI_CMD_MEM_SPACE) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  Bus Master:          %s", (command & PCI_CMD_BUS_MASTER) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  Special Cycles:      %s", (command & 0x08) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  Memory Write/Inval:  %s", (command & 0x10) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  VGA Palette Snoop:   %s", (command & 0x20) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  Parity Error:        %s", (command & 0x40) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  SERR#:               %s", (command & 0x100) ? "Enabled" : "Disabled");
    cmn_err(CE_NOTE, "  Fast B2B:            %s", (command & 0x200) ? "Enabled" : "Disabled");

    /* Status register */
    cmn_err(CE_NOTE, "Status: 0x%04x", status);
    cmn_err(CE_NOTE, "  66MHz Capable:       %s", (status & 0x20) ? "Yes" : "No");
    cmn_err(CE_NOTE, "  Fast B2B Capable:    %s", (status & 0x80) ? "Yes" : "No");
    cmn_err(CE_NOTE, "  Master Data Parity:  %s", (status & 0x100) ? "Error" : "OK");
    cmn_err(CE_NOTE, "  DEVSEL Timing:       %d", (status >> 9) & 3);
    cmn_err(CE_NOTE, "  Target Abort (sig):  %s", (status & 0x800) ? "Yes" : "No");
    cmn_err(CE_NOTE, "  Target Abort (rcv):  %s", (status & 0x1000) ? "Yes" : "No");
    cmn_err(CE_NOTE, "  Master Abort:        %s", (status & 0x2000) ? "Yes" : "No");
    cmn_err(CE_NOTE, "  System Error:        %s", (status & 0x4000) ? "Yes" : "No");
    cmn_err(CE_NOTE, "  Parity Error:        %s", (status & 0x8000) ? "Yes" : "No");

    /* Bridge-specific registers (Type 1 header) */
    if ((header_type & 0x7F) == 1) {
        primary_bus = (uchar_t)pciio_config_get(conn, PCI_BRIDGE_PRIMARY_BUS, 1);
        secondary_bus = (uchar_t)pciio_config_get(conn, PCI_BRIDGE_SECONDARY_BUS, 1);
        subordinate_bus = (uchar_t)pciio_config_get(conn, PCI_BRIDGE_SUBORDINATE_BUS, 1);
        sec_latency = (uchar_t)pciio_config_get(conn, PCI_BRIDGE_SEC_LATENCY, 1);

        cmn_err(CE_NOTE, "Bus Numbers:");
        cmn_err(CE_NOTE, "  Primary Bus:         %u", primary_bus);
        cmn_err(CE_NOTE, "  Secondary Bus:       %u", secondary_bus);
        cmn_err(CE_NOTE, "  Subordinate Bus:     %u", subordinate_bus);
        cmn_err(CE_NOTE, "  Secondary Latency:   %u", sec_latency);

        /* I/O window */
        io_base = (uchar_t)pciio_config_get(conn, PCI_BRIDGE_IO_BASE, 1);
        io_limit = (uchar_t)pciio_config_get(conn, PCI_BRIDGE_IO_LIMIT, 1);
        io_base_hi = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_IO_BASE_HI, 2);
        io_limit_hi = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_IO_LIMIT_HI, 2);

        cmn_err(CE_NOTE, "I/O Window:");
        cmn_err(CE_NOTE, "  Base:  0x%04x%02x00 (type: %s)",
                io_base_hi, io_base & 0xF0,
                (io_base & 0x0F) == 0 ? "16-bit" : "32-bit");
        cmn_err(CE_NOTE, "  Limit: 0x%04x%02xff",
                io_limit_hi, io_limit & 0xF0);

        /* Memory window */
        mem_base = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_MEM_BASE, 2);
        mem_limit = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_MEM_LIMIT, 2);

        cmn_err(CE_NOTE, "Memory Window:");
        cmn_err(CE_NOTE, "  Base:  0x%04x0000", mem_base & 0xFFF0);
        cmn_err(CE_NOTE, "  Limit: 0x%04xffff", mem_limit & 0xFFF0);

        /* Prefetchable memory window */
        prefetch_base = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_PREFETCH_BASE, 2);
        prefetch_limit = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_PREFETCH_LIMIT, 2);
        prefetch_base_hi = (uint_t)pciio_config_get(conn, PCI_BRIDGE_PREFETCH_BASE_HI, 4);
        prefetch_limit_hi = (uint_t)pciio_config_get(conn, PCI_BRIDGE_PREFETCH_LIMIT_HI, 4);

        cmn_err(CE_NOTE, "Prefetchable Memory Window:");
        cmn_err(CE_NOTE, "  Base:  0x%08x%04x0000 (type: %s)",
                prefetch_base_hi, prefetch_base & 0xFFF0,
                (prefetch_base & 0x0F) == 0 ? "32-bit" : "64-bit");
        cmn_err(CE_NOTE, "  Limit: 0x%08x%04xffff",
                prefetch_limit_hi, prefetch_limit & 0xFFF0);

        /* Secondary status */
        sec_status = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_SEC_STATUS, 2);
        cmn_err(CE_NOTE, "Secondary Status: 0x%04x", sec_status);
        cmn_err(CE_NOTE, "  66MHz Capable:       %s", (sec_status & 0x20) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Fast B2B Capable:    %s", (sec_status & 0x80) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Master Data Parity:  %s", (sec_status & 0x100) ? "Error" : "OK");
        cmn_err(CE_NOTE, "  Target Abort (sig):  %s", (sec_status & 0x800) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Target Abort (rcv):  %s", (sec_status & 0x1000) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Master Abort:        %s", (sec_status & 0x2000) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  System Error:        %s", (sec_status & 0x4000) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Parity Error:        %s", (sec_status & 0x8000) ? "Yes" : "No");

        /* Bridge control */
        bridge_control = (ushort_t)pciio_config_get(conn, PCI_BRIDGE_CONTROL, 2);
        cmn_err(CE_NOTE, "Bridge Control: 0x%04x", bridge_control);
        cmn_err(CE_NOTE, "  Parity Error Resp:   %s", (bridge_control & PCI_BRIDGE_CTL_PARITY) ? "Enabled" : "Disabled");
        cmn_err(CE_NOTE, "  SERR# Enable:        %s", (bridge_control & PCI_BRIDGE_CTL_SERR) ? "Enabled" : "Disabled");
        cmn_err(CE_NOTE, "  ISA Enable:          %s", (bridge_control & PCI_BRIDGE_CTL_ISA) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  VGA Enable:          %s", (bridge_control & PCI_BRIDGE_CTL_VGA) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Master Abort Mode:   %s", (bridge_control & PCI_BRIDGE_CTL_MASTER_ABORT) ? "Yes" : "No");
        cmn_err(CE_NOTE, "  Secondary Bus Reset: %s", (bridge_control & PCI_BRIDGE_CTL_BUS_RESET) ? "ACTIVE" : "Inactive");
        cmn_err(CE_NOTE, "  Fast B2B Enable:     %s", (bridge_control & PCI_BRIDGE_CTL_FAST_BACK) ? "Enabled" : "Disabled");
    }

    cmn_err(CE_NOTE, "=== End PCI Bridge Dump ===");
}
#endif /* NVME_DBG_EXTRA */

/*
 * ============================================================================
 * Controller Initialization and Shutdown Functions
 * ============================================================================
 */

/*
 * nvme_wait_for_ready: Wait for controller ready status
 *
 * Arguments:
 *   soft       - Controller state
 *   ready      - TRUE to wait for ready, FALSE to wait for not ready
 *   timeout_ms - Timeout in milliseconds
 *
 * Returns:
 *   0 on success, -1 on timeout
 */
static int
nvme_wait_for_ready(nvme_soft_t *soft, int ready, uint_t timeout_ms)
{
    uint_t csts;
    uint_t iterations = timeout_ms;  /* Check every 1ms */
    uint_t i;

    for (i = 0; i < iterations; i++) {
        csts = NVME_RD(soft, NVME_REG_CSTS);

        if (ready) {
            if (csts & NVME_CSTS_RDY)
                return 0;  /* Controller is ready */
        } else {
            if (!(csts & NVME_CSTS_RDY))
                return 0;  /* Controller is not ready */
        }

        /* Check for fatal status */
        if (csts & NVME_CSTS_CFS) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme: controller fatal status detected");
#endif
            return -1;
        }

        us_delay(1000);  /* Delay 1ms */
    }

#ifdef NVME_DBG
    cmn_err(CE_WARN, "nvme: timeout waiting for controller %s",
            ready ? "ready" : "not ready");
#endif
    return -1;
}

/*
 * nvme_sanitize: Clean up inherited controller state
 *
 * This function aggressively resets the controller to a known clean state.
 * It handles the case where a previous driver (like option ROM) may have
 * left the controller enabled with queues configured and interrupts firing.
 *
 * Based on Windows driver nuclear option approach.
 */
static int
nvme_sanitize(nvme_soft_t *soft)
{
    uint_t cc, csts;
    int retry_count;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: sanitizing controller state");
#endif
    /*
     * Step 1: MASK ALL INTERRUPTS IMMEDIATELY
     * Critical - do this BEFORE reading any other registers
     */
    NVME_WR(soft, NVME_REG_INTMS, 0xFFFFFFFF);

    /*
     * Step 2: Check current controller state
     */
    csts = NVME_RD(soft, NVME_REG_CSTS);
    cc = NVME_RD(soft, NVME_REG_CC);

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: controller state before reset: CC=%x CSTS=%x", cc, csts);
#endif
    if (csts & NVME_CSTS_RDY) {
        cmn_err(CE_WARN, "nvme: WARNING: controller is READY (previous driver left it enabled!)");
    }
    if (csts & NVME_CSTS_CFS) {
        cmn_err(CE_WARN, "nvme: WARNING: controller fatal status bit set!");
    }

    /*
     * Step 3: Clear any pending interrupts
     */
    (void)NVME_RD(soft, NVME_REG_CSTS);

    /*
     * Step 4: Force mask ALL interrupts again
     */
    NVME_WR(soft, NVME_REG_INTMS, 0xFFFFFFFF);

    /*
     * Step 5: Clear admin queue registers BEFORE disabling controller
     * Prevents controller from trying to access stale queue memory
     */
    NVME_WR(soft, NVME_REG_AQA, 0);
    NVME_WR(soft, NVME_REG_ASQ, 0);       /* ASQ low */
    NVME_WR(soft, NVME_REG_ASQ + 4, 0);   /* ASQ high */
    NVME_WR(soft, NVME_REG_ACQ, 0);       /* ACQ low */
    NVME_WR(soft, NVME_REG_ACQ + 4, 0);   /* ACQ high */

    /*
     * Step 6: Force controller disable with retries
     * The option ROM may have left it in a weird state
     */
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: disabling controller");
#endif
    retry_count = 5;
    while (retry_count > 0) {
        /* Clear CC.EN and CC.SHN bits */
        cc = NVME_RD(soft, NVME_REG_CC);
        cc &= ~(NVME_CC_ENABLE | NVME_CC_SHN_MASK);
        NVME_WR(soft, NVME_REG_CC, cc);

        /* Wait for controller to become not ready (10 second timeout) */
        if (nvme_wait_for_ready(soft, 0, 10000) == 0) {
#ifdef NVME_DBG
            cmn_err(CE_NOTE, "nvme: controller disabled successfully");
#endif
            break;
        }

        cmn_err(CE_WARN, "nvme: controller disable retry %d/5", 6 - retry_count);
        retry_count--;

        /* On last retry, try nuclear option: write 0 to CC */
        if (retry_count == 1) {
            cmn_err(CE_WARN, "nvme: trying nuclear option: writing 0 to CC");
            NVME_WR(soft, NVME_REG_CC, 0);
            if (nvme_wait_for_ready(soft, 0, 10000) == 0) {
                break;
            }
        }
    }

    /*
     * Step 7: Verify controller is disabled
     */
    csts = NVME_RD(soft, NVME_REG_CSTS);
    if (csts & NVME_CSTS_RDY) {
        cmn_err(CE_WARN, "nvme: FATAL: controller failed to disable, CSTS=%x", csts);
        return -1;
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: controller is now disabled and clean");
#endif
    /*
     * Step 8: Re-mask interrupts after controller reset
     * Some controllers (like QEMU) clear INTMS during reset
     */
    NVME_WR(soft, NVME_REG_INTMS, 0xFFFFFFFF);

    return 0;
}

/*
 * nvme_initialize: Initialize NVMe controller
 *
 * This function:
 * - Allocates admin queue and utility buffer
 * - Enables the controller
 * - Queries controller and namespace information (polling mode, no interrupts)
 *
 * Based on Windows driver init sequence.
 */
static int
nvme_initialize(nvme_soft_t *soft)
{
    uint_t cc, aqa;
    uint_t queue_size;
    uint pages;
    alenaddr_t phys_addr;
    ushort_t command;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: initializing controller");
#endif
    /*
     * Configure PCI cache line size and latency timer to match system.
     * Cache line size of 128 bytes (0x80) matches SGI system cache line size.
     * This is important for proper DMA performance.
     */
    pciio_config_set(soft->pci_vhdl, PCI_CFG_LATENCY_TIMER, sizeof(uint8_t), (uint64_t)0xF0);
    pciio_config_set(soft->pci_vhdl, PCI_CFG_CACHE_LINE, sizeof(uint8_t), (uint64_t)0x80);
#ifdef IP30
#if 0
    // that causes ultra bad juju
    if (pcibr_device_flags_set(soft->pci_vhdl, PCIBR_64BIT) != 1) {
        cmn_err(CE_WARN, "nvme: cannot set 64bit");
    }
#endif
    {
        vertex_hdl_t xconn_vhdl; // The xtalk connect point for the bridge
        bridge_t *bridge;
        bridgereg_t timeout_reg;
        char name[256];

        hwgraph_vertex_name_get(soft->pci_vhdl,
                                name,
                                sizeof(name));
        cmn_err(CE_WARN, "PCI vertex is %s", name);

        xconn_vhdl = device_master_get(soft->pci_vhdl);

        hwgraph_vertex_name_get(xconn_vhdl,
                                name,
                                sizeof(name));
        cmn_err(CE_WARN, "PCI parent vertex is %s", name);

        if (soft->pcie_bridge_vhdl) {
            hwgraph_vertex_name_get(soft->pcie_bridge_vhdl,
                                    name,
                                    sizeof(name));
            cmn_err(CE_WARN, "PCI bridge vertex is %s", name);

            xconn_vhdl = device_master_get(soft->pcie_bridge_vhdl);

            hwgraph_vertex_name_get(xconn_vhdl,
                                    name,
                                    sizeof(name));
            cmn_err(CE_WARN, "PCI bridge parent vertex is %s", name);
        }

#if 0        
        bridge = (bridge_t *)
                xtalk_piotrans_addr(xconn_vhdl, NULL,
                                    0, sizeof (bridge_t), 0);
        if (!bridge)
            cmn_err(CE_WARN, "nvme: Could not map bridge registers");


        timeout_reg = bridge->b_bus_timeout;
        timeout_reg &= ~BRIDGE_BUS_PCI_RETRY_MASK;
        timeout_reg |= BRIDGE_BUS_PCI_RETRY_CNT(0x3FF);
        bridge->b_bus_timeout = timeout_reg;
#endif        
    }
#endif

    soft->cap = (__uint64_t)NVME_RD(soft, NVME_REG_CAP);          /* CAP low */
    soft->cap |= ((__uint64_t)NVME_RD(soft, NVME_REG_CAP + 4)) << 32;  /* CAP high */
    soft->vs = NVME_RD(soft, NVME_REG_VS);

    /* Parse MQES (Maximum Queue Entries Supported) - 0-based, so add 1 */
    soft->max_queue_entries = (uint_t)((soft->cap & NVME_CAP_MQES_MASK) + 1);

    soft->min_page_size = (uint_t)((soft->cap >> 48) & 0xF);
    soft->max_page_size = (uint_t)((soft->cap >> 52) & 0xF);
#ifdef NVME_ATTACH_VERBOSE
    cmn_err(CE_NOTE, "nvme: system page size %u supported range %u-%u",
            NBPP, 1u << (soft->min_page_size + 12), 1u << (soft->max_page_size + 12));
#endif
#ifdef NVME_FORCE_4K
        soft->nvme_page_size = 4096;
        soft->nvme_page_shift = 12;
#else            
    if (PAGE_SHIFT < soft->min_page_size + 12 || PAGE_SHIFT > soft->max_page_size + 12) {
        cmn_err(CE_WARN, "nvme: system page size %u outside supported range %u-%u",
                NBPP, 1u << (soft->min_page_size + 12), 1u << (soft->max_page_size + 12));
        soft->nvme_page_shift = soft->min_page_size + 12;        
        soft->nvme_page_size = 1u << soft->nvme_page_shift;
    } else {
        soft->nvme_page_size = NBPP;
        soft->nvme_page_shift = PAGE_SHIFT;
    }
#endif    
    soft->nvme_prp_entries = soft->nvme_page_size >> 3; // prp entry is 8 bytes 

    /* Calculate doorbell stride in bytes */
    soft->doorbell_stride = 4 << (((soft->cap >> 32) & NVME_CAP_DSTRD_MASK));
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: CAP=%x MQES=%u DBS=%u",
            (uint_t)soft->cap, soft->max_queue_entries, soft->doorbell_stride);
#endif
    /*
     * Determine queue size - use minimum of our max and controller's max
     * Must be power of 2
     */
    queue_size = NVME_ADMIN_QUEUE_SIZE;
    if (queue_size > soft->max_queue_entries) {
        queue_size = soft->max_queue_entries;
    }

    /*
     * Allocate admin queue (both SQ and CQ)
     * Use uncached, physically contiguous memory
     */
    pages = (uint)btoc(queue_size * NVME_SQ_ENTRY_SIZE);

    /* Allocate submission queue */
    soft->admin_queue.sq = (nvme_command_t *)kvpalloc(pages,
                                                       VM_UNCACHED | VM_PHYSCONTIG | VM_DIRECT | VM_NOSLEEP,
                                                       0);
    if (!soft->admin_queue.sq) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to allocate admin SQ");
#endif
        goto err_out;
    }
    bzero(soft->admin_queue.sq, pages * NBPP);


    /* Get physical address for SQ - use pciio_dmatrans_addr for proper PCI DMA mapping */
    soft->admin_queue.sq_phys = pciio_dmatrans_addr(soft->pci_vhdl, 0,
                                                    kvtophys(soft->admin_queue.sq),
                                                    queue_size * NVME_SQ_ENTRY_SIZE,
                                                    PCIIO_DMA_CMD | DMATRANS64 | QUEUE_SWAP);

    /* Allocate completion queue */
    pages = (uint)btoc(queue_size * NVME_CQ_ENTRY_SIZE);
    soft->admin_queue.cq = (nvme_completion_t *)kvpalloc(pages,
                                                          VM_UNCACHED | VM_PHYSCONTIG | VM_DIRECT | VM_NOSLEEP,
                                                          0);
    if (!soft->admin_queue.cq) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to allocate admin CQ");
#endif
        goto err_free_admin_sq;
    }
    bzero(soft->admin_queue.cq, pages * NBPP);

    /* Get physical address for CQ - use pciio_dmatrans_addr for proper PCI DMA mapping */
    soft->admin_queue.cq_phys = pciio_dmatrans_addr(soft->pci_vhdl, 0,
                                                    kvtophys(soft->admin_queue.cq),
                                                    queue_size * NVME_CQ_ENTRY_SIZE,
                                                    PCIIO_DMA_CMD | DMATRANS64 | QUEUE_SWAP);

    /* Initialize queue state */
    soft->admin_queue.qid = 0;
    soft->admin_queue.size = queue_size;
    soft->admin_queue.size_mask = queue_size - 1;
    soft->admin_queue.size_shift = nvme_log2(queue_size);  /* For phase bit extraction */
    soft->admin_queue.sq_head = 0;
    soft->admin_queue.sq_tail = 0;
    soft->admin_queue.cq_head = soft->admin_queue.size;  /* Start with phase = 1 */

    /* Calculate doorbell register addresses (qid=0 for admin) */
    soft->admin_queue.sq_doorbell = 0x1000 + (2 * 0 * soft->doorbell_stride);  /* = 0x1000 */
    soft->admin_queue.cq_doorbell = 0x1000 + ((2 * 0 + 1) * soft->doorbell_stride);
    soft->admin_queue.cpl_handler = nvme_handle_admin_completion;
    soft->admin_queue.outstanding = 0;
    soft->admin_queue.watchdog_id = 0;
    soft->admin_queue.watchdog_active = 0;

    init_mutex(&soft->admin_queue.lock, MUTEX_DEFAULT, "nvme_admin", 0);

    /*
     * Allocate I/O queue
     */
    queue_size = NVME_IO_QUEUE_SIZE;
    if (queue_size > soft->max_queue_entries) {
        queue_size = soft->max_queue_entries;
    }

    /* Allocate I/O submission queue */
    pages = (uint)btoc(queue_size * NVME_SQ_ENTRY_SIZE);
    soft->io_queue.sq = (nvme_command_t *)kvpalloc(pages,
                                                    VM_UNCACHED | VM_PHYSCONTIG | VM_DIRECT | VM_NOSLEEP,
                                                    0);
    if (!soft->io_queue.sq) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to allocate I/O SQ");
#endif
        goto err_free_admin_cq;
    }
    bzero(soft->io_queue.sq, pages * NBPP);

    /* Get physical address for I/O SQ - use pciio_dmatrans_addr for proper PCI DMA mapping */
    soft->io_queue.sq_phys = pciio_dmatrans_addr(soft->pci_vhdl, 0,
                                                  kvtophys(soft->io_queue.sq),
                                                  queue_size * NVME_SQ_ENTRY_SIZE,
                                                  PCIIO_DMA_CMD | DMATRANS64 | QUEUE_SWAP);

    /* Allocate I/O completion queue */
    pages = (uint)btoc(queue_size * NVME_CQ_ENTRY_SIZE);
    soft->io_queue.cq = (nvme_completion_t *)kvpalloc(pages,
                                                       VM_UNCACHED | VM_PHYSCONTIG | VM_DIRECT | VM_NOSLEEP,
                                                       0);
    if (!soft->io_queue.cq) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to allocate I/O CQ");
#endif
        goto err_free_io_sq;
    }
    bzero(soft->io_queue.cq, pages * NBPP);

    /* Get physical address for I/O CQ - use pciio_dmatrans_addr for proper PCI DMA mapping */
    soft->io_queue.cq_phys = pciio_dmatrans_addr(soft->pci_vhdl, 0,
                                                  kvtophys(soft->io_queue.cq),
                                                  queue_size * NVME_CQ_ENTRY_SIZE,
                                                  PCIIO_DMA_CMD | DMATRANS64 | QUEUE_SWAP);

    /* Initialize I/O queue state */
    soft->io_queue.qid = 1;
    soft->io_queue.size = queue_size;
    soft->io_queue.size_mask = queue_size - 1;
    soft->io_queue.size_shift = nvme_log2(queue_size);  /* For phase bit extraction */
    soft->io_queue.sq_head = 0;
    soft->io_queue.sq_tail = 0;
    soft->io_queue.cq_head = soft->io_queue.size;  /* Start with phase = 1 */

    /* Calculate doorbell register addresses (qid=1 for I/O) */
    soft->io_queue.sq_doorbell = 0x1000 + (2 * 1 * soft->doorbell_stride);
    soft->io_queue.cq_doorbell = 0x1000 + ((2 * 1 + 1) * soft->doorbell_stride);
    soft->io_queue.vector = 0;
    soft->io_queue.cpl_handler = nvme_handle_io_completion;
    soft->io_queue.outstanding = 0;
    soft->io_queue.watchdog_id = 0;
    soft->io_queue.watchdog_active = 0;

    init_mutex(&soft->io_queue.lock, MUTEX_DEFAULT, "nvme_io", 0);

    /*
     * Initialize I/O command tracking
     * (io_requests and io_cid_bitmap already zeroed by kmem_zalloc)
     */
    /*
     * CIDs usable by the SCSI path.  Never more than the SQ can hold
     * (size - 1) minus a reserve for internal commands, so that a request
     * whose CIDs were reserved can always be submitted.
     */
    soft->io_cid_max = soft->io_queue.size - 1 - NVME_IO_CID_RESERVE;
    if (soft->io_cid_max > NVME_IO_QUEUE_SIZE)
        soft->io_cid_max = NVME_IO_QUEUE_SIZE;
    soft->io_cid_free_count = soft->io_cid_max;
    init_mutex(&soft->io_requests_lock, MUTEX_DEFAULT, "nvme_io_cid", 0);

    /* Deferred request ring + dispatcher state */
    nvme_defer_init(soft);
    bzero(&soft->stats, sizeof(soft->stats));
    soft->stats.io_timeout_sec = NVME_IO_TIMEOUT_MIN_SEC;
    soft->abort_outstanding = 0;
    soft->abort_limit = NVME_ABORT_MAX_DEFAULT;
    soft->shutting_down = 0;

    /*
     * Initialize PRP pool for I/O operations
     */
    if (nvme_prp_pool_init(soft) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to initialize PRP pool");
#endif
        goto err_destroy_io_locks;
    }

    /*
     * Initialize pre-allocated alenlist for address/length conversions
     * Pre-grow it to v.v_maxdmasz (max DMA size in pages) to avoid dynamic allocation failures
     */
    soft->alenlist = alenlist_create(0);
    if (!soft->alenlist) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to create alenlist");
#endif
        goto err_free_prp_pool;
    }
    if (alenlist_grow(soft->alenlist, v.v_maxdmasz * (NBPP / soft->nvme_page_size)) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to grow alenlist to %d pages", v.v_maxdmasz);
#endif
        alenlist_destroy(soft->alenlist);
        soft->alenlist = NULL;
        goto err_free_prp_pool;
    }
    init_mutex(&soft->alenlist_lock, MUTEX_DEFAULT, "nvme_alenlist", 0);
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: pre-allocated alenlist for %d pages", v.v_maxdmasz);
#endif

    /*
     * Initialize aborted command FIFO for retry detection
     * (aborted_cmds array already zeroed by kmem_zalloc)
     */
    soft->aborted_head = 0;
    soft->aborted_bitmap = 0;
    init_mutex(&soft->aborted_lock, MUTEX_DEFAULT, "nvme_aborted", 0);
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: initialized aborted command FIFO");
#endif
    /*
     * Allocate utility buffer (1 page for admin commands during init)
     * This will be used for Identify commands
     */
    soft->utility_buffer = kvpalloc(1, VM_UNCACHED | VM_PHYSCONTIG | VM_DIRECT | VM_NOSLEEP, 0);
    if (!soft->utility_buffer) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme: failed to allocate utility buffer");
#endif
        goto err_free_alenlist;
    }
#ifdef NVME_UTILBUF_USEDMAP
    soft->utility_buffer_dmamap = pciio_dmamap_alloc(soft->pci_vhdl, NULL,
                                                     NBPP,
                                                     UTILBUF_DMA_TYPE);
    soft->utility_buffer_phys = pciio_dmamap_addr(soft->utility_buffer_dmamap,
                                                  kvtophys(soft->utility_buffer),
                                                  NBPP);

#else    
    /* Get physical address for utility buffer - use pciio_dmatrans_addr for proper PCI DMA mapping */
    soft->utility_buffer_phys = pciio_dmatrans_addr(soft->pci_vhdl, 0,
                                                     kvtophys(soft->utility_buffer),
                                                     NBPP,  /* 1 page */
                                                     UTILBUF_DMA_TYPE | DMATRANS64);
#endif
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: allocated admin queue (size=%u, shift=%u) at phys SQ=%llx CQ=%llx",
            soft->admin_queue.size, soft->admin_queue.size_shift,
            soft->admin_queue.sq_phys, soft->admin_queue.cq_phys);
    cmn_err(CE_NOTE, "nvme: allocated I/O queue (size=%u, shift=%u) at phys SQ=%llx CQ=%llx",
            soft->io_queue.size, soft->io_queue.size_shift,
            soft->io_queue.sq_phys, soft->io_queue.cq_phys);
#endif
    /*
     * Configure controller
     */

    /* Set admin queue attributes: ACQS (15:0) and ASQS (31:16), both 0-based */
    aqa = ((soft->admin_queue.size - 1) << 16) | (soft->admin_queue.size - 1);
    NVME_WR(soft, NVME_REG_AQA, aqa);

    /* Set admin queue addresses (write low and high separately) */
    phys_addr = soft->admin_queue.sq_phys;
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: Writing ASQ address 0x%llx (lo=0x%08x, hi=0x%08x)",
            phys_addr, PHYS64_LO(phys_addr), PHYS64_HI(phys_addr));
    cmn_err(CE_NOTE, "nvme: ASQ write addresses: lo=%p (offset 0x%x), hi=%p (offset 0x%x)",
            soft->bar0 + NVME_REG_ASQ, NVME_REG_ASQ,
            soft->bar0 + NVME_REG_ASQ + 4, NVME_REG_ASQ + 4);
#endif
    NVME_WR(soft, NVME_REG_ASQ, PHYS64_LO(phys_addr));
    NVME_WR(soft, NVME_REG_ASQ + 4, PHYS64_HI(phys_addr));
#ifdef NVME_DBG
    /* Read back to verify */
    cmn_err(CE_NOTE, "nvme: ASQ readback: lo=0x%08x, hi=0x%08x",
            NVME_RD(soft, NVME_REG_ASQ), NVME_RD(soft, NVME_REG_ASQ + 4));
#endif
    phys_addr = soft->admin_queue.cq_phys;
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: Writing ACQ address 0x%llx (lo=0x%08x, hi=0x%08x)",
            phys_addr, PHYS64_LO(phys_addr), PHYS64_HI(phys_addr));
    cmn_err(CE_NOTE, "nvme: ACQ write addresses: lo=%p (offset 0x%x), hi=%p (offset 0x%x)",
            soft->bar0 + NVME_REG_ACQ, NVME_REG_ACQ,
            soft->bar0 + NVME_REG_ACQ + 4, NVME_REG_ACQ + 4);
#endif
    NVME_WR(soft, NVME_REG_ACQ, PHYS64_LO(phys_addr));
    NVME_WR(soft, NVME_REG_ACQ + 4, PHYS64_HI(phys_addr));
#ifdef NVME_DBG
    /* Read back to verify */
    cmn_err(CE_NOTE, "nvme: ACQ readback: lo=0x%08x, hi=0x%08x",
            NVME_RD(soft, NVME_REG_ACQ), NVME_RD(soft, NVME_REG_ACQ + 4));
#endif
    /*
     * Configure and enable controller
     * MPS = PAGE_SHIFT-12 (match system page size)
     * CSS = 0 (NVM command set)
     * AMS = 0 (round robin arbitration)
     * SHN = 0 (no shutdown)
     * IOSQES = 6 (64 byte submission queue entry)
     * IOCQES = 4 (16 byte completion queue entry)
     */
    cc = NVME_CC_ENABLE |
         ((soft->nvme_page_shift - 12) << NVME_CC_MPS_SHIFT) |
         NVME_CC_CSS_NVM |
         NVME_CC_AMS_RR |
         NVME_CC_SHN_NONE |
         NVME_CC_IOSQES |
         NVME_CC_IOCQES;

    NVME_WR(soft, NVME_REG_CC, cc);

    /*  Wait for controller to become ready (timeout from CAP.TO) */
    if (nvme_wait_for_ready(soft, 1, 60000) != 0) {
        cmn_err(CE_WARN, "nvme: controller failed to become ready");
        goto err_free_utility_buffer;
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: controller is ready");
#endif
    /*
     * NOTE: Interrupts stay masked during init
     * We will use polling for admin commands
     */

#ifdef NVME_DBG_EXTRA
    /* Dump controller state before first admin command */
    nvme_dump_controller_state(soft, "Before first admin command (Identify Controller)");
#endif /* NVME_DBG_EXTRA */

    if (!nvme_admin_identify_controller(soft)) {
        goto err_free_utility_buffer;
    }

#ifdef NVME_DBG_EXTRA
    /* Dump state right after submitting first command */
    nvme_dump_controller_state(soft, "After submitting Identify Controller command");
#endif /* NVME_DBG_EXTRA */
#ifndef NVME_COMPLETION_MANUAL
    nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
#endif
    if (!nvme_admin_identify_namespace(soft)) {
        goto err_free_utility_buffer;
    }
#ifndef NVME_COMPLETION_MANUAL
    nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
#endif
    if (!nvme_admin_create_cq(soft, soft->io_queue.qid, soft->io_queue.size,
                              soft->io_queue.cq_phys, soft->io_queue.vector)) {
        goto err_free_utility_buffer;
    }
#ifndef NVME_COMPLETION_MANUAL
    nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
#endif
    if (!nvme_admin_create_sq(soft, soft->io_queue.qid, soft->io_queue.size,
                              soft->io_queue.sq_phys, soft->io_queue.qid)) {
        goto err_free_utility_buffer;
    }
#ifndef NVME_COMPLETION_MANUAL
    nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
#endif

    /* Query controller features to discover capabilities */
    if (!nvme_admin_query_features(soft)) {
        cmn_err(CE_WARN, "nvme_attach: failed to query controller features (continuing anyway)");
        /* Non-fatal - continue initialization even if feature query fails */
    }

    /*
     * Disable the volatile write cache (FID 0x06, WCE bit = 0).
     *
     * IRIX does not call any driver entry point during a clean system
     * shutdown (init 0 / halt).  After rc0.d unmounts filesystems the
     * kernel jumps directly to the PROM halt call without notifying PCI
     * or SCSI drivers.  This means there is no opportunity to issue a
     * Flush command before power is removed.
     *
     * With the write cache enabled (the factory default), the controller
     * acknowledges Write commands as soon as data lands in its DRAM
     * cache.  If power is cut before the drive's internal GC/flush writes
     * that DRAM data to NAND, any filesystem metadata written during
     * unmount (superblock, journal tail, inode table) is silently lost,
     * producing the observed filesystem corruption.
     *
     * With WCE = 0 the controller may only return a Write completion
     * after the data has been committed to non-volatile NAND storage.
     * This is equivalent to every write being FUA (Force Unit Access).
     * Power can now be removed at any point without losing acknowledged
     * write data.
     *
     * The setting is not saved to the controller's non-volatile store
     * (SVE bit not set in CDW10), so it applies only for this power
     * cycle and resets to the factory default on next power-on.  The
     * driver re-applies it on every attach.
     *
     * Note: soft->features[NVME_FEAT_VOLATILE_WRITE_CACHE] is non-zero
     * only when the drive actually has a write cache AND permits the
     * host to toggle it (Get Features SEL=SUPPORTED returned bit 0 set).
     * Drives without a write cache, or with a fixed-always-on cache,
     * report 0 here; we warn but continue (such drives typically have
     * hardware power-loss protection of their own).
     */

    /* Track current VWC state. NVMe default is enabled; the DISABLE_WC block
     * below clears this flag when it successfully disables the cache. */
    soft->vwc_enabled = (soft->features[NVME_FEAT_VOLATILE_WRITE_CACHE] ? 1 : 0);

#if defined(NVME_DISABLE_WRITE_CACHE) && defined(IP32) && defined(NVME_BUILTIN)
    if (soft->features[NVME_FEAT_VOLATILE_WRITE_CACHE]) {
        if (nvme_admin_set_features(soft, NVME_FEAT_VOLATILE_WRITE_CACHE, 0)) {
            nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
            soft->vwc_enabled = 0;
            cmn_err(CE_NOTE, "nvme: volatile write cache DISABLED — "
                    "writes are now synchronous with NAND (safe for power-off)");
        } else {
            cmn_err(CE_WARN, "nvme: Set Features(VWC=0) failed — "
                    "filesystem corruption on unclean power-off remains possible");
        }
    } else {
        cmn_err(CE_WARN, "nvme: volatile write cache not host-controllable "
                "(features[VWC]=0x%x) — drive may have fixed cache or built-in PLP",
                soft->features[NVME_FEAT_VOLATILE_WRITE_CACHE]);
    }
#endif /* NVME_DISABLE_WRITE_CACHE */

#if defined(NVME_FUA_WRITES) && defined(IP32) && defined(NVME_BUILTIN)
    soft->fua_enabled = 1;
    cmn_err(CE_NOTE, "nvme: FUA mode active — Force Unit Access bit set on all "
            "Write commands, writes committed to NAND before completion");
#endif /* NVME_FUA_WRITES */

    /* Configure interrupt coalescing if supported
     * Coalesce after 10 completions OR 500 microseconds (whichever comes first)
     * Time calculation: 10 × 4KB reads over 33MHz PCI ≈ 400us, use 500us for margin
     * CDW11 format: Threshold (7:0), Time (31:8) in 100us units */
    if (soft->features[NVME_FEAT_INTERRUPT_COALESCING]) {
        uint_t coalesce_value = 10 | (5 << 8);  /* 10 completions, 500us (5 × 100us) */

        if (nvme_admin_set_features(soft, NVME_FEAT_INTERRUPT_COALESCING, coalesce_value)) {
#ifndef NVME_COMPLETION_MANUAL
            nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
#endif
#ifdef NVME_ATTACH_VERBOSE
            cmn_err(CE_NOTE, "nvme: Interrupt coalescing configured (10 completions, 500us)");
#endif
        } else {
            cmn_err(CE_WARN, "nvme: Failed to configure interrupt coalescing");
        }
    }

    soft->initialized = 1;
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "!nvme_attach: set soft=%p initialized=1", soft);
#endif
    return 0;

    /* Error cleanup path - free resources in reverse order of allocation */
err_free_utility_buffer:
    if (soft->utility_buffer) {
        kvpfree(soft->utility_buffer, 1);
        soft->utility_buffer = NULL;
    }

err_free_alenlist:
    if (soft->alenlist) {
        mutex_destroy(&soft->alenlist_lock);
        alenlist_destroy(soft->alenlist);
        soft->alenlist = NULL;
    }

err_free_prp_pool:
    nvme_prp_pool_done(soft);

err_destroy_io_locks:
    nvme_defer_destroy(soft);
    mutex_destroy(&soft->io_requests_lock);
    mutex_destroy(&soft->io_queue.lock);

    if (soft->io_queue.cq) {
        kvpfree(soft->io_queue.cq, (uint)btoc(soft->io_queue.size * NVME_CQ_ENTRY_SIZE));
        soft->io_queue.cq = NULL;
    }

err_free_io_sq:
    if (soft->io_queue.sq) {
        kvpfree(soft->io_queue.sq, (uint)btoc(soft->io_queue.size * NVME_SQ_ENTRY_SIZE));
        soft->io_queue.sq = NULL;
    }

err_free_admin_cq:
    mutex_destroy(&soft->admin_queue.lock);
    if (soft->admin_queue.cq) {
        kvpfree(soft->admin_queue.cq, (uint)btoc(soft->admin_queue.size * NVME_CQ_ENTRY_SIZE));
        soft->admin_queue.cq = NULL;
    }

err_free_admin_sq:
    if (soft->admin_queue.sq) {
        kvpfree(soft->admin_queue.sq, (uint)btoc(soft->admin_queue.size * NVME_SQ_ENTRY_SIZE));
        soft->admin_queue.sq = NULL;
    }

err_out:
    return -1;
}

/*
 * nvme_wait_for_queue_idle: Wait for queue to drain all outstanding commands
 *
 * Polls the completion queue until all outstanding commands complete or timeout.
 * Returns 0 on success (queue idle), -1 on timeout.
 */
int
nvme_wait_for_queue_idle(nvme_soft_t *soft, nvme_queue_t *q, uint_t timeout_ms)
{
    int elapsed_ms = 0;
    int processed;
    int outstanding;

    /* Read outstanding counter atomically */
    outstanding = atomicAddInt((int *)&q->outstanding, 0);

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_wait_for_queue_idle: waiting for queue %d (outstanding=%d, timeout=%dms)",
            q->qid, outstanding, timeout_ms);
#endif

    /* Poll completions until queue is idle or timeout */
    while (outstanding > 0 && elapsed_ms < timeout_ms) {
        processed = nvme_process_completions(soft, q);

#ifdef NVME_DBG_EXTRA
        if (processed > 0) {
            cmn_err(CE_NOTE, "nvme_wait_for_queue_idle: processed %d completions (outstanding=%d)",
                    processed, outstanding);
        }
#endif

        /* Re-read outstanding counter atomically */
        outstanding = atomicAddInt((int *)&q->outstanding, 0);

        if (outstanding == 0) {
            break;
        }

        /* Sleep 1ms between polls */
        us_delay(1000);
        elapsed_ms++;
    }

    if (outstanding > 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_wait_for_queue_idle: timeout waiting for queue %d (still %d outstanding)",
                q->qid, outstanding);
#endif
        return -1;
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_wait_for_queue_idle: queue %d is idle (took %dms)", q->qid, elapsed_ms);
#endif
    return 0;
}

/*
 * nvme_shutdown: Shutdown NVMe controller cleanly
 */
static int
nvme_shutdown_ex(nvme_soft_t *soft, int fast_alias)
{
    uint_t cc, csts;
    int timeout;
    uint_t io_wait_ms;
    uint_t admin_wait_ms;
    uint_t disable_wait_ms;
    uint_t cap_to_ms;

    cmn_err(CE_NOTE, "nvme: shutdown started — adapter %d [%.40s / SN %.20s]%s",
            soft->adap, (char *)soft->model, (char *)soft->serial,
            fast_alias ? " (fast alias path)" : "");

    /* Stop timeout watchdog - no more commands should time out */
    nvme_timeout_watchdog_stop(soft);

    /* Nothing new gets started; fail whatever is still parked */
    soft->shutting_down = 1;
    nvme_defer_fail_all(soft);

    io_wait_ms = fast_alias ? 100 : 5000;
    admin_wait_ms = fast_alias ? 100 : 5000;
    disable_wait_ms = fast_alias ? 250 : 5000;

    /* Wait for any in-flight I/O commands to complete */
    cmn_err(CE_NOTE, "nvme: draining I/O queue (outstanding: %d, timeout: %ums)",
            atomicAddInt((int *)&soft->io_queue.outstanding, 0), io_wait_ms);

    if (nvme_wait_for_queue_idle(soft, &soft->io_queue, io_wait_ms) == 0) {
        cmn_err(CE_NOTE, "nvme: I/O queue drained");
    } else {
        cmn_err(CE_WARN, "nvme: I/O queue drain timed out (%d commands still outstanding)",
                atomicAddInt((int *)&soft->io_queue.outstanding, 0));
    }

    /* Stop completion watchdog timers */
    nvme_watchdog_stop(&soft->io_queue);
    nvme_watchdog_stop(&soft->admin_queue);

    if (!fast_alias) {
        /*
         * Issue a Flush command before deleting the I/O queues.
         *
         * This forces the NVMe controller to commit all data from its
         * volatile write-back cache to non-volatile media (NAND flash)
         * while the I/O queue is still intact and operational.  Without
         * this step, any write data acknowledged to the host but not yet
         * persisted in NAND can be lost when power is removed, resulting
         * in filesystem corruption.
         *
         * We poll for completion rather than relying on interrupts because
         * this path may be reached with interrupts already masked.
         */
        cmn_err(CE_NOTE, "nvme: issuing Flush to commit volatile write cache to NAND");
        if (nvme_cmd_special_flush(soft) == 0) {
            if (nvme_wait_for_queue_idle(soft, &soft->io_queue, io_wait_ms) == 0) {
                cmn_err(CE_NOTE, "nvme: Flush complete");
            } else {
                cmn_err(CE_WARN, "nvme: Flush timed out — write cache may not be fully committed");
            }
        } else {
            cmn_err(CE_WARN, "nvme: Flush submission failed, proceeding anyway");
        }

        /*
         * Delete I/O queues using admin commands (must happen BEFORE disabling controller)
         * This is the proper NVMe shutdown sequence:
         * 1. Delete I/O Submission Queue
         * 2. Wait for completion
         * 3. Delete I/O Completion Queue
         * 4. Wait for completion
         */
        cmn_err(CE_NOTE, "nvme: deleting I/O submission queue (qid %d)", soft->io_queue.qid);
        if (nvme_admin_delete_sq(soft, soft->io_queue.qid)) {
            nvme_wait_for_queue_idle(soft, &soft->admin_queue, admin_wait_ms);
            cmn_err(CE_NOTE, "nvme: I/O submission queue deleted");
        } else {
            cmn_err(CE_WARN, "nvme: failed to delete I/O submission queue");
        }

        cmn_err(CE_NOTE, "nvme: deleting I/O completion queue (qid %d)", soft->io_queue.qid);
        if (nvme_admin_delete_cq(soft, soft->io_queue.qid)) {
            nvme_wait_for_queue_idle(soft, &soft->admin_queue, admin_wait_ms);
            cmn_err(CE_NOTE, "nvme: I/O completion queue deleted");
        } else {
            cmn_err(CE_WARN, "nvme: failed to delete I/O completion queue");
        }

        /*
         * Wait for shutdown to complete using the controller's own declared
         * timeout (CAP.TO, bits [31:24], in units of 500ms).  The previous
         * hardcoded 1-second limit was too short for drives that need up to
         * several seconds to flush their DRAM write cache to NAND flash;
         * timing out early caused the controller to be forcibly disabled
         * before the flush completed, losing data identical to a surprise
         * power-loss event.  Use a minimum of 5 seconds regardless of CAP.TO.
         */
        cap_to_ms = (uint_t)((soft->cap >> 24) & 0xFF) * 500;
        if (cap_to_ms < 5000)
            cap_to_ms = 5000;

        cmn_err(CE_NOTE, "nvme: setting CC.SHN=Normal, waiting up to %ums for CSTS.SHST=Complete"
                " (CAP.TO=%u × 500ms)",
                cap_to_ms, (uint_t)((soft->cap >> 24) & 0xFF));

        /* Request clean shutdown via CC register */
        cc = NVME_RD(soft, NVME_REG_CC);
        cc &= ~NVME_CC_SHN_MASK;
        cc |= NVME_CC_SHN_NORMAL;
        NVME_WR(soft, NVME_REG_CC, cc);

        timeout = (int)cap_to_ms;  /* 1 iteration = 1ms via us_delay(1000) */
        while (timeout > 0) {
            csts = NVME_RD(soft, NVME_REG_CSTS);
            if ((csts & NVME_CSTS_SHST_MASK) == NVME_CSTS_SHST_COMPLETE) {
                break;
            }
            us_delay(1000);
            timeout--;
        }

        if (timeout == 0) {
            cmn_err(CE_WARN, "nvme: CSTS.SHST=Complete not seen after %ums — "
                    "forcing disable (CSTS=0x%x)", cap_to_ms,
                    NVME_RD(soft, NVME_REG_CSTS));
        } else {
            cmn_err(CE_NOTE, "nvme: shutdown complete in %ums (CSTS=0x%x)",
                    cap_to_ms - (uint_t)timeout,
                    NVME_RD(soft, NVME_REG_CSTS));
        }
    }

    /* Disable controller */
    cmn_err(CE_NOTE, "nvme: disabling controller (CC.EN=0)");
    cc = NVME_RD(soft, NVME_REG_CC);
    cc &= ~NVME_CC_ENABLE;
    NVME_WR(soft, NVME_REG_CC, cc);

    /* Wait for controller to become not ready */
    if (nvme_wait_for_ready(soft, 0, disable_wait_ms) == 0) {
        cmn_err(CE_NOTE, "nvme: controller disabled (CSTS.RDY=0)");
    } else {
        cmn_err(CE_WARN, "nvme: controller did not clear RDY within %ums", disable_wait_ms);
    }

    /* Free utility buffer */
    if (soft->utility_buffer) {
        kvpfree(soft->utility_buffer, 1);
        soft->utility_buffer = NULL;
    }

    /* Free alenlist */
    if (soft->alenlist) {
        mutex_destroy(&soft->alenlist_lock);
        alenlist_destroy(soft->alenlist);
        soft->alenlist = NULL;
    }

    /* Free PRP pool */
    nvme_prp_pool_done(soft);

    /* Destroy I/O command tracking lock and defer ring */
    nvme_defer_destroy(soft);
    mutex_destroy(&soft->io_requests_lock);

    /* Destroy aborted command tracking lock */
    mutex_destroy(&soft->aborted_lock);
#ifdef NVME_UTILBUF_USEDMAP
    pciio_dmamap_free(soft->utility_buffer_dmamap);
#endif

    /* Free I/O queue */
    if (soft->io_queue.sq) {
        kvpfree(soft->io_queue.sq, (uint)btoc(soft->io_queue.size * NVME_SQ_ENTRY_SIZE));
        soft->io_queue.sq = NULL;
    }
    if (soft->io_queue.cq) {
        kvpfree(soft->io_queue.cq, (uint)btoc(soft->io_queue.size * NVME_CQ_ENTRY_SIZE));
        soft->io_queue.cq = NULL;
    }
    mutex_destroy(&soft->io_queue.lock);

    /* Free admin queue */
    if (soft->admin_queue.sq) {
        kvpfree(soft->admin_queue.sq, (uint)btoc(soft->admin_queue.size * NVME_SQ_ENTRY_SIZE));
        soft->admin_queue.sq = NULL;
    }
    if (soft->admin_queue.cq) {
        kvpfree(soft->admin_queue.cq, (uint)btoc(soft->admin_queue.size * NVME_CQ_ENTRY_SIZE));
        soft->admin_queue.cq = NULL;
    }
    mutex_destroy(&soft->admin_queue.lock);

    return 0;
}

static int
nvme_shutdown(nvme_soft_t *soft)
{
    return nvme_shutdown_ex(soft, 0);
}

/*
 * nvme_intr: interrupt handler
 *
 * Called when NVMe device generates an interrupt (completion queue not empty).
 * Process both admin and I/O completion queues.
 */

volatile int nvme_intcount = 0;

/*ARGSUSED*/
void
nvme_intr(
#if defined(IP32)
    eframe_t *ep, 
#endif
    intr_arg_t arg)
{
    int admin_processed, io_processed;
    nvme_soft_t *soft = (nvme_soft_t *)arg;

    if (!soft || !soft->initialized) {
        return;
    }
    /* Process admin queue completions */
    admin_processed = nvme_process_completions(soft, &soft->admin_queue);

    /* Process I/O queue completions */
    io_processed = nvme_process_completions(soft, &soft->io_queue);

#ifdef NVME_DBG_EXTRA
    /* Debug: log if we actually processed something */
    if (admin_processed || io_processed) {
        cmn_err(CE_DEBUG, "nvme_intr: processed %d admin, %d I/O completions",
                admin_processed, io_processed);
    }
#endif
}

static int
nvme_enable_interrupts(nvme_soft_t *soft)
{
    ushort_t command;
    uchar_t intr_pin;
    pciio_intr_line_t intr_line;
    device_desc_t dev_desc;
    pciio_info_t pciioinfo = pciio_info_get(soft->pci_vhdl);
    vertex_hdl_t intr_conn = soft->pci_vhdl;

    /*
        if we enumerated pci-pcie bridge it means we are on older irix (6.5.22?)
        that doesn't understand bridges. we will attempt to hook up the interrupts 
        to the bridge. 6.5.30 will just tell us to attach to unclaimed device and
        seems to understand bridges so pcie_bridge_vhdl will end up 0 and hooking
        up interrupts directly to pci_vhdl will work fine.
    */
    if (soft->pcie_bridge_vhdl) {
#ifdef IP32
        intr_conn = soft->pcie_bridge_vhdl;
#endif
#ifdef IP30
        intr_conn = soft->pcie_bridge_vhdl;
#endif
    }
    pciioinfo = pciio_info_get(intr_conn);
    soft->interrupts_enabled = 0;
    intr_pin = (uchar_t)pciio_config_get(soft->pci_vhdl, PCI_INTR_PIN, 1);

    /*
     * Setup interrupts
     * Allocate and connect interrupt only if device has an interrupt pin configured
     * The pin value (1-4) maps to INTA-INTD (lines A-D)
     */
    if (intr_pin == 0)
        return 0;

    /* Map interrupt pin (1-4) to interrupt line (A-D) */
    switch (intr_pin) {
    case 1: intr_line = PCIIO_INTR_LINE_A; break;
    case 2: intr_line = PCIIO_INTR_LINE_B; break;
    case 3: intr_line = PCIIO_INTR_LINE_C; break;
    case 4: intr_line = PCIIO_INTR_LINE_D; break;
    default:
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_attach: invalid interrupt pin %d", intr_pin);
#endif
        return -1;
    }

    dev_desc = device_desc_dup(intr_conn);
    device_desc_intr_name_set(dev_desc, "nvme");
#ifdef IP32
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme: IP32 interrupt Bus: %d, Slot: %d, Function: %d",
            pciio_info_bus_get(pciioinfo),
            pciio_info_slot_get(pciioinfo),
            pciio_info_function_get(pciioinfo));
#endif
    device_desc_intr_swlevel_set(dev_desc, (ilvl_t)splhintr);
#endif
    device_desc_default_set(intr_conn, dev_desc);

    soft->intr = pciio_intr_alloc(intr_conn, dev_desc, intr_line, intr_conn);
    if (!soft->intr) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_attach: failed to allocate interrupt");
#endif
        return -1;
    }

    if (pciio_intr_connect(soft->intr, (intr_func_t)nvme_intr, (intr_arg_t)soft, NULL)) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_attach: failed to connect interrupt");
#endif
        pciio_intr_free(soft->intr);
        soft->intr = NULL;
        return -1;
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_attach: interrupt connected successfully (pin %d, line %d)",
            intr_pin, intr_line);
#endif
            /* Enable interrupts in PCI command register */
    command = (ushort_t)pciio_config_get(soft->pci_vhdl, PCI_CFG_COMMAND, 2);
    command &= ~PCI_CMD_INTERRUPT_DISABLE;  /* Clear the interrupt disable bit */
    pciio_config_set(soft->pci_vhdl, PCI_CFG_COMMAND, 2, command);

    /* Unmask all NVMe interrupts (write all 1's to INTMC to clear mask) */
    NVME_WR(soft, NVME_REG_INTMC, 0xFFFFFFFF);
    soft->interrupts_enabled = 1;
    return 0;
}

static int
nvme_disable_interrupts(nvme_soft_t *soft)
{
    ushort_t command;

    if (!soft->interrupts_enabled)
        return 0;

    /* Mask all NVMe interrupts */
    NVME_WR(soft, NVME_REG_INTMS, 0xFFFFFFFF);

    /* Disable interrupts in PCI command register */
    command = (ushort_t)pciio_config_get(soft->pci_vhdl, PCI_CFG_COMMAND, 2);
    command |= PCI_CMD_INTERRUPT_DISABLE;  /* Set the interrupt disable bit */
    pciio_config_set(soft->pci_vhdl, PCI_CFG_COMMAND, 2, command);

    /* Teardown interrupts if they were allocated */
    if (soft->intr) {
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_detach: disconnecting interrupt");
#endif
        pciio_intr_disconnect(soft->intr);
        pciio_intr_free(soft->intr);
        soft->intr = NULL;
    }
    return 0;
}

static vertex_hdl_t nvme_pcie_bridge_conn = 0;
static unsigned int nvme_dev_counter = 0;

/*
 * nvme_attach: called by pciio infrastructure for each PCI device
 * (because we registered with wildcards).
 * We must check the class code to ensure this is actually an NVMe device.
 */
int
nvme_attach(vertex_hdl_t conn)
{
    static int pci_device_counter = 0;  /* Track PCI device enumeration order */
    nvme_soft_t        *soft;
    pciio_piomap_t      bar0_map = 0;
    uint_t              class_code;
    ushort_t            vendor_id, device_id;
    ushort_t            command;
    pciio_info_t        pciioinfo;
    size_t              bar0_size;
    graph_error_t       rv;
#if defined(IP32)
    char                serial_key[21];
    int                 current_slot;
#endif

#if defined(IP32)
    serial_key[0] = '\0';
    current_slot = -1;
#endif
    if (!conn) {
        cmn_err(CE_WARN, "nvme_attach: PCI device #%d conn is 0");    
        return -1;
    }
    pciioinfo = pciio_info_get(conn);
    if (pciioinfo == NULL) {
        cmn_err(CE_WARN, "nvme_attach: PCI device #%d info is NULL", pci_device_counter);
        pci_device_counter++;
        return -1;    
    } else {
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_attach: PCI device #%d %04X:%04X", pci_device_counter, (int)pciio_info_vendor_id_get(pciioinfo), (int)pciio_info_device_id_get(pciioinfo));
#endif
        if (pciio_info_vendor_id_get(pciioinfo) < 0) {
            pci_device_counter++;
            return -1;
        }
    }
    
#ifdef xxIP30
    /*
     On my Octane, after we get past last PCI dev in the shoebox (tigon in my case)
     we get bus error when calling pciio_config_get.
     Probably some weirdness of the PCI-PCIe bridge?
     So skip messing with anything after 1st NVME drive.
    */
    if (nvme_dev_counter >= 1) {
        cmn_err(CE_NOTE, "nvme_attach: PCI device #%d there be dragons", pci_device_counter);
        pci_device_counter++;
        return -1;
    }
#endif
    /* Read vendor/device ID (2 bytes each) */
    vendor_id = (ushort_t)pciio_config_get(conn, PCI_CFG_VENDOR_ID, 2);
    device_id = (ushort_t)pciio_config_get(conn, PCI_CFG_DEVICE_ID, 2);

    /* Read class code (3 bytes at offset 0x09, read as 4 bytes and mask) */
    class_code = (uint_t)pciio_config_get(conn, PCI_CFG_CLASS_CODE, 4) & 0x00FFFFFF;

    /* Log every PCI device we encounter (for debugging PCI enumeration order) */
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_attach: PCI device #%d: %04x:%04x (class 0x%06x)",
            pci_device_counter, vendor_id, device_id, class_code);
#endif
    pci_device_counter++;
#ifdef NVME_DBG_EXTRA
    /* If this is a PCI bridge, dump its configuration */
    if ((class_code >> 16) == PCI_CLASS_BRIDGE) {
        nvme_dump_pci_bridge(conn);
        return -1;  /* We don't attach to bridges */
    }
#endif /* NVME_DBG_EXTRA */

    if (class_code == PCI_BRIDGE_CLASS_CODE) {
        nvme_pcie_bridge_conn = conn;
        return -1;
    }

    /* Check if this is an NVMe controller (class 0x010802) */
    if (class_code != NVME_CLASS_CODE) {
        /* Not an NVMe device, silently ignore */
        return -1;
    }
    nvme_dev_counter++;

    /* This is an NVMe device! */
    cmn_err(CE_NOTE, "nvme_attach: found NVMe device %04x:%04x (class %06x) at conn 0x%x",
            vendor_id, device_id, class_code, conn);

#if defined(IP32)
    /*
     * Finalize pass fast-path: if we have a pre-initialized soft state saved
     * from the discovery phase for this exact conn, skip the expensive
     * nvme_sanitize() + nvme_initialize() (feature queries, queue setup, etc.)
     * and jump directly to SCSI registration.  This makes the finalize pass
     * complete in milliseconds instead of ~1.5s per controller, allowing the
     * drives to be ready before IRIX's rootfs lookup.
     */
    if (nvme_ip32_finalize_active && nvme_ip32_finalize_conn == conn &&
        nvme_ip32_finalize_soft != NULL) {
        soft = (nvme_soft_t *)nvme_ip32_finalize_soft;
        nvme_ip32_finalize_soft = NULL;
        bar0_map = soft->bar0_map;
        nvme_ip32_build_serial_key(soft->serial, serial_key, sizeof(serial_key));
        current_slot = pciio_info_slot_get(pciioinfo);
        goto nvme_attach_post_init;
    }
#endif

#ifdef NVME_DBG
    /* Dump the bridge configuration of the parent (if it's a bridge) to understand topology */
    {
        vertex_hdl_t parent_vhdl;
        char parent_path[256];

        /* Get the connect point (parent) of this device vertex */
        parent_vhdl = hwgraph_connectpt_get(conn);
        if (parent_vhdl != GRAPH_VERTEX_NONE) {
            /* Get the path for informational purposes */
            if (vertex_to_name(parent_vhdl, parent_path, sizeof(parent_path)) != NULL) {
                cmn_err(CE_NOTE, "nvme_attach: Parent device path: %s", parent_path);
            }
            /* NOTE: Disabled - parent vertex might not be a PCI device, causes kernel crash
            cmn_err(CE_NOTE, "nvme_attach: Checking parent device for bridge configuration");
            nvme_dump_pci_bridge(parent_vhdl);
            */
        }
    }
#endif
    /* Allocate software state structure */
    NEW(soft);
    if (!soft)
        return -1;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_attach: allocated soft=%p bridge vhdl=%p", soft, nvme_pcie_bridge_conn);
#endif
    soft->pci_vhdl = conn;
    soft->pcie_bridge_vhdl = nvme_pcie_bridge_conn;
    soft->vendor_id = vendor_id;
    soft->device_id = device_id;
    pciioinfo = pciio_info_get(conn);

    /* Check if pciio_info is valid */
    if (!pciioinfo) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_attach: pciio_info_get returned NULL - device not properly initialized!");
#endif
        DEL(soft);
        return -1;
    }

#ifdef NVME_DBG
    /* Dump PCI device information for debugging */
    cmn_err(CE_NOTE, "nvme_attach: PCI Device Information (pciio_info=0x%p):", pciioinfo);
    cmn_err(CE_NOTE, "  Bus: %d, Slot: %d, Function: %d",
            pciio_info_bus_get(pciioinfo),
            pciio_info_slot_get(pciioinfo),
            pciio_info_function_get(pciioinfo));
    cmn_err(CE_NOTE, "  Vendor ID: 0x%04x, Device ID: 0x%04x",
            pciio_info_vendor_id_get(pciioinfo),
            pciio_info_device_id_get(pciioinfo));
#endif
#ifdef NVME_DBG
    /* Dump all 6 BARs */
    {
        int bar;
        for (bar = 0; bar < 6; bar++) {
            pciio_space_t space = pciio_info_bar_space_get(pciioinfo, bar);
            alenaddr_t base = pciio_info_bar_base_get(pciioinfo, bar);
            size_t size = pciio_info_bar_size_get(pciioinfo, bar);

            if (size > 0) {
                cmn_err(CE_NOTE, "  BAR%d: space=0x%x, base=0x%x%0x, size=%u (0x%x)",
                        bar, space, (uint_t)(base >> 32), (uint_t)(base & 0xFFFFFFFF), size, size);
            }
        }
    }
#endif
#ifdef NVME_DBG
    /* Dump ROM info if present */
    {
        alenaddr_t rom_base = pciio_info_rom_base_get(pciioinfo);
        size_t rom_size = pciio_info_rom_size_get(pciioinfo);
        if (rom_size > 0) {
            cmn_err(CE_NOTE, "  ROM: base=0x%llx, size=%u (0x%x)",
                    rom_base, rom_size, rom_size);
        }
    }
#endif

    /* Enable bus master and memory space in PCI command register */
    command = (ushort_t)pciio_config_get(conn, PCI_CFG_COMMAND, 2);
    command |= PCI_CMD_BUS_MASTER | PCI_CMD_MEM_SPACE;
    //command |= PCI_CMD_MEMW_INV_ENAB;
    //command |= PCI_CMD_F_BK_BK_ENABLE;
    pciio_config_set(conn, PCI_CFG_COMMAND, 2, command);
    bar0_size = pciio_info_bar_size_get(pciioinfo, 0);
    if (!bar0_size) {
#ifdef NVME_ATTACH_VERBOSE
        cmn_err(CE_WARN, "nvme_attach: irix reported bar0 size is 0, overring");
#endif
        bar0_size = NVME_BAR0_SIZE;
    }

    bar0_map = pciio_piomap_alloc(conn, 0,
                                   PCIIO_SPACE_WIN(0),
                                   0, bar0_size,
                                   bar0_size,
                                   PCIIO_FIXED);

    if (!bar0_map) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_attach: unable to map BAR0");
#endif
        DEL(soft);
        return -1;
    }

    soft->bar0 = (volatile uchar_t *)pciio_piomap_addr(bar0_map, 0, bar0_size);
    if (!soft->bar0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_attach: unable to get BAR0 address");
#endif
        pciio_piomap_free(bar0_map);
        DEL(soft);
        return -1;
    }

    soft->bar0_map = bar0_map;
    soft->bar0_size = bar0_size;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_attach: mapped BAR0 at %p (size=%u)", soft->bar0, bar0_size);
#endif
    /* Validate that BAR0 mapping actually works by reading NVMe Version register
     * This is a read-only register that should return a non-zero, non-0xFF value
     * If we get all 0s or all FFs, the mapping is bad */
    {
        uint_t vs_reg = NVME_RD(soft, NVME_REG_VS);
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_attach: NVMe Version register read test: 0x%08x", vs_reg);
#endif
        if (vs_reg == 0x00000000 || vs_reg == 0xFFFFFFFF) {
            cmn_err(CE_WARN, "nvme_attach: BAR0 mapping appears invalid (VS reg = 0x%08x)", vs_reg);
            cmn_err(CE_WARN, "nvme_attach: This device may not be properly enumerated - skipping");
            pciio_piomap_free(bar0_map);
            DEL(soft);
            return -1;
        }
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_attach: BAR0 mapping validated - proceeding with initialization");
#endif
    }

    /*
     * Sanitize and initialize controller
     */
    if (nvme_sanitize(soft) != 0) {
        cmn_err(CE_WARN, "nvme_attach: failed to sanitize controller");
        pciio_piomap_free(bar0_map);
        DEL(soft);
        return -1;
    }

    if (nvme_initialize(soft) != 0) {
        cmn_err(CE_WARN, "nvme_attach: failed to initialize controller");
        pciio_piomap_free(bar0_map);
        DEL(soft);
        return -1;
    }

#if defined(IP32)
    nvme_ip32_build_serial_key(soft->serial, serial_key, sizeof(serial_key));
    current_slot = pciio_info_slot_get(pciioinfo);

    /*
     * IP32 two-phase attach:
     *  1) Discovery phase (normal enumeration): stage latest conn per serial and
     *     release controller state immediately (no SCSI graph churn here).
     *  2) Finalize phase (worker thread): re-enter nvme_attach() with
     *     nvme_ip32_finalize_active=1 and perform normal attach once per serial.
     */
    if (!(nvme_ip32_finalize_active && nvme_ip32_finalize_conn == conn)) {
        if (!nvme_ip32_finalize_thread_running) {
            nvme_ip32_start_finalize_thread();
        }

        if (serial_key[0] == '\0') {
            cmn_err(CE_WARN, "nvme_attach: unable to derive IP32 serial key for conn 0x%x", conn);
            nvme_ip32_cleanup_staged_soft(soft);
            return -1;
        }

        /*
         * Save the pre-initialized soft in the candidate table instead of
         * destroying it.  The finalize pass will consume it to skip the costly
         * nvme_sanitize() + nvme_initialize() on the good conn, allowing SCSI
         * registration to complete before IRIX's rootfs lookup fires.
         * If staging fails, fall through to cleanup below.
         */
        if (nvme_ip32_candidate_set_latest(serial_key, conn, current_slot, soft) != 0) {
            nvme_ip32_cleanup_staged_soft(soft);
            return -1;
        }

        cmn_err(CE_NOTE,
                "nvme_attach: IP32 staged candidate SN=%s at conn 0x%x (slot %d)",
                serial_key, (uint_t)conn, current_slot);

        /*
         * Synchronously finalize on the calling thread so the controller is
         * visible to the SCSI layer before this nvme_attach() returns.
         *
         * On the first appearance of a serial: registers the controller.
         * On subsequent appearances (aliases): swaps the soft state to use
         * the latest conn's hardware without re-registering in the SCSI layer.
         *
         * By doing this synchronously we guarantee the drive is in the hwgraph
         * before IRIX's rootfs lookup fires, even on a single-CPU IP32 where
         * the background finalize thread cannot preempt the main boot thread.
         */
        nvme_ip32_finalize_candidates();

        /* Also wake the background thread as a fallback (it will find no
         * candidates since the sync pass already consumed them). */
        if (nvme_ip32_finalize_thread_running) {
            vsema(&nvme_ip32_finalize_sema);
        }

        /* soft is now owned by the candidate table (or was consumed above) */
        return -1;
    }
#endif
/*
    unsigned char sl = ((unsigned char *)pciioinfo)[13];
    ((unsigned char *)pciioinfo)[13] = 3;
    ((unsigned char *)pciioinfo)[13] = sl;
        somehow our slot for the working card is 9
        override slot to match pci-pcie bridge (3)
        according to pcimh.c:mace_ivecs() it looks at pcimh_info_s which is a superset of pciio_info_s
        after dumping the struct in 6.5.22 the slot is at byte 13
        we want to set 3 to match expansion slot
*/
nvme_attach_post_init:
    /*
     * Start timeout watchdog only for controllers that will actually attach.
     * On IP32 staged-candidate probes, nvme_attach returns above and the soft
     * state is owned by the candidate table; arming the watchdog there can race.
     */
    nvme_timeout_watchdog_start(soft);

    /* Initialize quiesce state - not quiesced by default */
    soft->quiesce_state = NO_QUIESCE_IN_PROGRESS;

    /*
     * Initialize interrupt flag and start polling thread
     * Set interrupts_enabled to 0 to use polling mode as backup
     * Set to 1 if interrupts are working reliably
     */
    soft->interrupts_enabled = 0;  /* Use polling mode by default */
#ifdef NVME_COMPLETION_THREAD
    /* Start completion processing thread */
    nvme_start_poll_thread(soft);
#endif
#ifdef NVME_COMPLETION_INTERRUPT
    if (nvme_enable_interrupts(soft) < 0) {
        nvme_shutdown(soft);
        pciio_piomap_free(soft->bar0_map);
        DEL(soft);
        return -1;
    }
#endif

#ifdef NVME_TEST
    nvme_test_admin(soft);
    nvme_test_io(soft);
#endif

    /*
     * Register as a SCSI controller.
     * NVMe namespaces will appear as SCSI LUNs.
     * Derive adapter number from PCI slot (like adp78.c does: slot - 1)
     */
    {
        scsi_ctlr_info_t *scsi_info;
        vertex_hdl_t ctlr_vhdl;
        char loc_str[MAXDEVNAME];
        int slot;

        /* Get PCI slot number for logging */
        slot = pciio_info_slot_get(pciioinfo);

        /* Find next available adapter number by scanning inventory 
           FIXME - let ioconfig do it for us and use ioctl when we are a module?
        */
        soft->adap = nvme_get_next_adapter_num();
#if (defined(IP30) || defined(IP32)) && defined(NVME_BUILTIN)
        // workaround for early boot, give first 2 slots to ql/Adaptec
        if (soft->adap == 0) {
            soft->adap = 2;
        }
#endif        

        cmn_err(CE_NOTE, "nvme_attach: PCI slot=%d, assigned adapter=%d", slot, soft->adap);

        /* Register in global table for character device ioctl */
        if (soft->adap < NVME_MAX_CTLR)
            nvme_ctlr_softmap[soft->adap] = soft;

        /* Create character control device /hw/nvme/ctrl<N> via hwgraph.
         * The minor device number equals the adapter number, so nvme_ioctl()
         * can look up the right soft state via getminor(dev). */
        {
            vertex_hdl_t nvme_root = GRAPH_VERTEX_NONE;
            char ctrl_path[32];

            hwgraph_path_add(hwgraph_root, "nvme", &nvme_root);
            if (nvme_root != GRAPH_VERTEX_NONE) {
                sprintf(ctrl_path, "ctrl%d", soft->adap);
                if (hwgraph_char_device_add(nvme_root, ctrl_path,
                                            "nvme_", &soft->ctrl_vhdl) == GRAPH_SUCCESS) {
                    device_info_set(soft->ctrl_vhdl, soft);
                    cmn_err(CE_NOTE, "nvme: created control device /hw/nvme/%s "
                            "(minor %d)", ctrl_path, soft->adap);
                } else {
                    soft->ctrl_vhdl = GRAPH_VERTEX_NONE;
                    cmn_err(CE_WARN, "nvme: failed to create /hw/nvme/%s "
                            "— ioctl unavailable", ctrl_path);
                }
            }
        }

        /* Create SCSI controller vertex under the PCI connection */
        sprintf(loc_str, "%s/%d", EDGE_LBL_SCSI_CTLR, SCSI_EXT_CTLR(soft->adap));
        rv = hwgraph_path_add(conn, loc_str, &ctlr_vhdl);
        if (rv != GRAPH_SUCCESS) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_attach: unable to create SCSI controller vertex");
#endif
            nvme_shutdown(soft);
            pciio_piomap_free(bar0_map);
            DEL(soft);
            return -1;
        }

        soft->scsi_vhdl = ctlr_vhdl;

        /* Initialize SCSI controller info */
        scsi_info = scsi_ctlr_info_init();
        if (!scsi_info) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_attach: unable to allocate SCSI controller info");
#endif
            hwgraph_vertex_destroy(ctlr_vhdl);
            nvme_shutdown(soft);
            pciio_piomap_free(bar0_map);
            DEL(soft);
            return -1;
        }

        /* Set up SCSI function pointers */
        SCI_ADAP(scsi_info)     = soft->adap;
        SCI_CTLR_VHDL(scsi_info) = ctlr_vhdl;
        SCI_ALLOC(scsi_info)    = nvme_scsi_alloc;
        SCI_COMMAND(scsi_info)  = nvme_scsi_command;
        SCI_FREE(scsi_info)     = nvme_scsi_free;
        SCI_DUMP(scsi_info)     = nvme_scsi_dump;
        SCI_INQ(scsi_info)      = nvme_scsi_info;
        SCI_IOCTL(scsi_info)    = nvme_scsi_ioctl;
        SCI_ABORT(scsi_info)    = nvme_scsi_abort;

        /* Store our soft state in the controller info */
        SCI_INFO(scsi_info) = soft;
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_attach: stored soft=%p in SCI_INFO(scsi_info=%p)", soft, scsi_info);
#endif
        /* Attach controller info to vertex */
        scsi_ctlr_info_put(ctlr_vhdl, scsi_info);

        /* Create SCSI bus */
        scsi_bus_create(ctlr_vhdl);

        /*
         * Create "scsi" edge for boot compatibility (like ql.c does for IP30)
         * This allows /hw/.../pci/.../scsi to work as a shortcut
         */
        rv = hwgraph_edge_add(conn, ctlr_vhdl, EDGE_LBL_SCSI);
#ifdef NVME_DBG
        if (rv == GRAPH_SUCCESS) {
            cmn_err(CE_NOTE, "nvme_attach: created scsi edge at PCI device");
        }
#endif
        /*
         * Create link from /hw/scsi_ctlr/<N> to the actual controller vertex
         * This makes the controller discoverable
         * at the standard system location.
         */
        if (ctlr_vhdl != GRAPH_VERTEX_NONE) {
            int ret;

            /* Get the full path to the controller vertex */
            ret = hwgraph_vertex_name_get(ctlr_vhdl, loc_str, MAXDEVNAME);
            if (ret != 0) {
                cmn_err(CE_WARN, "nvme_attach: hwgraph_vertex_name_get failed with ret=%d, ctlr_vhdl=0x%x",
                        ret, ctlr_vhdl);
            } else {
                char src_name[10], edge_name[5];
                char *path_relative;
                /* Strip /hw/ prefix - hwgraph_link_add expects paths relative to hwgraph_root */
                path_relative = loc_str;
                if (strncmp(path_relative, "/hw/", 4) == 0) {
                    path_relative += 4;
                }

                sprintf(src_name, "%s", EDGE_LBL_SCSI_CTLR);
                sprintf(edge_name, "%d", SCSI_EXT_CTLR(soft->adap));

                hwgraph_link_add(path_relative, src_name, edge_name);

#if (defined(IP30) || defined(IP32)) && defined(NVME_BUILTIN)
                /*
                 * Create link from /hw/ql/<N> to ctlr_vhdl (similar to ql driver)
                 * This makes the device findable for root device discovery
                 * The ql driver creates this to allow boot from SCSI controllers
                 */
                sprintf(src_name, "ql");
                sprintf(edge_name, "%d", soft->adap);
                hwgraph_link_add(path_relative, src_name, edge_name);
#endif                
            }
        }

        /*
         * Add device to system inventory
         * This is critical for ioconfig to recognize the controller
         * Use INV_SCSICONTROL instead of INV_PCI_SCSICONTROL so ioconfig spawns a process
         * State=0 for generic/unknown controller type (shows up as WD33 for internal)
         */
        device_inventory_add(ctlr_vhdl,
                            INV_DISK,             /* class: disk controller */
                            INV_SCSICONTROL,      /* type: SCSI controller */
                            soft->adap,           /* controller number */
                            0,                    /* unit number */
                            0);                   /* state: 0 = generic/unknown */

        cmn_err(CE_NOTE, "nvme_attach: registered as SCSI adapter %d", soft->adap);
        cmn_err(CE_NOTE, "nvme_attach: added device inventory: class=INV_DISK, type=INV_SCSICONTROL, ctlr=%d",
                soft->adap);

        /*
         * Probe target and LUN during attach
         * We present namespace 1 as target 0, LUN 0
         * The SCSI upper layer will call SOP_SCAN, but we'll just return success
         * since we're doing the actual probing here.
         */
        {
            vertex_hdl_t lun_vhdl;
            uint_t targ = 0;
            uint_t lun = 0;

            /* Create target and LUN vertices */
            lun_vhdl = scsi_device_add(ctlr_vhdl, targ, lun);
            if (lun_vhdl == GRAPH_VERTEX_NONE) {
#ifdef NVME_DBG
                cmn_err(CE_WARN, "nvme_attach: unable to create device vertices");
#endif
                hwgraph_vertex_destroy(ctlr_vhdl);
                nvme_shutdown(soft);
                pciio_piomap_free(bar0_map);
                DEL(soft);
                return -1;
            }

            /*
             * Add inventory for the disk device
             */
            device_inventory_add(lun_vhdl,
                                INV_DISK,      /* class: disk */
                                INV_SCSIDRIVE, /* type: SCSI drive */
                                soft->adap,    /* controller number */
                                targ,          /* unit (target ID) */
                                0);            /* state */

            /*
             * Initialize SCSI target info structure
             * Note: We zero the data buffers but NOT tinfo itself since
             * nvme_init_scsi_target_info will set all fields
             */
            bzero(soft->inq_data, sizeof(soft->inq_data));
            bzero(soft->sense_data, sizeof(soft->sense_data));

            /* Build INQUIRY data from NVMe controller info */
            nvme_build_inquiry_data(soft, soft->inq_data);

            /* Initialize SCSI target info structure */
            nvme_init_scsi_target_info(soft);

#ifdef NVME_DBG
            cmn_err(CE_NOTE, "nvme_attach: AFTER INIT soft=%p inq_data=%p sense_data=%p",
                    soft, soft->inq_data, soft->sense_data);
#endif
            /*
             * Notify SCSI layer about the device by providing INQUIRY data
             * This triggers dksc (disk driver) to attach and create disk/volume vertices
             */
            scsi_device_update(soft->inq_data, lun_vhdl);

#ifdef NVME_DBG
            cmn_err(CE_NOTE, "nvme_attach: created SCSI target %d lun %d for namespace %d",
                    targ, lun, soft->nsid);
            cmn_err(CE_NOTE, "nvme_attach: added disk inventory: class=INV_DISK, type=INV_SCSIDRIVE, ctlr=%d, unit=%d",
                    soft->adap, targ);
            cmn_err(CE_NOTE, "nvme_attach: called scsi_device_update to create disk vertices");
#endif
        }
    }

    /*
     * Register PCI error handler
     * This allows the driver to be notified of hardware errors
     * (parity errors, target aborts, etc.)
     */
    pciio_error_register(conn, nvme_error_handler, soft);
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_attach: registered PCI error handler");
#endif

#if defined(IP32)
    if (serial_key[0] != '\0') {
        if (current_slot < 0) {
            current_slot = pciio_info_slot_get(pciioinfo);
        }
        nvme_ip32_alias_set_conn(serial_key, conn, soft, current_slot);
    }
#endif

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_attach: successfully attached NVMe device");
#endif
    return 0;
}

/*
 * nvme_remove_disk_aliases: Remove disk device aliases from /hw/disk or /hw/rdisk
 */
static void
nvme_remove_disk_aliases(char *disk_label, uint_t adap, uint_t targ)
{
    vertex_hdl_t disk_vhdl;
    char dks_name[32];
    int part;

    if (hwgraph_traverse(hwgraph_root, disk_label, &disk_vhdl) == GRAPH_SUCCESS) {
        /* Remove whole disk entry */
        sprintf(dks_name, "dks%dd%d", SCSI_EXT_CTLR(adap), targ);
        hwgraph_edge_remove(disk_vhdl, dks_name, NULL);

        /* Remove partition entries (vh, vol, s0-s15) */
        sprintf(dks_name, "dks%dd%dvh", SCSI_EXT_CTLR(adap), targ);
        hwgraph_edge_remove(disk_vhdl, dks_name, NULL);
        sprintf(dks_name, "dks%dd%dvol", SCSI_EXT_CTLR(adap), targ);
        hwgraph_edge_remove(disk_vhdl, dks_name, NULL);
        for (part = 0; part < 16; part++) {
            sprintf(dks_name, "dks%dd%ds%d", SCSI_EXT_CTLR(adap), targ, part);
            hwgraph_edge_remove(disk_vhdl, dks_name, NULL);
        }
    }
}

/*
 * nvme_detach_attached_soft: detach an already-attached controller by soft state.
 * This path is used both by regular detach and IP32 alias replacement.
 */
static int
nvme_detach_attached_soft(nvme_soft_t *soft, vertex_hdl_t conn, int fast_alias)
{
    vertex_hdl_t ctlr_vhdl;
    uint_t targ = 0;
    uint_t lun = 0;

    if (!soft) {
        return -1;
    }

    cmn_err(CE_NOTE, "nvme: detach initiated — adapter %d [%.40s / SN %.20s]%s",
            soft->adap, (char *)soft->model, (char *)soft->serial,
            fast_alias ? " (fast alias)" : "");

    ctlr_vhdl = soft->scsi_vhdl;

#if defined(IP32)
    {
        char serial_key[21];
        nvme_ip32_build_serial_key(soft->serial, serial_key, sizeof(serial_key));
        if (serial_key[0] != '\0') {
            nvme_ip32_alias_clear_conn(serial_key, conn);
        }
    }
#endif

    /* Remove inventory entries BEFORE removing vertices */
    if (ctlr_vhdl != GRAPH_VERTEX_NONE) {
        vertex_hdl_t lun_vhdl = scsi_lun_vhdl_get(ctlr_vhdl, targ, lun);
        if (lun_vhdl != GRAPH_VERTEX_NONE) {
            /* Remove disk inventory (-1 matches any value) */
            hwgraph_inventory_remove(lun_vhdl, -1, -1, -1, -1, -1);
        }
        /* Remove controller inventory (-1 matches any value) */
        hwgraph_inventory_remove(ctlr_vhdl, -1, -1, -1, -1, -1);
    }

    if (ctlr_vhdl != GRAPH_VERTEX_NONE) {
        /* Remove SCSI device (LUN and target vertices) */
        scsi_device_remove(ctlr_vhdl, targ, lun);

        /* Remove /hw/scsi_ctlr/%d link */
        {
            vertex_hdl_t scsi_ctlr_vhdl;
            char edge_name[5];
            sprintf(edge_name, "%d", SCSI_EXT_CTLR(soft->adap));
            if (hwgraph_traverse(hwgraph_root, EDGE_LBL_SCSI_CTLR, &scsi_ctlr_vhdl) == GRAPH_SUCCESS) {
                hwgraph_edge_remove(scsi_ctlr_vhdl, edge_name, NULL);
            }
        }

        /* Remove /hw/.../pci/.../scsi edge for this connection */
        hwgraph_edge_remove(conn, EDGE_LBL_SCSI, NULL);
    }

    /* Remove disk device aliases (dks entries in /hw/disk and /hw/rdisk) */
    nvme_remove_disk_aliases(EDGE_LBL_DISK, soft->adap, targ);
    nvme_remove_disk_aliases(EDGE_LBL_RDISK, soft->adap, targ);

#ifdef NVME_COMPLETION_INTERRUPT
    /* Disable interrupts */
    nvme_disable_interrupts(soft);
#endif
#ifdef NVME_COMPLETION_THREAD
    /* Stop completion processing thread */
    nvme_stop_poll_thread(soft);
#endif

    /* Shutdown controller and free resources */
    nvme_shutdown_ex(soft, fast_alias);

    if (soft->bar0_map)
        pciio_piomap_free(soft->bar0_map);

    /* Destroy SCSI controller vertex */
    if (ctlr_vhdl != GRAPH_VERTEX_NONE) {
        hwgraph_vertex_destroy(ctlr_vhdl);
    }

    /* Remove control device and clear global table entry */
    if (soft->adap < NVME_MAX_CTLR)
        nvme_ctlr_softmap[soft->adap] = NULL;
    if (soft->ctrl_vhdl != GRAPH_VERTEX_NONE) {
        hwgraph_vertex_destroy(soft->ctrl_vhdl);
        soft->ctrl_vhdl = GRAPH_VERTEX_NONE;
    }

    DEL(soft);

    cmn_err(CE_NOTE, "nvme: detach complete");

    return 0;
}

/*
 * nvme_detach: called when device is being removed
 */
int
nvme_detach(vertex_hdl_t conn)
{
    nvme_soft_t        *soft;
    vertex_hdl_t        ctlr_vhdl;
    uint_t              class_code;
    pciio_info_t        pciioinfo;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_detach: detaching device");
#endif

    /*
     * We registered with wildcards (-1, -1), so detach is called for EVERY PCI device.
     * Check the class code to ensure this is actually an NVMe device before proceeding.
     */
    if (!conn) {
        return -1;
    }
    pciioinfo = pciio_info_get(conn);
    if (pciioinfo == NULL) {
        return -1;
    }
    if (pciio_info_vendor_id_get(pciioinfo) < 0) {
        return -1;
    }

    /* Read class code to verify this is an NVMe device */
    class_code = (uint_t)pciio_config_get(conn, PCI_CFG_CLASS_CODE, 4) & 0x00FFFFFF;

    if (class_code != NVME_CLASS_CODE) {
        /* Not an NVMe device, silently ignore */
        return -1;
    }

    /* Get SCSI controller vertex from the "scsi" edge on the PCI connection */
    if (hwgraph_edge_get(conn, EDGE_LBL_SCSI, &ctlr_vhdl) != GRAPH_SUCCESS) {
        return -1;
    }

    {
        scsi_ctlr_info_t *ctlr_info = scsi_ctlr_info_get(ctlr_vhdl);
        if (!ctlr_info) {
            hwgraph_vertex_unref(ctlr_vhdl);
            return -1;
        }
        soft = SCI_INFO(ctlr_info);
        if (!soft) {
            hwgraph_vertex_unref(ctlr_vhdl);
            return -1;
        }
    }

    return nvme_detach_attached_soft(soft, conn, 0);
}


/* =====================================================================
 *    Completion Processing Thread (Backup for When Interrupts Don't Work)
 * =====================================================================
 */

#define NVME_POLL_INTERVAL_MS   1       /* Sleep 1ms between polls */
#define NVME_POLL_MAX_COUNT     10      /* Poll up to 10 times per wakeup */

/*
 * nvme_poll_thread: Kernel thread that processes completion queues
 *
 * This thread sleeps on a semaphore and wakes up when a command is submitted.
 * Once awake, it polls the completion queues NVME_POLL_MAX_COUNT times with
 * 1ms delays between polls, then goes back to sleep.
 */
void
nvme_poll_thread(void *arg)
{
    nvme_soft_t *soft = (nvme_soft_t *)arg;
    int admin_processed, io_processed;
    int poll_count;
    timespec_t sleep_time;

    sleep_time.tv_sec = 0;
    sleep_time.tv_nsec = NVME_POLL_INTERVAL_MS * 1000000;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_poll_thread: started");
#endif
    while (!soft->poll_shutdown) {
        /* Wait for signal to poll completions */
        psema(&soft->poll_sema, PZERO);

        /* Check shutdown flag after waking */
        if (soft->poll_shutdown) {
            break;
        }

        /* Poll completions up to NVME_POLL_MAX_COUNT times */
        for (poll_count = 0; poll_count < NVME_POLL_MAX_COUNT; poll_count++) {
            if (soft->poll_shutdown || !soft->initialized) {
                break;
            }

            /* Process completions */
            admin_processed = nvme_process_completions(soft, &soft->admin_queue);
            io_processed = nvme_process_completions(soft, &soft->io_queue);

#ifdef NVME_DBG
            if (admin_processed || io_processed) {
                cmn_err(CE_DEBUG, "nvme_poll_thread: processed %d admin, %d I/O completions",
                        admin_processed, io_processed);
            }
#endif
            /* Sleep 1ms between polls */
            if (poll_count < NVME_POLL_MAX_COUNT - 1) {
                nano_delay(&sleep_time);
            }
        }
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_poll_thread: exiting");
#endif
    soft->poll_thread_running = 0;
}

/*
 * nvme_kick_poll_thread: Wake up the polling thread
 *
 * Called when a command is submitted and interrupts are disabled.
 */
void
nvme_kick_poll_thread(nvme_soft_t *soft)
{
    if (soft->poll_thread_running) {
        vsema(&soft->poll_sema);
    }
}

/*
 * nvme_start_poll_thread: Start the completion polling thread
 *
 * Called during driver initialization.
 */
void
nvme_start_poll_thread(nvme_soft_t *soft)
{
    /* Initialize semaphore (count = 0) */
    initnsema(&soft->poll_sema, 0, "nvme_poll");

    /* Clear shutdown flag */
    soft->poll_shutdown = 0;
    soft->poll_thread_running = 1;

    /* Create kernel thread */
    sthread_create("nvme_poll",
                    NULL, 2 * KTHREAD_DEF_STACKSZ, /* stack/stack size */
                    0, /* flags */
                    scsi_intr_pri, /* some priority */
                    KT_PS, /* scheduling flags PS - priority scheduled */
                    nvme_poll_thread,
                    (void *)soft, /* arg0 */ 
                    0, 0, 0); /* rest of args */

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_start_poll_thread: polling thread created");
#endif
}

/*
 * nvme_stop_poll_thread: Stop the completion polling thread
 *
 * Called during driver shutdown.
 */
void
nvme_stop_poll_thread(nvme_soft_t *soft)
{
    if (!soft->poll_thread_running) {
        return;
    }

    /* Signal shutdown */
    soft->poll_shutdown = 1;

    /* Wake thread so it can exit */
    vsema(&soft->poll_sema);

    /* Wait for thread to exit (poll with timeout) */
    {
        int timeout = 100;  /* 100 iterations * 10ms = 1 second */
        while (soft->poll_thread_running && timeout-- > 0) {
            delay(drv_usectohz(10000));  /* 10ms */
        }
    }

    /* Free semaphore */
    freesema(&soft->poll_sema);

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_stop_poll_thread: polling thread stopped");
#endif
}

/*
 * nvme_watchdog_timeout: Watchdog timer callback for missed interrupts
 *
 * This function is called by itimeout when the watchdog timer expires.
 * It checks if there are outstanding commands and processes any completions
 * that may have been missed due to lost interrupts.
 *
 * Called at interrupt level - must be fast and non-blocking.
 */
void
nvme_watchdog_timeout(nvme_soft_t *soft)
{
    nvme_queue_t *q = &soft->io_queue;
    int num_completions;

    /* Clear the active flag atomically */
    if (!compare_and_swap_int((int *)&q->watchdog_active, 1, 0)) {
        /* Watchdog was already cancelled or not active */
        return;
    }

    /*
     * Read outstanding counter with memory barrier to ensure we see
     * the latest value across CPUs. We use atomicAddInt(ptr, 0) which
     * atomically reads the value with full ll/sc memory barriers.
     */
    {
        int outstanding = atomicAddInt((int *)&q->outstanding, 0);

#ifdef NVME_DBG_EXTRA
        cmn_err(CE_NOTE, "nvme_watchdog_timeout: checking for missed interrupts (outstanding=%d)",
                outstanding);
#endif

        /* Check if there are outstanding commands */
        if (outstanding > 0) {
            /* Process any pending completions */
            num_completions = nvme_process_completions(soft, q);

#ifdef NVME_DBG
            if (num_completions > 0) {
                cmn_err(CE_WARN, "nvme_watchdog_timeout: recovered %d missed completions (outstanding now=%d)",
                        num_completions, q->outstanding);
            }
#endif

            /* If there are still outstanding commands, restart the watchdog */
            if (q->outstanding > 0) {
                nvme_watchdog_start(soft, q);
            }
        }
    }
}

/*
 * nvme_watchdog_start: Start the watchdog timer for a queue
 *
 * Called after submitting a command to ensure we catch missed interrupts.
 * Uses atomic compare-and-swap to ensure only one watchdog is active at a time.
 */
void
nvme_watchdog_start(nvme_soft_t *soft, nvme_queue_t *q)
{
    /* Only start if not already active (atomic test-and-set) */
    if (!compare_and_swap_int((int *)&q->watchdog_active, 0, 1)) {
        /* Watchdog is already running */
        return;
    }

    /* Schedule the watchdog timer using fast clock for sub-tick resolution */
    q->watchdog_id = fast_itimeout(nvme_watchdog_timeout,
                                   (void *)soft,
                                   drv_usectohz(NVME_WATCHDOG_TIMEOUT_US),
                                   pldisk);

#ifdef NVME_DBG_EXTRA
    cmn_err(CE_NOTE, "nvme_watchdog_start: watchdog started for queue %d (timeout=%dus)",
            q->qid, NVME_WATCHDOG_TIMEOUT_US);
#endif
}

/*
 * nvme_watchdog_stop: Stop the watchdog timer for a queue
 *
 * Called when all commands have completed or during shutdown.
 */
void
nvme_watchdog_stop(nvme_queue_t *q)
{
    toid_t watchdog_id;

    /* Atomically clear the active flag and get the ID */
    if (!compare_and_swap_int((int *)&q->watchdog_active, 1, 0)) {
        /* Watchdog is not active */
        return;
    }

    /* Cancel the timeout if it hasn't fired yet */
    watchdog_id = q->watchdog_id;
    if (watchdog_id) {
        untimeout(watchdog_id);
        q->watchdog_id = 0;
    }

#ifdef NVME_DBG_EXTRA
    cmn_err(CE_NOTE, "nvme_watchdog_stop: watchdog stopped for queue %d", q->qid);
#endif
}

/*
 * nvme_check_timeouts: Check all in-flight commands for timeouts
 *
 * Iterates through all in-flight CIDs and, for any that has exceeded its
 * per-request timeout (derived from req->sr_timeout with a floor of
 * NVME_IO_TIMEOUT_MIN_SEC), issues an NVMe Abort.  Aborts are rate limited
 * to the controller's ACL so we never flood the admin queue.
 *
 * Also checks CSTS.CFS: a controller fatal status is logged loudly because
 * nothing is going to complete after that.
 *
 * Called from timeout watchdog handler (nvme_timeout_watchdog_handler).
 */
void
nvme_check_timeouts(nvme_soft_t *soft)
{
    time_t now = lbolt;
    nvme_queue_t *q = &soft->io_queue;
    int cid, i;
    scsi_request_t *req;
    time_t elapsed;
    nvme_aborted_cmd_t *entry;
    nvme_cmd_info_t *ci;
    uint_t csts;
    static time_t last_cfs_warn = 0;

    /* Age out stale aborted command entries */
    mutex_lock(&soft->aborted_lock, PZERO);
    if (soft->aborted_bitmap != 0) {
        for (i = 0; i < NVME_ABORT_FIFO_SIZE; i++) {
            if (!(soft->aborted_bitmap & (1U << i)))
                continue;
            entry = &soft->aborted_cmds[i];
            if ((now - entry->abort_time) > NVME_ABORT_TIMEOUT_TICKS)
                soft->aborted_bitmap &= ~(1U << i);
        }
    }
    mutex_unlock(&soft->aborted_lock);

    /* Nothing in flight: nothing can time out.  But a parked request may
     * be waiting for a dispatcher pass that got lost; give it one. */
    if (atomicAddInt((int *)&q->outstanding, 0) == 0) {
        if (soft->defer_count)
            nvme_dispatch_deferred(soft);
        return;
    }

    /* Controller fatal status makes everything below moot */
    csts = NVME_RD(soft, NVME_REG_CSTS);
    if (csts & NVME_CSTS_CFS) {
        if (now - last_cfs_warn > 5 * HZ) {
            last_cfs_warn = now;
            cmn_err(CE_WARN, "nvme: ctlr=%d CONTROLLER FATAL STATUS (CSTS=0x%08x) with %d "
                    "commands outstanding - drive needs a reset (power cycle)",
                    soft->adap, csts, atomicAddInt((int *)&q->outstanding, 0));
        }
    }

    /* Lock the I/O requests structure while we check */
    mutex_lock(&soft->io_requests_lock, PZERO);

    if (soft->io_cid_free_count == soft->io_cid_max) {
        mutex_unlock(&soft->io_requests_lock);
        return;
    }

    for (cid = 0; cid < NVME_IO_QUEUE_SIZE; ) {
        uint_t word_idx = cid >> 5u;
        uint_t word = soft->io_cid_bitmap[word_idx];

        if (word == 0) {
            cid += 32;
            continue;
        }
        if (!(word & (1u << (cid & 0x1F)))) {
            cid++;
            continue;
        }

        ci = &soft->io_requests[cid];
        req = ci->req;
        if (req == NULL) {
            /* Cannot happen now that slots are initialised under the lock */
            cid++;
            continue;
        }

        elapsed = now - ci->start_time;

        if (elapsed > ci->timeout) {
            if (!ci->aborted)
                soft->stats.io_timeouts++;

            /* Respect the controller's abort limit; try the rest next pass */
            if (atomicAddInt((int *)&soft->abort_outstanding, 0) >= (int)soft->abort_limit) {
                cid++;
                continue;
            }

            cmn_err(CE_WARN,
                    "nvme: ctlr=%d SN=%.*s CID %d %s timed out after %d s (limit %d s)%s - aborting",
                    soft->adap, 20, soft->serial, cid,
                    (req->sr_flags & SRF_DIR_IN) ? "read" : "write",
                    (int)(elapsed / HZ), (int)(ci->timeout / HZ),
                    ci->aborted ? " [again]" : "");

            /* Store in aborted FIFO for retry detection (informational) */
            nvme_aborted_fifo_add(soft, req);

            /* Reset the clock so we don't re-abort every 100 ms */
            ci->start_time = now;
            ci->aborted = 1;

            if (nvme_admin_abort_command(soft, (ushort_t)cid)) {
                atomicAddInt((int *)&soft->abort_outstanding, 1);
                soft->stats.io_aborts++;
            }
        }

        cid++;
    }

    mutex_unlock(&soft->io_requests_lock);
}

/*
 * nvme_timeout_watchdog_handler: Timeout watchdog timer callback
 *
 * Called periodically (every 100ms by default) to check for timed-out commands.
 * Reschedules itself if there are outstanding commands.
 */
void
nvme_timeout_watchdog_handler(nvme_soft_t *soft)
{
    nvme_queue_t *q = &soft->io_queue;

    /* Clear the active flag atomically */
    if (!compare_and_swap_int((int *)&soft->timeout_watchdog_active, 1, 0)) {
        /* Watchdog was already cancelled or not active */
        return;
    }

    /* Check for timeouts */
    nvme_check_timeouts(soft);

    nvme_timeout_watchdog_start(soft);
}

/*
 * nvme_timeout_watchdog_start: Start the timeout watchdog timer
 *
 * Schedules periodic timeout checking (default 100ms = 10 Hz).
 * Uses atomic compare-and-swap to ensure only one watchdog is active.
 */
void
nvme_timeout_watchdog_start(nvme_soft_t *soft)
{
    /* Only start if not already active (atomic test-and-set) */
    if (!compare_and_swap_int((int *)&soft->timeout_watchdog_active, 0, 1)) {
        /* Timeout watchdog is already running */
        return;
    }

    /* Schedule the timeout watchdog timer (100ms = HZ/10) */
    soft->timeout_watchdog_id = fast_itimeout(nvme_timeout_watchdog_handler,
                                              (void *)soft,
                                              drv_usectohz(NVME_TIMEOUT_CHECK_INTERVAL_MS * 1000),
                                              pldisk);

#ifdef NVME_DBG_EXTRA
    cmn_err(CE_NOTE, "nvme_timeout_watchdog_start: timeout watchdog started (%dms interval)",
            NVME_TIMEOUT_CHECK_INTERVAL_MS);
#endif
}

/*
 * nvme_timeout_watchdog_stop: Stop the timeout watchdog timer
 *
 * Called during driver detach or shutdown.
 */
void
nvme_timeout_watchdog_stop(nvme_soft_t *soft)
{
    /* Atomically clear the active flag */
    if (!compare_and_swap_int((int *)&soft->timeout_watchdog_active, 1, 0)) {
        /* Timeout watchdog is not active */
        return;
    }

    /* Cancel the timeout if it hasn't fired yet */
    if (soft->timeout_watchdog_id) {
        untimeout(soft->timeout_watchdog_id);
        soft->timeout_watchdog_id = 0;
    }

#ifdef NVME_DBG_EXTRA
    cmn_err(CE_NOTE, "nvme_timeout_watchdog_stop: timeout watchdog stopped");
#endif
}

#ifdef NVME_TEST
void
nvme_test_admin(nvme_soft_t *soft) {
    unsigned int i;
    unsigned int errors = 0;

    cmn_err(CE_WARN, "nvme_test_admin: START");
    soft->admin_queue.cpl_handler = nvme_test_admin_completion;
    for (i = 0; i < soft->admin_queue.size * 4; i++) {
        nvme_cmd_admin_test(soft, i);
        if (i != soft->test_cid) {
            errors += (soft->test_cid == (i ^ 0xFFFF)) ? 0x8001 : 0x1;
            cmn_err(CE_WARN, "nvme_test_admin: %u != %u", i, soft->test_cid);
        }
    }
    soft->admin_queue.cpl_handler = nvme_handle_admin_completion;
    cmn_err(CE_WARN, "nvme_test_admin: END errors=%08X", errors);
}

void
nvme_test_io(nvme_soft_t *soft) {
    unsigned int i;
    unsigned int errors = 0;

    cmn_err(CE_WARN, "nvme_test_io: START");
    soft->io_queue.cpl_handler = nvme_test_io_completion;
    for (i = 0; i < soft->io_queue.size * 4; i++) {
        nvme_cmd_io_test(soft, i);
        if (i != soft->test_cid) {
            errors += (soft->test_cid == (i ^ 0xFFFF)) ? 0x8001 : 0x1;
            cmn_err(CE_WARN, "nvme_test_io: %u != %u", i, soft->test_cid);
        }
    }
    soft->io_queue.cpl_handler = nvme_handle_io_completion;
    cmn_err(CE_WARN, "nvme_test_io: END errors=%08X", errors);
}
#endif

/*
 * nvme_reloadme: called for each device during reload
 */
static void
nvme_reloadme(vertex_hdl_t conn)
{
    cmn_err(CE_NOTE, "nvme_reloadme: reloading device");
}

/*
 * nvme_unloadme: called for each device during unload
 */
static void
nvme_unloadme(vertex_hdl_t conn)
{
    cmn_err(CE_NOTE, "nvme_unloadme: unloading device");
}

/* =====================================================================
 *    Character Device Entry Points (for ml loading support)
 *
 * These are dummy entry points to allow loading via ml(1M).
 * The actual driver functionality is through the SCSI layer.
 */

/*
 * nvme_soft_from_dev: Retrieve soft state from a hwgraph character device dev_t.
 *
 * hwgraph_char_device_add() encodes the vertex handle in the dev_t minor field.
 * dev_to_vhdl() decodes it back; device_info_get() retrieves the soft pointer
 * we stored via device_info_set() at attach time.
 */
static nvme_soft_t *
nvme_soft_from_dev(dev_t dev)
{
    vertex_hdl_t vhdl = dev_to_vhdl(dev);
    if (vhdl == GRAPH_VERTEX_NONE)
        return NULL;
    return (nvme_soft_t *)device_info_get(vhdl);
}

/*
 * nvme_open: Character device open for /hw/nvme/ctrl<N>
 */
int
nvme_open(dev_t *devp, int flag, int otyp, struct cred *crp)
{
    nvme_soft_t *soft = nvme_soft_from_dev(*devp);
    if (!soft || !soft->initialized)
        return ENXIO;
    return 0;
}

/*
 * nvme_close: Character device close for /hw/nvme/ctrl<N>
 */
int
nvme_close(dev_t dev, int flag, int otyp, struct cred *crp)
{
    return 0;
}

/*
 * nvme_ioctl: Runtime control of NVMe controller features.
 *
 * Commands:
 *   NVME_IOC_GET_VWC  -- copy int (0=disabled, 1=enabled) to *arg
 *   NVME_IOC_SET_VWC  -- read int from *arg; 0=disable, non-zero=enable
 *
 * Device node: /hw/nvme/ctrl<N>  (minor number = hwgraph vertex handle)
 */
int
nvme_ioctl(dev_t dev, int cmd, void *arg, int mode, struct cred *crp, int *rvalp)
{
    nvme_soft_t *soft = nvme_soft_from_dev(dev);
    int val;

    if (!soft || !soft->initialized)
        return ENXIO;

    switch (cmd) {
    case NVME_IOC_GET_VWC:
        val = soft->vwc_enabled;
        if (copyout(&val, (caddr_t)arg, sizeof(int)))
            return EFAULT;
        return 0;

    case NVME_IOC_SET_VWC:
        if (copyin((caddr_t)arg, &val, sizeof(int)))
            return EFAULT;
        if (!soft->features[NVME_FEAT_VOLATILE_WRITE_CACHE]) {
            cmn_err(CE_WARN, "nvme_ioctl: drive does not support "
                    "host-controlled write cache (VWC feature absent)");
            return ENOTSUP;
        }
        if (nvme_admin_set_features(soft, NVME_FEAT_VOLATILE_WRITE_CACHE,
                                    val ? 1 : 0)) {
            nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
            soft->vwc_enabled = val ? 1 : 0;
            cmn_err(CE_NOTE, "nvme_ioctl: adapter %d write cache %s",
                    soft->adap, soft->vwc_enabled ? "ENABLED" : "DISABLED");
            return 0;
        }
        cmn_err(CE_WARN, "nvme_ioctl: Set Features(VWC=%d) failed", val ? 1 : 0);
        return EIO;

    case NVME_IOC_GET_FUA:
        val = soft->fua_enabled;
        if (copyout(&val, (caddr_t)arg, sizeof(int)))
            return EFAULT;
        return 0;

    case NVME_IOC_GET_STATS: {
        nvme_stats_t st = soft->stats;
        st.max_transfer_blocks = soft->max_transfer_blocks;
        if (copyout(&st, (caddr_t)arg, sizeof(st)))
            return EFAULT;
        return 0;
    }

    case NVME_IOC_SET_FUA:
        if (copyin((caddr_t)arg, &val, sizeof(int)))
            return EFAULT;
        soft->fua_enabled = val ? 1 : 0;
        cmn_err(CE_NOTE, "nvme_ioctl: adapter %d FUA %s",
                soft->adap, soft->fua_enabled ? "ENABLED" : "DISABLED");
        return 0;

    default:
        return EINVAL;
    }
}
