#ifndef __NVMEDRV_H
#define __NVMEDRV_H

#define noNVME_DBG
#define noNVME_DBG_EXTRA
#define noNVME_DBG_CMD

#define noNVME_COMPLETION_THREAD
#define noNVME_COMPLETION_MANUAL
#define noNVME_COMPLETION_INTERRUPT
#define noNVME_FORCE_4K
#define noNVME_TEST

/* IP32 can mix and match swapping regions because it is all address based
   IP30 and IP35 has endianess set per PCI slot, so everything has to be bytestream 
*/
#ifdef IP30
#define NVME_UTILBUF_USEDMAP
#define NVME_QUEUE_BYTESWAP
#define NVME_COMPLETION_INTERRUPT
#define DMATRANS64 PCIIO_DMA_A64
#endif

#ifdef IP35
#define NVME_COMPLETION_INTERRUPT
#define NVME_UTILBUF_USEDMAP
#define NVME_QUEUE_BYTESWAP
#define DMATRANS64 PCIIO_DMA_A64
#endif

#if defined(IP32)
#define NVME_COMPLETION_INTERRUPT
#define DMATRANS64 0
#endif

#define NVME_UTILBUF_BSWAP

/*
 * nvmedrv.h - IRIX NVMe Driver Internal Definitions
 *
 * Driver-specific structures, constants, and function prototypes
 */

/* Standard system headers */
#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/cmn_err.h>
#include <sys/errno.h>
#include <sys/ddi.h>

/* Memory management */
#include <sys/kmem.h>
#include <sys/immu.h>           /* For CACH_* flags used by data_cache_* macros */

/* I/O and buffer management */
#include <sys/buf.h>
#include <sys/alenlist.h>

/* Synchronization */
#include <sys/sema.h>

/* Hardware graph and device infrastructure */
#include <sys/hwgraph.h>
//#include <ksys/hwg.h>
extern void hwgraph_link_add(   char *dest_path,
                                char *src_path,
                                char *edge_name);
#include <sys/iograph.h>
#include <sys/iobus.h>
#include <sys/invent.h>         /* For device inventory (INV_DISK, etc.) */

/* PCI infrastructure */
#include <sys/PCI/PCI_defs.h>
#include <sys/PCI/pciio.h>
#if defined(IP30) || defined(IP35)
#include <sys/PCI/pcibr.h>
#include <sys/PCI/bridge.h>
#endif

/* SCSI emulation */
#include <sys/scsi.h>

/* Loadable module support */
#include <sys/mload.h>

#include <sys/var.h>
#include <sys/atomic_ops.h>

/* NVMe specification definitions */
#include "nvme.h"

#ifdef IP30
#include <sys/RACER/IP30.h>
#ifndef HEART_COHERENCY_WAR
void heart_dcache_wb_inval(void *,int);
void heart_dcache_inval(void *,int);
int heart_need_flush(int read);
#endif
#ifndef HEART_INVALIDATE_WAR
int heart_need_invalidate(void);
void heart_invalidate_war(void *,int);
struct buf;
void bp_heart_invalidate_war(struct buf *bp);
#endif
#endif
#ifdef IP32
#include <sys/IP32.h>
#endif
#ifdef IP35
#include <sys/SN/SN1/IP35.h>
#endif

#define PAGE_SIZE NBPP
#define PAGE_MASK (PAGE_SIZE - 1)
#if NBPP == 4096
#define PAGE_SHIFT 12
#endif
#if NBPP == 16384
#define PAGE_SHIFT 14
#endif

/*
 * Driver Configuration
 */
#define NVME_ADMIN_QUEUE_SIZE   64      /* Admin queue depth */
#define NVME_IO_QUEUE_SIZE      512     /* I/O queue depth */
#define NVME_WATCHDOG_TIMEOUT_US 2000   /* Watchdog timeout in microseconds (2ms) */
#define NVME_TIMEOUT_CHECK_INTERVAL_MS 100  /* Check for timeouts every 100ms (10 Hz) */

/* SCSI CDB Operation Codes we handle */
#define SCSIOP_TEST_UNIT_READY    0x00
#define SCSIOP_INQUIRY            0x12
#define SCSIOP_SEND_DIAGNOSTIC    0x1D
#define SCSIOP_MODE_SENSE_6       0x1A
#define SCSIOP_READ_CAPACITY_10   0x25
#define SCSIOP_READ_6             0x08
#define SCSIOP_READ_10            0x28
#define SCSIOP_READ_16            0x88
#define SCSIOP_WRITE_6            0x0A
#define SCSIOP_WRITE_10           0x2A
#define SCSIOP_WRITE_16           0x8A
#define SCSIOP_SYNC_CACHE         0x35

/* MODE SENSE page codes */
#define MODE_SENSE_RETURN_ALL           0x3F
#define MODE_PAGE_FORMAT_DEVICE         0x03
#define MODE_PAGE_RIGID_GEOMETRY        0x04
#define MODE_PAGE_CACHING               0x08
#define MODE_PAGE_CONTROL               0x0A
#define MODE_PAGE_POWER_CONDITION       0x1A
#define MODE_PAGE_FAULT_REPORTING       0x1C

/* MODE SENSE page control values */
#define MODE_SENSE_CURRENT_VALUES       0x00
#define MODE_SENSE_CHANGEABLE_VALUES    0x01
#define MODE_SENSE_DEFAULT_VALUES       0x02
#define MODE_SENSE_SAVED_VALUES         0x03

/* NVMe internal command flags (for passing through the call stack) */
#define NF_WRITE    0x01    /* Command is a write operation */
#define NF_RETRY    0x02    /* Command is a retry of an aborted command */

/* Sense codes */
#define SCSI_SENSE_NO_SENSE         0x00
#define SCSI_SENSE_RECOVERED_ERROR  0x01
#define SCSI_SENSE_NOT_READY        0x02
#define SCSI_SENSE_MEDIUM_ERROR     0x03
#define SCSI_SENSE_HARDWARE_ERROR   0x04
#define SCSI_SENSE_ILLEGAL_REQUEST  0x05
#define SCSI_SENSE_UNIT_ATTENTION   0x06
#define SCSI_SENSE_DATA_PROTECT     0x07
#define SCSI_SENSE_BLANK_CHECK      0x08
#define SCSI_SENSE_UNIQUE           0x09
#define SCSI_SENSE_COPY_ABORTED     0x0A
#define SCSI_SENSE_ABORTED_COMMAND  0x0B
#define SCSI_SENSE_EQUAL            0x0C
#define SCSI_SENSE_VOL_OVERFLOW     0x0D
#define SCSI_SENSE_MISCOMPARE       0x0E
#define SCSI_SENSE_RESERVED         0x0F


#define SCSI_ADSENSE_INVALID_LUN    0x25

/*
 * NVMe Queue Structures
 */

struct nvme_soft_s;
struct nvme_queue_s;

typedef void (*nvme_completion_handler_t)(struct nvme_soft_s *soft, struct nvme_queue_s *q, nvme_completion_t *cpl);


/* Queue Pair (Submission + Completion) */
typedef struct nvme_queue_s {
    /* Submission Queue */
    nvme_command_t     *sq;             /* Submission queue base */
    volatile uint_t     sq_head;        /* SQ head pointer (updated by completion, read by submission) */
    uint_t              sq_tail;        /* SQ tail pointer */
    uint_t              size;           /* size in entries */
    uint_t              size_mask;      /* mask (size - 1) for wraparound */
    uint_t              size_shift;     /* Shift amount to extract phase bit */
    uint_t              sq_doorbell;    /* SQ doorbell register offset */

    /* Completion Queue */
    nvme_completion_t  *cq;             /* Completion queue base */
    uint_t              cq_head;        /* CQ head pointer (bit 0 = phase after mask) */
    uint_t              cq_doorbell;    /* CQ doorbell register offset */

    /* DMA mappings */
    pciio_dmamap_t      sq_dmamap;      /* DMA map for SQ */
    pciio_dmamap_t      cq_dmamap;      /* DMA map for CQ */
    alenaddr_t          sq_phys;        /* Physical address of SQ */
    alenaddr_t          cq_phys;        /* Physical address of CQ */

    /* Queue metadata */
    ushort_t            qid;            /* Queue ID */
    ushort_t            vector;         /* Interrupt vector */
    mutex_t             lock;           /* Queue lock */
    nvme_completion_handler_t cpl_handler; /* Completion handler */

    /* Outstanding command tracking */
    volatile int        outstanding;    /* Atomic counter of commands in flight */

    /* Watchdog timer for missed interrupts */
    toid_t              watchdog_id;    /* Timeout ID for watchdog timer */
    volatile int        watchdog_active; /* Flag: 1 if watchdog is running */
} nvme_queue_t;


/*
 * PRP List Pool Configuration
 */
#define NVME_PRP_POOL_SIZE      64      /* Number of PRP list pages (64 * 4KB = 256KB) */

/*
 * Command Tracking Structure
 */
#define NVME_CMD_MAX_PRPS 4
typedef struct nvme_cmd_info {
    scsi_request_t     *req;           /* Associated SCSI request */
    time_t              start_time;    /* lbolt when command was issued */
    int                 prpidx[NVME_CMD_MAX_PRPS]; /* Index into PRP pool (0-63, -1 if none) */
    int                 last;
} nvme_cmd_info_t;

/*
 * Aborted Command Tracking for Retry Detection
 * Stores key fields from scsi_request_t that remain constant across retries
 */
#define NVME_ABORT_FIFO_SIZE 16  /* Track last 16 aborted commands */
#define NVME_ABORT_TIMEOUT_TICKS (1 * HZ)  /* 1 second - age out stale aborted entries */
#define SCSI_MAX_CDB_LEN 16      /* Maximum CDB length */

typedef struct nvme_aborted_cmd {
    u_char              cdb[SCSI_MAX_CDB_LEN]; /* SCSI CDB (command descriptor block) */
    ushort_t            sr_flags;       /* Direction and control flags */
    u_char             *sr_buffer;      /* Buffer address */
    uint_t              sr_buflen;      /* Buffer length */
    void               *sr_bp;          /* buf_t pointer */
    time_t              abort_time;     /* lbolt when command was aborted (for aging) */
} nvme_aborted_cmd_t;

typedef struct nvme_soft_s {
    /* Hardware graph vertices */
    vertex_hdl_t        pci_vhdl;         /* PCI connection vertex */
    vertex_hdl_t        scsi_vhdl;        /* SCSI controller vertex */
    vertex_hdl_t        pcie_bridge_vhdl; /* PCI connection vertex */

    /* Controller registers (BAR0) */
    volatile uchar_t   *bar0;           /* BAR0 base address */
    pciio_piomap_t      bar0_map;       /* PIO map for BAR0 */
    size_t              bar0_size;      /* BAR0 size */

    /* Controller capabilities */
    __uint64_t          cap;            /* CAP register value */
    uint_t              vs;             /* Version register */
    uint_t              max_queue_entries; /* MQES + 1 */
    uint_t              min_page_size;
    uint_t              max_page_size;
    uint_t              nvme_page_size; /* size used for transfers, ideally matching NBPP */
    uint_t              nvme_page_shift;
    u_int8_t            nvme_prp_entries; /* number of prp entries in nvme page */
    uint_t              doorbell_stride; /* Doorbell stride */

    /* Queues */
    nvme_queue_t        admin_queue;    /* Admin queue pair */
    nvme_queue_t        io_queue;      /* Array of I/O queue pairs */

    /* Interrupts */
    pciio_intr_t        intr;           /* Interrupt handle */
    int                 interrupts_enabled; /* Flag: 1 if interrupts work, 0 to use polling */

    /* Completion processing thread (backup for when interrupts don't work) */
    sema_t              poll_sema;            /* Semaphore to wake polling thread */
    volatile int        poll_shutdown;        /* Set to 1 to shutdown polling thread */
    int                 poll_thread_running;  /* 1 if thread is running */

    /* Quiesce state for SOP_QUIESCE_STATE ioctl */
    int                 quiesce_state;        /* QUIESCE_* state flags */

    /* Utility buffer for admin commands during init */
    void               *utility_buffer;      /* Virtual address */
    alenaddr_t          utility_buffer_phys; /* Physical address */
#ifdef NVME_UTILBUF_USEDMAP
    pciio_dmamap_t      utility_buffer_dmamap; /* DMA map for utility buffer */
#endif
    /* PRP list pool for I/O operations (64 nvme_page_size pages, nvme_prp_entries PRP entries per page) */
    void               *prp_pool;            /* Virtual address of PRP list pool (64 pages) */
    alenaddr_t          prp_pool_phys;       /* Physical address of PRP list pool */
    pciio_dmamap_t      prp_pool_dmamap;     /* DMA map for PRP list pool */
    mutex_t             prp_pool_lock;       /* Lock for PRP pool allocation */
    __uint64_t          prp_pool_bitmap;     /* Bitmap of available pages (64 bits) */

    /* I/O Command tracking - indexed by CID */
    nvme_cmd_info_t     io_requests[NVME_IO_QUEUE_SIZE]; /* wrapped SCSI requests + PRP idx by CID */
    mutex_t             io_requests_lock;    /* Lock for CID allocation */
    __uint32_t          io_cid_bitmap[NVME_IO_QUEUE_SIZE/32];    /* Bitmap of free CIDs (256 bits = 8x32) */
    uint_t              io_cid_free_count;   /* Number of free CIDs available */

    /* Pre-allocated alenlist for address/length conversions (avoids dynamic allocation failures) */
    alenlist_t          alenlist;            /* Pre-grown alenlist for buf_to_alenlist/kvaddr/uvaddr conversions */
    mutex_t             alenlist_lock;       /* Lock to protect alenlist during concurrent use */

    /* Controller limits */
    uchar_t             mdts;           /* Maximum Data Transfer Size (2^n pages, 0=unlimited) */
    uint_t              max_transfer_blocks; /* Maximum transfer in blocks (calculated from MDTS) */

    /* Namespace information - we and everyone in the world only use ns 1 */
    __uint64_t          num_blocks;     /* Total blocks */
    uint_t              block_size;     /* Block size in bytes */
    uint_t              lba_shift;      /* log2(block_size) */
    uint_t              nsid;           /* Namespace ID (always 1) */
    uint_t              num_namespaces; /* Number of namespaces */

    /* SCSI emulation */
    int                 adap;           /* SCSI adapter number */
    scsi_target_info_t  tinfo;          /* SCSI target info for LUN 0 */
    u_char              inq_data[SCSI_INQUIRY_LEN];  /* INQUIRY data buffer */
    u_char              sense_data[SCSI_SENSE_LEN];  /* Sense data buffer */

    /* State */
    int                 initialized;

    /* Timeout watchdog timer */
    toid_t              timeout_watchdog_id;    /* Timeout ID for timeout checking */
    volatile int        timeout_watchdog_active; /* Flag: 1 if timeout watchdog is running */

    /* Aborted command tracking for retry detection */
    nvme_aborted_cmd_t  aborted_cmds[NVME_ABORT_FIFO_SIZE]; /* FIFO of aborted commands */
    uint_t              aborted_head;   /* Head index (next write position) */
    uint_t              aborted_bitmap; /* Bitmap of valid entries (bit N = entry N valid) */
    mutex_t             aborted_lock;   /* Lock for FIFO access */

    /* Identification */
    ushort_t            vendor_id;      /* PCI vendor ID */
    ushort_t            device_id;      /* PCI device ID */
    uchar_t             serial[21];     /* Serial number (20 bytes + null terminator) */
    uchar_t             model[41];      /* Model number (40 bytes + null terminator) */
    uchar_t             firmware_rev[9]; /* Firmware revision (8 bytes + null terminator) */

    /* Controller Features (from Get Features commands with SEL=SUPPORTED)
     * Array indexed by FID - stores capability bitmask showing which bits are changeable.
     * Value of 0 means feature not supported or not changeable. */
    uint_t              features[16];               /* Features 0x00-0x0F */

    /* Optional NVMe Command Support (ONCS) from Identify Controller */
    uchar_t             oncs_compare;               /* Bit 0: Compare command supported */
    uchar_t             oncs_dataset_mgmt;          /* Bit 2: Dataset Management (TRIM) supported */
    uchar_t             oncs_verify;                /* Bit 5: Verify command supported */

#ifdef NVME_TEST
    volatile unsigned int test_cid;
#endif
} nvme_soft_t;

#define NVME_ADMIN_CID_DONT_CARE             0
#define NVME_ADMIN_CID_IDENTIFY_CONTROLLER   1
#define NVME_ADMIN_CID_IDENTIFY_NAMESPACE    2
#define NVME_ADMIN_CID_CREATE_CQ             3
#define NVME_ADMIN_CID_CREATE_SQ             4
#define NVME_ADMIN_CID_DELETE_SQ             5
#define NVME_ADMIN_CID_DELETE_CQ             6
#define NVME_ADMIN_CID_GET_LOG_PAGE_ERROR    7
#define NVME_ADMIN_CID_SET_FEATURES          8

/* Get Features CIDs: Reserve CIDs 16-31 for Get Features (16 slots)
 * CID = 16 + FID, so we can extract FID from CID in completion handler */
#define NVME_ADMIN_CID_GET_FEATURES_BASE     16
#define NVME_ADMIN_CID_GET_FEATURES_END      31
#define NVME_ADMIN_CID_GET_FEATURES(fid)     (NVME_ADMIN_CID_GET_FEATURES_BASE + (fid))
#define NVME_ADMIN_CID_IS_GET_FEATURES(cid)  ((cid) >= NVME_ADMIN_CID_GET_FEATURES_BASE && (cid) <= NVME_ADMIN_CID_GET_FEATURES_END)
#define NVME_ADMIN_CID_EXTRACT_FID(cid)      ((cid) - NVME_ADMIN_CID_GET_FEATURES_BASE)

/* Special CIDs for ordered I/O commands not associated with scsi_request */
#define NVME_IO_CID_FLUSH                    0x8000

/* Special CID encoding for abort commands in admin queue */
#define NVME_ADMIN_CID_ABORT_MASK            0x8000  /* Bit 15 set = abort command */
#define NVME_ADMIN_CID_ABORT_CID_MASK        0x7FFF
#define NVME_ADMIN_CID_MAKE_ABORT(cid)       (NVME_ADMIN_CID_ABORT_MASK | ((cid) & NVME_ADMIN_CID_ABORT_CID_MASK))
#define NVME_ADMIN_CID_IS_ABORT(cid)         (((cid) & NVME_ADMIN_CID_ABORT_MASK) != 0)
#define NVME_ADMIN_CID_GET_ABORTED_CID(cid)  ((cid) & NVME_ADMIN_CID_ABORT_CID_MASK)

/*
 * Function Prototypes - nvme_cmd.c
 */

/* Alenlist allocation strategy thresholds */
#define NVME_ALENLIST_SMALL_PAGES  16  /* Use dynamic alenlist for requests <= 16 pages (64KB) */

/* Alenlist type tracking for cleanup */
#define NVME_ALENLIST_SUPPLIED     0   /* Alenlist supplied by SCSI layer (sr_ha_alenlist) */
#define NVME_ALENLIST_DYNAMIC      1   /* Dynamically allocated, needs alenlist_destroy() */
#define NVME_ALENLIST_SHARED       2   /* Using shared alenlist, needs mutex_unlock() */

typedef struct nvme_rwcmd_state_s {
    scsi_request_t *req;
    alenlist_t alenlist;
    int alenlist_type;  /* NVME_ALENLIST_* - tracks cleanup method */
    __uint64_t lba;
    uint_t buflen;
    uint_t num_blocks;
    uint_t flags;
    uint_t max_transfer_blocks;
    uint_t commands;
    uint_t cidx;
    unsigned int cids[NVME_IO_QUEUE_SIZE];
    nvme_command_t cmd;
} nvme_rwcmd_state_t;


int nvme_admin_identify_controller(nvme_soft_t *soft);
int nvme_admin_identify_namespace(nvme_soft_t *soft);
int nvme_admin_get_log_page_error(nvme_soft_t *soft);
int nvme_admin_create_cq(nvme_soft_t *soft, ushort_t qid, ushort_t qsize,
                         alenaddr_t phys_addr, ushort_t vector);
int nvme_admin_create_sq(nvme_soft_t *soft, ushort_t qid, ushort_t qsize,
                         alenaddr_t phys_addr, ushort_t cqid);
int nvme_admin_delete_sq(nvme_soft_t *soft, ushort_t qid);
int nvme_admin_delete_cq(nvme_soft_t *soft, ushort_t qid);
int nvme_admin_abort_command(nvme_soft_t *soft, ushort_t cid);
int nvme_admin_get_features(nvme_soft_t *soft, uchar_t fid, uchar_t sel);
int nvme_admin_set_features(nvme_soft_t *soft, uchar_t fid, uint_t value);
int nvme_admin_query_features(nvme_soft_t *soft);

int nvme_submit_cmd(nvme_soft_t *soft, nvme_queue_t *q, nvme_command_t *cmd);
int nvme_wait_for_completion(nvme_queue_t *q, ushort_t cid, uint_t timeout_ms);


int nvme_get_translated_addr(nvme_soft_t *soft,
                             alenlist_t alenlist,
                             size_t maxlength,
                             alenaddr_t *out_address,
                             size_t *out_length,
                             int flags);

int nvme_prepare_alenlist(nvme_soft_t *soft, nvme_rwcmd_state_t *ps);

int nvme_io_build_rw_command(nvme_soft_t *soft, nvme_rwcmd_state_t *ps);
int nvme_build_prps_from_alenlist(nvme_soft_t *soft, nvme_rwcmd_state_t *ps);
void nvme_cleanup_alenlist(nvme_soft_t *soft, nvme_rwcmd_state_t *ps);

int nvme_prp_pool_init(nvme_soft_t *soft);
void nvme_prp_pool_done(nvme_soft_t *soft);
int nvme_prp_pool_alloc(nvme_soft_t *soft);
void nvme_prp_pool_free(nvme_soft_t *soft, int index);

int nvme_io_cid_alloc(nvme_soft_t *soft, scsi_request_t *req, unsigned int commands, unsigned int *cid_array);
scsi_request_t *nvme_io_cid_done(nvme_soft_t *soft, unsigned int cid, int *last);
int nvme_io_cid_store_prp(nvme_soft_t *soft, unsigned int cid, int prpidx);

int nvme_cmd_special_flush(nvme_soft_t *soft);

/*
 * Function Prototypes - nvme_cpl.c
 */
void nvme_read_completion(nvme_completion_t *cpl, nvme_queue_t *q);
int nvme_process_completions(nvme_soft_t *soft, nvme_queue_t *q);
void nvme_handle_admin_completion(nvme_soft_t *soft, nvme_queue_t *q, nvme_completion_t *cpl);
void nvme_handle_io_completion(nvme_soft_t *soft, nvme_queue_t *q, nvme_completion_t *cpl);

/* SCSI status helpers */
void nvme_set_adapter_status(scsi_request_t *req, uint_t sr_status, u_char sr_scsi_status);
void nvme_set_adapter_error(scsi_request_t *req);
void nvme_set_success(scsi_request_t *req);
void nvme_scsi_set_error(scsi_request_t *req, u_char sense_key, u_char asc, u_char ascq);

/* Request completion with R10K+ cache invalidation workaround */
void nvme_complete_request(scsi_request_t *req);
/*
 * Function Prototypes - nvme_scsi.c
 */
void nvme_scsi_command(scsi_request_t *req);
int nvme_scsi_alloc(vertex_hdl_t lun_vhdl, int opt, void (*cb)());
void nvme_scsi_free(vertex_hdl_t lun_vhdl, void (*cb)());
struct scsi_target_info *nvme_scsi_info(vertex_hdl_t lun_vhdl);
int nvme_scsi_dump(vertex_hdl_t ctlr_vhdl);
int nvme_scsi_abort(scsi_request_t *req);
int nvme_scsi_ioctl(vertex_hdl_t ctlr_vhdl, unsigned int cmd, struct scsi_ha_op *op);
void nvme_build_inquiry_data(nvme_soft_t *soft, u_char *inq_data);
void nvme_init_scsi_target_info(nvme_soft_t *soft);

/* Aborted command FIFO management */
void nvme_aborted_fifo_add(nvme_soft_t *soft, scsi_request_t *req);
int nvme_aborted_fifo_find_and_remove(nvme_soft_t *soft, nvme_rwcmd_state_t *ps);

void nvme_scsi_read_write(nvme_soft_t *soft, scsi_request_t *req);
int nvme_scsi_inquiry(nvme_soft_t *soft, scsi_request_t *req);
int nvme_scsi_read_capacity(nvme_soft_t *soft, scsi_request_t *req);
int nvme_scsi_test_unit_ready(nvme_soft_t *soft, scsi_request_t *req);
int nvme_scsi_send_diagnostic(nvme_soft_t *soft, scsi_request_t *req);
int nvme_scsi_sync_cache(nvme_soft_t *soft, scsi_request_t *req);

/*
 * Function Prototypes - nvmedrv.c (controller management)
 */
int nvme_ctlr_enable(nvme_soft_t *soft);
int nvme_ctlr_disable(nvme_soft_t *soft);
int nvme_ctlr_reset(nvme_soft_t *soft);
int nvme_ctlr_init(nvme_soft_t *soft);
int nvme_ctlr_shutdown(nvme_soft_t *soft);
void nvme_dump_controller_state(nvme_soft_t *soft, const char *context);

/* Helper to wait for queue to drain completions */
int nvme_wait_for_queue_idle(nvme_soft_t *soft, nvme_queue_t *q, uint_t timeout_ms);
void nvme_dump_sq_entry(nvme_command_t *cmd, const char *context);
void nvme_dump_memory(void *addr, size_t len, const char *context);
void nvme_dump_pci_bridge(vertex_hdl_t conn);

/* Completion processing thread for interrupt fallback */
void nvme_start_poll_thread(nvme_soft_t *soft);
void nvme_stop_poll_thread(nvme_soft_t *soft);
void nvme_kick_poll_thread(nvme_soft_t *soft);
void nvme_poll_thread(void *arg);

/* Watchdog timer for missed interrupts */
void nvme_watchdog_start(nvme_soft_t *soft, nvme_queue_t *q);
void nvme_watchdog_stop(nvme_queue_t *q);
void nvme_watchdog_timeout(nvme_soft_t *soft);

/* Timeout checking for stuck commands */
void nvme_check_timeouts(nvme_soft_t *soft);
void nvme_timeout_watchdog_start(nvme_soft_t *soft);
void nvme_timeout_watchdog_stop(nvme_soft_t *soft);
void nvme_timeout_watchdog_handler(nvme_soft_t *soft);

/*
 * Utility Macros - NVMe BAR MMIO Access
 *
 * 32-bit accesses only to avoid endianness issues on big-endian MIPS.
 * MMIO accesses are automatically byte-swapped by SGI's PCI bridge hardware.
 */

#define NVME_RD(soft, offset) \
    (*(uint_t volatile *)((soft)->bar0 + (offset)))

#define NVME_WR(soft, offset, value) \
    (*(uint_t volatile *)((soft)->bar0 + (offset)) = (value))

/*
 * Utility Macros - NVMe Memory Access (DMA structures)
 *
 * DMA structures (submission/completion queues, PRPs, etc.) are accessed
 * by the NVMe controller via DMA.
 *
 * When using pciio_dmatrans_addr() with PCIIO_DMA_CMD flag, the PCI bridge
 * handles byte-swapping automatically for command/data structures, so we
 * don't need to manually swap bytes in software.
 *
 * Define NVME_MANUAL_BYTESWAP if you need manual byte-swapping (e.g., when
 * not using PCIIO_DMA_CMD, or for debugging).
 */
/* Manual byte-swapping: Convert between CPU (big-endian) and NVMe (little-endian) */
#define NVME_SWAP32(x) \
    (((x) << 24) | (((x) & 0xff00) << 8) | (((x) >> 8) & 0xff00) | ((x) >> 24))

 #ifdef NVME_QUEUE_BYTESWAP
#define NVME_MEMRD(ptr) \
    NVME_SWAP32(*(volatile uint_t *)(ptr))

#define NVME_MEMWR(ptr, value) \
    (*(volatile uint_t *)(ptr) = NVME_SWAP32(value))
#define QUEUE_SWAP PCIIO_BYTE_STREAM
#else
/* No byte-swapping: PCI bridge handles it via PCIIO_DMA_CMD */
#define NVME_MEMRD(ptr) \
    (*(volatile uint_t *)(ptr))

#define NVME_MEMWR(ptr, value) \
    (*(volatile uint_t *)(ptr) = (value))
#define QUEUE_SWAP PCIIO_WORD_VALUES
#endif /* NVME_QUEUE_BYTESWAP */

#define NVME_MEMRDBS(ptr) \
    NVME_SWAP32(*(volatile uint_t *)(ptr))

#define NVME_MEMWRBS(ptr, value) \
    (*(volatile uint_t *)(ptr) = NVME_SWAP32(value))

/*
 * Soft state storage:
 * - SCSI controller vertex ONLY: Uses SCI_INFO(scsi_ctlr_info) for all access
 * - DO NOT use nvme_soft_set/get on PCI vertex (conn) - that's where pciio_info is stored!
 * - Store in SCI_INFO, retrieve via SLI_CTLR_INFO(lun_info)
 * - These macros are kept for potential future use on driver-created vertices only
 */
#define nvme_soft_set(v,i)  device_info_set((v),(void *)(i))
#define nvme_soft_get(v)    ((nvme_soft_t *)device_info_get((v)))

/* IP32 doesn't care but IP30 actually does
   PCIIO_DMA_DATA does not turn on barrier
   PCIIO_DMA_CMD does turn on barrier (but it is slower?)
   and apparently it can be combined with BYTE_STREAM
*/
#ifdef NVME_UTILBUF_BSWAP
#define UTILBUF_DMA_TYPE (PCIIO_DMA_CMD | PCIIO_BYTE_STREAM)
#else 
#define UTILBUF_DMA_TYPE (PCIIO_DMA_CMD | PCIIO_WORD_VALUES)
#endif

extern volatile int nvme_intcount;

#pragma set woff 3201
#pragma set woff 1174
#pragma set woff 1204
#pragma set woff 1552


extern int scsi_intr_pri;

#include <sys/kthread.h>
#include <sys/pda.h>
/* ksys/sthread.h not available for peasants */
extern void sthread_exit(void);
extern int sthread_create(char *, caddr_t, uint_t, uint_t, uint_t, uint_t,
                          st_func_t, void *, void *, void *, void *);

/* pretty much reimplementation of nano_delay() */
#if 0
static void nvme_usleep(unsigned int usec) {
    timespec_t ts;
    int rv;
    kthread_t *kt = private.p_curkthread;
    ts.tv_sec = 0;
    ts.tv_nsec = usec * 1000;

    rv = kt_lock(kt);    
    sv_bitlock_timedwait(&(kt->k_timewait), /* sync queue */
                         0, /* flags */
                         &kt->k_flags,
                         KT_LOCK, /*lock bit */
                         rv, /* bitlock return value */
                         SVTIMER_FAST,
                         &ts, /* time to wait */
                         NULL /* rts */);
}
#endif

#ifdef NVME_TEST
void
nvme_test_admin_completion(nvme_soft_t *soft, nvme_queue_t *q, nvme_completion_t *cpl);
void
nvme_cmd_admin_test(nvme_soft_t *soft, unsigned int i);
void
nvme_test_admin(nvme_soft_t *soft);

void
nvme_test_io_completion(nvme_soft_t *soft, nvme_queue_t *q, nvme_completion_t *cpl);
void
nvme_cmd_io_test(nvme_soft_t *soft, unsigned int i);
void
nvme_test_io(nvme_soft_t *soft);

#endif

#endif /* __NVMEDRV_H */
