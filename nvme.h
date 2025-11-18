#ifndef __NVME_H
#define __NVME_H

/*
 * nvme.h - NVMe Specification Definitions
 *
 * Pure NVMe spec structures, registers, and constants
 * Based on NVMe 1.0e specification
 *
 * Adapted from Windows 2000 NVMe driver for IRIX
 */

#include <sys/types.h>

/*
 * NVMe Register Offsets
 */
#define NVME_REG_CAP        0x0000  /* Controller Capabilities (8 bytes) */
#define NVME_REG_VS         0x0008  /* Version (4 bytes) */
#define NVME_REG_INTMS      0x000C  /* Interrupt Mask Set */
#define NVME_REG_INTMC      0x0010  /* Interrupt Mask Clear */
#define NVME_REG_CC         0x0014  /* Controller Configuration */
#define NVME_REG_CSTS       0x001C  /* Controller Status */
#define NVME_REG_AQA        0x0024  /* Admin Queue Attributes */
#define NVME_REG_ASQ        0x0028  /* Admin Submission Queue Base Address (8 bytes) */
#define NVME_REG_ACQ        0x0030  /* Admin Completion Queue Base Address (8 bytes) */

/*
 * Controller Capabilities Register bits
 */
#define NVME_CAP_MQES_MASK  0x0000FFFF  /* Maximum Queue Entries Supported (bits 15:0) */
#define NVME_CAP_DSTRD_SHIFT 32         /* Doorbell Stride shift */
#define NVME_CAP_DSTRD_MASK 0xF         /* Doorbell Stride mask */

/*
 * Controller Configuration Register bits
 */
#define NVME_CC_ENABLE      0x00000001
#define NVME_CC_CSS_NVM     0x00000000
#define NVME_CC_MPS_SHIFT   7
#define NVME_CC_AMS_RR      0x00000000
#define NVME_CC_SHN_NONE    0x00000000
#define NVME_CC_SHN_NORMAL  0x00004000  /* Normal shutdown notification (bits 15:14 = 01b) */
#define NVME_CC_SHN_ABRUPT  0x00008000  /* Abrupt shutdown notification (bits 15:14 = 10b) */
#define NVME_CC_SHN_MASK    0x0000C000  /* Shutdown notification mask */
#define NVME_CC_IOSQES      0x00060000  /* I/O Submission Queue Entry Size (64 bytes = 6) */
#define NVME_CC_IOCQES      0x00400000  /* I/O Completion Queue Entry Size (16 bytes = 4) */

/*
 * Controller Status Register bits
 */
#define NVME_CSTS_RDY       0x00000001
#define NVME_CSTS_CFS       0x00000002  /* Controller Fatal Status */
#define NVME_CSTS_SHST_MASK 0x0000000C  /* Shutdown Status mask (bits 3:2) */
#define NVME_CSTS_SHST_NORMAL 0x00000000  /* Normal operation (no shutdown) */
#define NVME_CSTS_SHST_OCCURRING 0x00000004  /* Shutdown processing occurring (bits 3:2 = 01b) */
#define NVME_CSTS_SHST_COMPLETE 0x00000008  /* Shutdown processing complete (bits 3:2 = 10b) */

/*
 * NVMe Admin Command Opcodes
 */
#define NVME_ADMIN_DELETE_SQ    0x00
#define NVME_ADMIN_CREATE_SQ    0x01
#define NVME_ADMIN_GET_LOG_PAGE 0x02
#define NVME_ADMIN_DELETE_CQ    0x04
#define NVME_ADMIN_CREATE_CQ    0x05
#define NVME_ADMIN_IDENTIFY     0x06
#define NVME_ADMIN_ABORT        0x08
#define NVME_ADMIN_SET_FEATURES 0x09
#define NVME_ADMIN_GET_FEATURES 0x0A

/*
 * NVMe I/O Command Opcodes
 */
#define NVME_CMD_FLUSH          0x00
#define NVME_CMD_WRITE          0x01
#define NVME_CMD_READ           0x02
#define NVME_CMD_COMPARE        0x05
#define NVME_CMD_ZERO           0x08
#define NVME_CMD_DSM            0x09  /* Dataset Management (TRIM/UNMAP) */
#define NVME_CMD_VERIFY         0x0C

/*
 * NVMe Identify CNS values
 */
#define NVME_CNS_NAMESPACE    0x00
#define NVME_CNS_CONTROLLER   0x01

/*
 * NVMe Log Page Identifiers
 */
#define NVME_LOG_PAGE_ERROR_INFO        0x01
#define NVME_LOG_PAGE_SMART_HEALTH      0x02
#define NVME_LOG_PAGE_FW_SLOT_INFO      0x03

/*
 * NVMe Error Information Log Entry (64 bytes)
 * Per NVMe 1.0e specification, Figure 73
 */
typedef struct _nvme_error_log_entry {
    __uint32_t error_count_lo;      /* Offset 0: Error Count low (63:32 in high) */
    __uint32_t error_count_hi;      /* Offset 4: Error Count high */
    __uint32_t sqid_cid;            /* Offset 8: SQID (15:0), CID (31:16) */
    __uint32_t status_pstat_loc;    /* Offset 12: Status (15:0), Param Error Loc (23:16), reserved (31:24) */
    __uint32_t lba_lo;              /* Offset 16: LBA low 32 bits */
    __uint32_t lba_hi;              /* Offset 20: LBA high 32 bits */
    __uint32_t nsid;                /* Offset 24: Namespace ID */
    uchar_t vendor_specific;        /* Offset 28: Vendor Specific Info Avail */
    uchar_t reserved[35];           /* Offset 29-63: Reserved */
} nvme_error_log_entry_t;

/*
 * NVMe Status Codes (Status Code field, bits 7:1 of Status Word DW3[15:0])
 * Per NVMe 1.0e specification, Figure 38
 */

/* Generic Command Status (Status Code Type = 0x0) */
#define NVME_SC_SUCCESS                     0x00  /* Successful Completion */
#define NVME_SC_INVALID_OPCODE              0x01  /* Invalid Command Opcode */
#define NVME_SC_INVALID_FIELD               0x02  /* Invalid Field in Command */
#define NVME_SC_CMDID_CONFLICT              0x03  /* Command ID Conflict */
#define NVME_SC_DATA_XFER_ERROR             0x04  /* Data Transfer Error */
#define NVME_SC_POWER_LOSS                  0x05  /* Commands Aborted due to Power Loss Notification */
#define NVME_SC_INTERNAL                    0x06  /* Internal Device Error */
#define NVME_SC_ABORT_REQ                   0x07  /* Command Abort Requested */
#define NVME_SC_ABORT_QUEUE                 0x08  /* Command Aborted due to SQ Deletion */
#define NVME_SC_FUSED_FAIL                  0x09  /* Command Aborted due to Failed Fused Command */
#define NVME_SC_FUSED_MISSING               0x0A  /* Command Aborted due to Missing Fused Command */
#define NVME_SC_INVALID_NS                  0x0B  /* Invalid Namespace or Format */
#define NVME_SC_CMD_SEQ_ERROR               0x0C  /* Command Sequence Error */
#define NVME_SC_INVALID_SGL_SEG_DESC        0x0D  /* Invalid SGL Segment Descriptor (NVMe 1.1+) */
#define NVME_SC_INVALID_NUM_SGL_DESC        0x0E  /* Invalid Number of SGL Descriptors (NVMe 1.1+) */
#define NVME_SC_DATA_SGL_LEN_INVALID        0x0F  /* Data SGL Length Invalid (NVMe 1.1+) */
#define NVME_SC_METADATA_SGL_LEN_INVALID    0x10  /* Metadata SGL Length Invalid (NVMe 1.1+) */
#define NVME_SC_SGL_DESC_TYPE_INVALID       0x11  /* SGL Descriptor Type Invalid (NVMe 1.1+) */

/* Command Specific Status (Status Code Type = 0x0) */
#define NVME_SC_LBA_RANGE                   0x80  /* LBA Out of Range */
#define NVME_SC_CAP_EXCEEDED                0x81  /* Capacity Exceeded */
#define NVME_SC_NS_NOT_READY                0x82  /* Namespace Not Ready */
#define NVME_SC_RESERVATION_CONFLICT        0x83  /* Reservation Conflict (NVMe 1.1+) */

/* Media Errors (Status Code Type = 0x2) */
#define NVME_SC_WRITE_FAULT                 0x80  /* Write Fault */
#define NVME_SC_READ_ERROR                  0x81  /* Unrecovered Read Error */
#define NVME_SC_GUARD_CHECK                 0x82  /* End-to-end Guard Check Error */
#define NVME_SC_APPTAG_CHECK                0x83  /* End-to-end Application Tag Check Error */
#define NVME_SC_REFTAG_CHECK                0x84  /* End-to-end Reference Tag Check Error */
#define NVME_SC_COMPARE_FAILED              0x85  /* Compare Failure */
#define NVME_SC_ACCESS_DENIED               0x86  /* Access Denied */

/*
 * Queue Flags (for CDW11 in Create I/O Queue commands)
 */
#define NVME_QUEUE_PHYS_CONTIG  0x0001  /* Bit 0: PC (Physically Contiguous) */
#define NVME_QUEUE_IRQ_ENABLED  0x0002  /* Bit 1: IEN (Interrupts Enabled) */

/*
 * Command Dword 0 fields
 */
#define NVME_CMD_PRP            0x00
#define NVME_CMD_SGL            0x40

/*
 * Queue sizes and scatter-gather limits
 */
#define NVME_SQ_ENTRY_SIZE      64      /* Submission Queue Entry size */
#define NVME_CQ_ENTRY_SIZE      16      /* Completion Queue Entry size */

#define NVME_BAR0_SIZE          0x2000

/*
 * NVMe Submission Queue Entry
 * All 32-bit fields to avoid endianness issues on DMA
 * 64-bit values (mptr, prp) split into low/high dwords
 *
 * CDW0 format: Opcode (7:0), Flags (15:8), CID (31:16)
 */
typedef struct _nvme_command {
    __uint32_t cdw0;        /* Command Dword 0: Opcode (7:0), Flags (15:8), CID (31:16) */
    __uint32_t nsid;        /* Namespace ID */
    __uint32_t cdw2;
    __uint32_t cdw3;
    __uint32_t mptr_lo;     /* Metadata Pointer - low 32 bits */
    __uint32_t mptr_hi;     /* Metadata Pointer - high 32 bits */
    __uint32_t prp1_lo;     /* Physical Region Page 1 - low 32 bits */
    __uint32_t prp1_hi;     /* Physical Region Page 1 - high 32 bits */
    __uint32_t prp2_lo;     /* Physical Region Page 2 - low 32 bits */
    __uint32_t prp2_hi;     /* Physical Region Page 2 - high 32 bits */
    __uint32_t cdw10;
    __uint32_t cdw11;
    __uint32_t cdw12;
    __uint32_t cdw13;
    __uint32_t cdw14;
    __uint32_t cdw15;
} nvme_command_t;

#define PHYS64_LO(p64) ((__uint32_t)(p64))
#define PHYS64_HI(p64) ((__uint32_t)((p64) >> 32))

/*
 * NVMe Completion Queue Entry (16 bytes)
 * Use 32-bit fields to avoid endianness issues - extract subfields via bit ops
 */
typedef struct _nvme_completion {
    __uint32_t dw0;         /* Command-specific */
    __uint32_t dw1;         /* Reserved */
    __uint32_t dw2;         /* SQ Head (15:0), SQ ID (31:16) */
    __uint32_t dw3;         /* CID (15:0), Phase/Status (31:16) */
} nvme_completion_t;

/*
 * NVMe Identify Controller Structure (partial)
 * Mixed 32-bit fields and byte arrays (strings don't need alignment)
 */
typedef struct _nvme_identify_controller {
    __uint32_t vid_ssvid;               /* Offset 0: VID (15:0), SSVID (31:16) */
    uchar_t serial_number[20];          /* Offset 4 */
    uchar_t model_number[40];           /* Offset 24 */
    uchar_t firmware_revision[8];       /* Offset 64 */
    uchar_t rab;                        /* Offset 72: RAB - Recommended Arbitration Burst */
    uchar_t ieee_oui[3];                /* Offset 73-75: IEEE OUI Identifier */
    uchar_t cmic;                       /* Offset 76: Controller Multi-Path I/O and Namespace Sharing Capabilities */
    uchar_t mdts;                       /* Offset 77: MDTS - Maximum Data Transfer Size (2^n pages, 0=unlimited) */
    uchar_t reserved1[438];             /* Offset 78-515 */
    __uint32_t number_of_namespaces;    /* Offset 516 (NN field) */
    uchar_t reserved2[3576];            /* Offset 520-4095 (rest of 4096 byte structure) */
} nvme_identify_controller_t;

/*
 * NVMe LBA Format Structure (used in Identify Namespace)
 * 32-bit field: MS (15:0), LBADS (23:16), RP (31:24)
 */
typedef struct _nvme_lba_format {
    __uint32_t dw0;     /* MS (15:0), LBADS (23:16), RP (31:24) */
} nvme_lba_format_t;

/*
 * NVMe Identify Namespace Structure (partial)
 * 64-bit values split into low/high 32-bit fields
 * Byte arrays (strings/GUIDs) don't need special alignment
 */
typedef struct _nvme_identify_namespace {
    __uint32_t nsze_lo;                 /* Offset 0: NSZE low - Namespace Size */
    __uint32_t nsze_hi;                 /* Offset 4: NSZE high */
    __uint32_t ncap_lo;                 /* Offset 8: NCAP low - Namespace Capacity */
    __uint32_t ncap_hi;                 /* Offset 12: NCAP high */
    __uint32_t nuse_lo;                 /* Offset 16: NUSE low - Namespace Utilization */
    __uint32_t nuse_hi;                 /* Offset 20: NUSE high */
    __uint32_t features_nlbaf_flbas_mc; /* Offset 24: NSFEAT (7:0), NLBAF (15:8), FLBAS (23:16), MC (31:24) */
    uchar_t reserved1[76];              /* Offset 28-103 */
    uchar_t nguid[16];                  /* Offset 104-119: NGUID */
    uchar_t eui64[8];                   /* Offset 120-127: EUI64 */
    nvme_lba_format_t lba_formats[16];  /* Offset 128-191: LBAF0-LBAF15 (16 x 4 bytes = 64 bytes) */
    uchar_t reserved2[3904];            /* Offset 192-4095 */
} nvme_identify_namespace_t;

#endif /* __NVME_H */
