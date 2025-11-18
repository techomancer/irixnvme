/*
 * nvme_scsi.c - SCSI to NVMe Translation Layer
 *
 * Translates SCSI commands to NVMe commands, allowing IRIX SCSI
 * upper layers (dksc, etc.) to work with NVMe devices.
 */

#include "nvmedrv.h"

/*
 * Helper: Set SCSI command error with sense data
 */
void
nvme_scsi_set_error(scsi_request_t *req, u_char sense_key, u_char asc, u_char ascq)
{
    req->sr_status = SC_GOOD;
    req->sr_scsi_status = ST_CHECK;
    req->sr_resid = req->sr_buflen;  /* No data transferred on error */

    /* Build sense data if buffer provided */
    if (req->sr_sense && req->sr_senselen >= 18) {
        bzero(req->sr_sense, req->sr_senselen);
        req->sr_sense[0] = 0x70;           /* Current error, fixed format */
        req->sr_sense[2] = sense_key;      /* Sense key */
        req->sr_sense[7] = 10;             /* Additional sense length */
        req->sr_sense[12] = asc;           /* Additional Sense Code */
        req->sr_sense[13] = ascq;          /* ASC Qualifier */
        req->sr_sensegotten = 18;
    } else {
        req->sr_sensegotten = 0;
    }
}

/*
 * nvme_scsi_test_unit_ready: Handle TEST UNIT READY
 */
int
nvme_scsi_test_unit_ready(nvme_soft_t *soft, scsi_request_t *req)
{
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_test_unit_ready: soft=%p initialized=%d",
            soft, soft->initialized);
#endif
    /* If controller is ready, we're ready */
    if (soft->initialized) {
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_test_unit_ready: returning SUCCESS");
#endif
        nvme_set_success(req);
    } else {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_test_unit_ready: returning ERROR (not initialized)");
#endif
        nvme_set_adapter_error(req);
    }

    return 0;
}

/*
 * nvme_scsi_send_diagnostic: Handle SEND DIAGNOSTIC command
 *
 * This is typically used for device self-tests. For NVMe, we just
 * return success since the controller manages its own diagnostics.
 */
int
nvme_scsi_send_diagnostic(nvme_soft_t *soft, scsi_request_t *req)
{
    uchar_t *cdb = req->sr_command;
    uchar_t self_test = (cdb[1] >> 2) & 0x01;  /* SelfTest bit */
    uchar_t pf = (cdb[1] >> 4) & 0x01;          /* Page Format bit */

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_send_diagnostic: self_test=%d pf=%d", self_test, pf);
#endif
    /* For NVMe, we just return success */
    /* Real implementation could use NVMe Device Self-test command if needed */
    nvme_set_success(req);
    return 0;
}

/*
 * nvme_build_inquiry_data: Build standard SCSI INQUIRY response from NVMe data
 *
 * Creates a 36-byte SCSI INQUIRY response using the NVMe controller's
 * model number and firmware revision.
 *
 * Arguments:
 *   soft       - NVMe controller soft state
 *   inq_data   - Buffer to fill (must be at least 36 bytes)
 */
void
nvme_build_inquiry_data(nvme_soft_t *soft, u_char *inq_data)
{
#ifdef NVME_DBG    
    cmn_err(CE_NOTE, "nvme_build_inquiry_data: soft=%p max_transfer_blocks:%d", soft, soft->max_transfer_blocks);
#endif

    /* Clear the buffer */
    bzero(inq_data, 36);

    /* Standard INQUIRY header */
    inq_data[0] = 0x00;  /* Peripheral Device Type: Direct-access device (disk) */
    inq_data[1] = 0x00;  /* RMB=0 (not removable) */
    inq_data[2] = 0x05;  /* Version: SPC-3 */
    inq_data[3] = 0x02;  /* Response Data Format: 2 */
    inq_data[4] = 31;    /* Additional Length (36 - 5) */
    inq_data[5] = 0x00;  /* SCCS=0, ACC=0, etc. */
    inq_data[6] = 0x00;  /* No special features */
    inq_data[7] = 0x02;  /* CmdQue=1 */

    /* Vendor ID (8 bytes): "NVMe    " */
    bcopy("NVMe    ", &inq_data[8], 8);

    /* Product ID (16 bytes): Use NVMe model number if available */
    if (soft->model[0] != '\0') {
        int i, len = 0;
        /* Find actual length (remove trailing spaces) */
        for (i = 39; i >= 0 && (soft->model[i] == ' ' || soft->model[i] == '\0'); i--)
            ;
        len = i + 1;
        if (len > 16) len = 16;
        bcopy(soft->model, &inq_data[16], len);
        /* Pad with spaces */
        for (i = len; i < 16; i++)
            inq_data[16 + i] = ' ';
    } else {
        /* Default if model not available */
        bcopy("IRIX NVMe Drive ", &inq_data[16], 16);
    }

    /* Product Revision (4 bytes): Use NVMe firmware revision if available */
    if (soft->firmware_rev[0] != '\0') {
        int i, len = 0;
        /* Find actual length */
        for (i = 7; i >= 0 && (soft->firmware_rev[i] == ' ' || soft->firmware_rev[i] == '\0'); i--)
            ;
        len = i + 1;
        if (len > 4) len = 4;
        bcopy(soft->firmware_rev, &inq_data[32], len);
        /* Pad with spaces */
        for (i = len; i < 4; i++)
            inq_data[32 + i] = ' ';
    } else {
        /* Default */
        bcopy("1.0 ", &inq_data[32], 4);
    }
}

/*
 * nvme_scsi_inquiry: Handle INQUIRY command
 * Supports standard INQUIRY and VPD page 0x80 (Unit Serial Number)
 */
int
nvme_scsi_inquiry(nvme_soft_t *soft, scsi_request_t *req)
{
    uchar_t *cdb = req->sr_command;
    uchar_t *buffer = (uchar_t *)req->sr_buffer;
    uchar_t evpd = cdb[1] & 0x01;  /* Enable Vital Product Data */
    uchar_t page_code = cdb[2];
    int copy_len;
    int i, sn_len;

    /* Check if VPD page requested */
    if (evpd) {
        /* VPD Page requested */
        if (page_code == 0x00) {
            /* Supported VPD Pages */
            if (req->sr_buflen < 6) {
                nvme_set_adapter_error(req);
                return -1;
            }
            buffer[0] = 0x00;  /* Peripheral Device Type: Direct-access */
            buffer[1] = 0x00;  /* Page Code: Supported VPD Pages */
            buffer[2] = 0x00;  /* Reserved */
            buffer[3] = 0x03;  /* Page Length: 3 pages */
            buffer[4] = 0x00;  /* Page 0x00 (this page) */
            buffer[5] = 0x80;  /* Page 0x80 (Unit Serial Number) */
            buffer[6] = 0xB0;  /* Page 0xB0 (Block Limits) */
            copy_len = 7;
        } else if (page_code == 0x80) {
            /* Unit Serial Number Page */
            /* Find actual serial number length (trim trailing spaces) */
            for (sn_len = 20; sn_len > 0 && (soft->serial[sn_len-1] == ' ' || soft->serial[sn_len-1] == '\0'); sn_len--)
                ;

            if (req->sr_buflen < (4 + sn_len)) {
                nvme_set_adapter_error(req);
                return -1;
            }
            buffer[0] = 0x00;  /* Peripheral Device Type */
            buffer[1] = 0x80;  /* Page Code: Unit Serial Number */
            buffer[2] = 0x00;  /* Reserved */
            buffer[3] = sn_len; /* Page Length */
            bcopy(soft->serial, &buffer[4], sn_len);
            copy_len = 4 + sn_len;
        } else if (page_code == 0xB0) {
            /* Block Limits VPD Page (SBC-3) */
            if (req->sr_buflen < 64) {
                nvme_set_adapter_error(req);
                return -1;
            }
            bzero(buffer, 64);
            buffer[0] = 0x00;  /* Peripheral Device Type: Direct-access */
            buffer[1] = 0xB0;  /* Page Code: Block Limits */
            buffer[2] = 0x00;  /* Reserved */
            buffer[3] = 0x3C;  /* Page Length: 60 bytes (0x3C) */

            /* Optimal Transfer Length Granularity: not specified */
            buffer[4] = 0x00;
            buffer[5] = 0x00;

            /* Maximum Transfer Length (blocks): use controller's MDTS limit */
            buffer[8] = (soft->max_transfer_blocks >> 24) & 0xFF;
            buffer[9] = (soft->max_transfer_blocks >> 16) & 0xFF;
            buffer[10] = (soft->max_transfer_blocks >> 8) & 0xFF;
            buffer[11] = soft->max_transfer_blocks & 0xFF;

            /* Optimal Transfer Length: same as maximum */
            buffer[12] = (soft->max_transfer_blocks >> 24) & 0xFF;
            buffer[13] = (soft->max_transfer_blocks >> 16) & 0xFF;
            buffer[14] = (soft->max_transfer_blocks >> 8) & 0xFF;
            buffer[15] = soft->max_transfer_blocks & 0xFF;

            /* All other fields remain zero (bzero above) */
            copy_len = 64;
        } else {
            /* Unsupported VPD page */
            cmn_err(CE_WARN, "nvme_scsi_inquiry: unsupported VPD page 0x%x", page_code);
            nvme_scsi_set_error(req, 0x05, 0x24, 0x00);  /* ILLEGAL REQUEST, Invalid field in CDB */
            return -1;
        }
    } else {
        /* Standard INQUIRY */
        uchar_t inquiry_data[36];

        /* Build INQUIRY response using controller info */
        nvme_build_inquiry_data(soft, inquiry_data);

        if (req->sr_buflen < sizeof(inquiry_data)) {
            copy_len = req->sr_buflen;
        } else {
            copy_len = sizeof(inquiry_data);
        }

        bcopy(inquiry_data, buffer, copy_len);
        {
            uint_t csts;
            cmn_err(CE_WARN, "nvme: outstanding reqs: %d,%d", soft->admin_queue.outstanding, soft->io_queue.outstanding);
            csts = NVME_RD(soft, NVME_REG_CSTS);
            cmn_err(CE_WARN, "CSTS (0x1C): 0x%08x", csts);
            cmn_err(CE_CONT, "  RDY (Ready):                 %u %s",
                    csts & 1, (csts & 1) ? "[READY]" : "[NOT READY]");
            cmn_err(CE_CONT, "  CFS (Controller Fatal):      %u %s",
                    (csts >> 1) & 1, ((csts >> 1) & 1) ? "[FATAL ERROR!]" : "[OK]");
            cmn_err(CE_CONT, "  SHST (Shutdown Status):      %u", (csts >> 2) & 3);
            cmn_err(CE_CONT, "  NSSRO (NVM Subsys Reset):    %u", (csts >> 4) & 1);
        }
    }

    nvme_set_success(req);
    req->sr_resid = req->sr_buflen - copy_len;

    return 0;
}

/*
 * nvme_scsi_read_capacity: Handle READ CAPACITY(10)
 */
int
nvme_scsi_read_capacity(nvme_soft_t *soft, scsi_request_t *req)
{
    uchar_t *buf;
    uint_t last_lba;
    uint_t block_size;

    /* READ CAPACITY returns last LBA (not size) */
    last_lba = (uint_t)(soft->num_blocks - 1);
    block_size = soft->block_size;

    if (req->sr_buflen < 8) {
        nvme_set_adapter_error(req);
        return -1;
    }

    buf = (uchar_t *)req->sr_buffer;

    /* Return in big-endian format */
    buf[0] = (last_lba >> 24) & 0xFF;
    buf[1] = (last_lba >> 16) & 0xFF;
    buf[2] = (last_lba >> 8) & 0xFF;
    buf[3] = last_lba & 0xFF;

    buf[4] = (block_size >> 24) & 0xFF;
    buf[5] = (block_size >> 16) & 0xFF;
    buf[6] = (block_size >> 8) & 0xFF;
    buf[7] = block_size & 0xFF;

    nvme_set_success(req);
    req->sr_resid = req->sr_buflen - 8;

    return 0;
}

/*
 * nvme_scsi_mode_sense: Handle MODE SENSE(6) command
 */
int
nvme_scsi_mode_sense(nvme_soft_t *soft, scsi_request_t *req)
{
    uchar_t *cdb = req->sr_command;
    uchar_t *buffer = (uchar_t *)req->sr_buffer;
    uchar_t pageCode;
    uchar_t pageControl;
    uchar_t dbd;  /* Disable Block Descriptors */
    uint_t offset;
    uint_t headerSize;
    uint_t blockDescLength;
    uint_t totalLength;
    uint_t blockSize;
    __uint64_t numBlocks;

    /* Parse CDB for MODE SENSE(6) */
    pageCode = cdb[2] & 0x3F;
    pageControl = (cdb[2] >> 6) & 0x03;
    dbd = (cdb[1] >> 3) & 0x01;
    headerSize = 4;  /* MODE SENSE(6) header is 4 bytes */

    /* Check buffer size */
    if (req->sr_buflen < headerSize) {
        nvme_set_adapter_error(req);
        return -1;
    }

    /* Validate requested page */
    if (pageCode != MODE_SENSE_RETURN_ALL &&
        pageCode != MODE_PAGE_FORMAT_DEVICE &&
        pageCode != MODE_PAGE_CACHING &&
        pageCode != MODE_PAGE_CONTROL &&
        pageCode != MODE_PAGE_RIGID_GEOMETRY &&
        pageCode != MODE_PAGE_POWER_CONDITION &&
        pageCode != MODE_PAGE_FAULT_REPORTING) {
        nvme_scsi_set_error(req, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);  /* Invalid field in CDB */
        return -1;
    }

    /* Clear buffer */
    bzero(buffer, req->sr_buflen);

    /* Determine block descriptor length */
    blockDescLength = dbd ? 0 : 8;
    offset = headerSize;

    blockSize = soft->block_size;
    numBlocks = soft->num_blocks;

    /* Add block descriptor if not disabled */
    if (!dbd && (offset + blockDescLength) <= req->sr_buflen) {
        uchar_t *blockDesc = buffer + offset;

        /* Block descriptor format (8 bytes) */
        blockDesc[0] = 0x00;  /* Density code (0 = default) */

        /* Bytes 1-3: Number of blocks (or 0xFFFFFF if > 24-bit) */
        if (numBlocks > 0xFFFFFF) {
            blockDesc[1] = 0xFF;
            blockDesc[2] = 0xFF;
            blockDesc[3] = 0xFF;
        } else {
            blockDesc[1] = (uchar_t)((numBlocks >> 16) & 0xFF);
            blockDesc[2] = (uchar_t)((numBlocks >> 8) & 0xFF);
            blockDesc[3] = (uchar_t)(numBlocks & 0xFF);
        }

        blockDesc[4] = 0x00;  /* Reserved */

        /* Bytes 5-7: Block length (big-endian) */
        blockDesc[5] = (uchar_t)((blockSize >> 16) & 0xFF);
        blockDesc[6] = (uchar_t)((blockSize >> 8) & 0xFF);
        blockDesc[7] = (uchar_t)(blockSize & 0xFF);

        offset += blockDescLength;
    }

    /* Add mode pages based on pageCode */
    if (pageControl == MODE_SENSE_CHANGEABLE_VALUES) {
        /* Return all zeros for changeable values (nothing is changeable) */
        /* Header already zeroed, just set lengths */
    } else if (pageCode == MODE_SENSE_RETURN_ALL || pageCode == MODE_PAGE_CACHING) {
        /* Page 08h: Caching Parameters Page */
        if ((offset + 20) <= req->sr_buflen) {
            uchar_t *cachePage = buffer + offset;

            cachePage[0] = MODE_PAGE_CACHING;  /* Page code */
            cachePage[1] = 18;  /* Page length (n-1, total 20 bytes) */
            cachePage[2] = 0x04;  /* WCE=1 (write cache enabled), RCD=0 */
            cachePage[3] = 0x00;  /* Read/write retention priority */
            cachePage[4] = 0x00;  /* Disable pre-fetch transfer length */
            cachePage[5] = 0x00;
            cachePage[6] = 0x00;  /* Minimum pre-fetch */
            cachePage[7] = 0x00;
            cachePage[8] = 0xFF;  /* Maximum pre-fetch */
            cachePage[9] = 0xFF;
            cachePage[10] = 0xFF;  /* Maximum pre-fetch ceiling */
            cachePage[11] = 0xFF;
            cachePage[12] = 0x00;  /* Flags */
            cachePage[13] = 0x00;  /* Number of cache segments */
            cachePage[14] = 0x00;  /* Cache segment size */
            cachePage[15] = 0x00;
            /* Bytes 16-19: Reserved */

            offset += 20;
        }
    }

    if (pageCode == MODE_SENSE_RETURN_ALL || pageCode == MODE_PAGE_CONTROL) {
        /* Page 0Ah: Control Mode Page */
        if ((offset + 12) <= req->sr_buflen) {
            uchar_t *controlPage = buffer + offset;

            controlPage[0] = MODE_PAGE_CONTROL;  /* Page code */
            controlPage[1] = 10;  /* Page length (n-1, total 12 bytes) */
            controlPage[2] = 0x00;  /* TST, TMF_ONLY */
            controlPage[3] = 0x00;  /* QERR=00b (restricted reordering) */
            /* Bytes 4-11: Reserved/zero */

            offset += 12;
        }
    }

    /* Calculate total length */
    totalLength = offset;

    /* Fill in MODE SENSE(6) header */
    buffer[0] = (uchar_t)(totalLength - 1);  /* Mode data length (excludes length field) */
    buffer[1] = 0x00;  /* Medium type (0 = default) */
    buffer[2] = 0x00;  /* Device-specific parameter */
    buffer[3] = (uchar_t)blockDescLength;

    /* Set residual */
    if (totalLength > req->sr_buflen) {
        totalLength = req->sr_buflen;
    }

    nvme_set_success(req);
    req->sr_resid = req->sr_buflen - totalLength;

    return 0;
}

/*
 * nvme_scsi_sync_cache: Handle SYNC CACHE command
 */
int
nvme_scsi_sync_cache(nvme_soft_t *soft, scsi_request_t *req)
{
    nvme_cmd_info_t *cmd_info;
    nvme_command_t cmd;
    unsigned int cid;
    int rc;

    /* Allocate a CID for this I/O command */
    if (nvme_io_cid_alloc(soft, req, 1, &cid) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_sync_cache: no free CID available");
#endif
        nvme_set_adapter_status(req, SC_REQUEST, ST_BUSY);
        return -1;
    }

    /* Build NVMe FLUSH command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_CMD_FLUSH | (cid << 16);

    /* Set namespace ID to 1 (we always use namespace 1) */
    cmd.nsid = 1;

    /* Submit the command to the I/O queue */
    rc = nvme_submit_cmd(soft, &soft->io_queue, &cmd);
    if (rc != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_sync_cache: failed to submit flush command");
#endif
        nvme_io_cid_done(soft, cid, NULL);
        nvme_set_adapter_status(req, SC_REQUEST, ST_BUSY);
        return -1;
    }

    /* Command submitted successfully - completion will be handled by interrupt */
    return 0;
}

int
nvme_parse_rw(nvme_soft_t *soft, nvme_rwcmd_state_t *ps)
{
    u_char *cdb = ps->req->sr_command;

    switch (cdb[0]) {
    case SCSIOP_READ_6:
    case SCSIOP_WRITE_6:
        /* READ(6)/WRITE(6) format:
         * Byte 0: Opcode
         * Byte 1: LBA bits 20-16 (5 bits) in bits 4-0
         * Byte 2: LBA bits 15-8
         * Byte 3: LBA bits 7-0
         * Byte 4: Transfer length (0 = 256 blocks)
         * Byte 5: Control
         */
        ps->lba = ((__uint64_t)(cdb[1] & 0x1F) << 16) |
                  ((__uint64_t)cdb[2] << 8) |
                  ((__uint64_t)cdb[3]);
        ps->num_blocks = cdb[4];
        if (ps->num_blocks == 0) {
            ps->num_blocks = 256;  /* 0 means 256 blocks in READ(6)/WRITE(6) */
        }
        if (cdb[0] == SCSIOP_WRITE_6)
            ps->flags |= NF_WRITE;
        break;

    case SCSIOP_READ_10:
    case SCSIOP_WRITE_10:
        /* READ(10)/WRITE(10) format:
         * Byte 0: Opcode
         * Byte 1: Flags
         * Byte 2: LBA bits 31-24
         * Byte 3: LBA bits 23-16
         * Byte 4: LBA bits 15-8
         * Byte 5: LBA bits 7-0
         * Byte 6: Reserved
         * Byte 7: Transfer length bits 15-8
         * Byte 8: Transfer length bits 7-0
         * Byte 9: Control
         */
        ps->lba = ((__uint64_t)cdb[2] << 24) |
                  ((__uint64_t)cdb[3] << 16) |
                  ((__uint64_t)cdb[4] << 8) |
                  ((__uint64_t)cdb[5]);
        ps->num_blocks = ((uint_t)cdb[7] << 8) | ((uint_t)cdb[8]);
        if (cdb[0] == SCSIOP_WRITE_10)
            ps->flags |= NF_WRITE;
        break;

    case SCSIOP_READ_16:
    case SCSIOP_WRITE_16:
        /* READ(16)/WRITE(16) format:
         * Byte 0: Opcode
         * Byte 1: Flags
         * Byte 2: LBA bits 63-56
         * Byte 3: LBA bits 55-48
         * Byte 4: LBA bits 47-40
         * Byte 5: LBA bits 39-32
         * Byte 6: LBA bits 31-24
         * Byte 7: LBA bits 23-16
         * Byte 8: LBA bits 15-8
         * Byte 9: LBA bits 7-0
         * Byte 10: Transfer length bits 31-24
         * Byte 11: Transfer length bits 23-16
         * Byte 12: Transfer length bits 15-8
         * Byte 13: Transfer length bits 7-0
         * Byte 14: Flags
         * Byte 15: Control
         */
        ps->lba = ((__uint64_t)cdb[2] << 56) |
                  ((__uint64_t)cdb[3] << 48) |
                  ((__uint64_t)cdb[4] << 40) |
                  ((__uint64_t)cdb[5] << 32) |
                  ((__uint64_t)cdb[6] << 24) |
                  ((__uint64_t)cdb[7] << 16) |
                  ((__uint64_t)cdb[8] << 8) |
                  ((__uint64_t)cdb[9]);
        ps->num_blocks = ((uint_t)cdb[10] << 24) |
                         ((uint_t)cdb[11] << 16) |
                         ((uint_t)cdb[12] << 8) |
                         ((uint_t)cdb[13]);
        if (cdb[0] == SCSIOP_WRITE_16)
            ps->flags |= NF_WRITE;
        break;

    default:
        cmn_err(CE_WARN, "nvme_parse_rw: unsupported CDB opcode 0x%02x",
                cdb[0]);
        return 0;
    }

    return 1;
}


/*
 * nvme_scsi_read_write: Handle READ/WRITE commands
 */
void
nvme_scsi_read_write(nvme_soft_t *soft, scsi_request_t *req)
{
    nvme_rwcmd_state_t s;
    int rc;

    /* Reject zero-length transfers */
    if (req->sr_buflen == 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_read_write: zero-length transfer rejected");
#endif
        nvme_set_success(req);
        if (req->sr_notify) {
            req->sr_ha = NULL;
            (*req->sr_notify)(req);
        }
        return;
    }

    /* Initialize refcount atomically to 1 (will be incremented by nvme_io_cid_alloc) */
    *(volatile int *)&(req->sr_ha) = 1;

    /* Initialize SCSI status to success (errors will override this) */
    nvme_set_success(req);

    s.req = req;
    s.buflen = req->sr_buflen;
    s.flags = 0;
    s.max_transfer_blocks = soft->max_transfer_blocks;
    if (!nvme_parse_rw(soft, &s)) {
        nvme_set_adapter_error(req);
        goto error;
    }

    /* Check if this is a retry of an aborted command */
    if (nvme_aborted_fifo_find_and_remove(soft, &s)) {
        s.flags |= NF_RETRY;
        s.max_transfer_blocks = 1;
        cmn_err(CE_WARN, "nvme_scsi_read_write: RETRY DETECTED buflen=%u buffer=%p bp=%p sr_flags=0x%x nf_flags=0x%x",
                req->sr_buflen, req->sr_buffer, req->sr_bp, req->sr_flags, s.flags);
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_read_write: ENTRY buflen=%d buffer=%p flags=0x%x tag=%d",
            req->sr_buflen, req->sr_buffer, req->sr_flags, req->sr_tag);
#endif

    /*
     * For ordered or head-of-queue tagged commands, issue a special flush first
     * to ensure proper ordering semantics. The flush will complete before this
     * command starts executing.
     */
    if (req->sr_tag == SC_TAG_ORDERED || req->sr_tag == SC_TAG_HEAD) {
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_read_write: issuing special flush before %s command",
                req->sr_tag == SC_TAG_ORDERED ? "ordered" : "head-of-queue");
#endif
        rc = nvme_cmd_special_flush(soft);
        if (rc != 0) {
            cmn_err(CE_WARN, "nvme_scsi_read_write: special flush failed");
            /* Continue anyway - best effort */
        }
    }

    if (req->sr_buflen > s.max_transfer_blocks * soft->block_size) {
        s.commands = (req->sr_buflen + s.max_transfer_blocks * soft->block_size - 1) / (s.max_transfer_blocks * soft->block_size);
    } else {
        s.commands = 1;
    }
    /* Prepare alenlist before allocating CIDs (initializes cursor at offset 0) */
    rc = nvme_prepare_alenlist(soft, &s);
    if (rc <= 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_read_write: failed to prepare alenlist");
#endif
        if (rc == 0)
            nvme_set_adapter_error(req);
        goto error;
    }
    if (!s.alenlist)
        goto error;

    /* Allocate CID(s) for this I/O command */
    if (nvme_io_cid_alloc(soft, req, s.commands, s.cids) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_read_write: no free CIDs available (requested %u)", s.commands);
#endif
        nvme_set_adapter_status(req, SC_REQUEST, ST_BUSY);
        goto error_cleanup_alenlist;
    }

    /* Process each command/CID */
    for (s.cidx = 0; s.cidx < s.commands; s.cidx++) {

        /* Build the NVMe READ/WRITE command (sets opcode, nsid, LBA, num_blocks) */
#ifdef NVME_DBG_CMD
        cmn_err(CE_WARN, "nvme_scsi_read_write: building NVMe command %u/%u (CID %u)...", s.cidx+1, s.commands, s.cids[s.cidx]);
#endif
        rc = nvme_io_build_rw_command(soft, &s);
        if (rc <= 0) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_read_write: failed to build NVMe command %u", s.cidx);
#endif
            if (rc == 0)
                nvme_set_adapter_error(req);
            goto error_cleanup_cids;
        }
#ifdef NVME_DBG_CMD
        cmn_err(CE_NOTE, "nvme_scsi_read_write: NVMe command %u built successfully", s.cidx);
#endif

        /* Build PRP entries for data transfer (sets prp1/prp2, allocates PRP list if needed) */
#ifdef NVME_DBG_CMD
        cmn_err(CE_NOTE, "nvme_scsi_read_write: building PRPs for command %u...", s.cidx);
#endif
        rc = nvme_build_prps_from_alenlist(soft, &s);
        if (rc <= 0) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_read_write: failed to build PRPs for command %u (rc=%d)", s.cidx, rc);
#endif
            if (rc == 0) {
                /* Hard error - set adapter error */
                nvme_set_adapter_error(req);
            }
            /* rc == -1: BUSY already set by nvme_build_prps_from_alenlist */
            goto error_cleanup_cids;
        }
#ifdef NVME_DBG_CMD
        cmn_err(CE_NOTE, "nvme_scsi_read_write: PRPs built successfully for command %u, prp1=0x%x%08x prp2=0x%x%08x blocks=%u",
                s.cidx, s.cmd.prp1_hi, s.cmd.prp1_lo, s.cmd.prp2_hi, s.cmd.prp2_lo, s.cmd.cdw12+1);
#endif
        /* Submit the command to the I/O queue */
#ifdef NVME_DBG_CMD
        cmn_err(CE_WARN, "nvme_scsi_read_write: submitting NVMe command %u/%u (CID=%d)...", s.cidx+1, s.commands, s.cids[s.cidx]);
#endif
        rc = nvme_submit_cmd(soft, &soft->io_queue, &s.cmd);
        if (rc != 0) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_read_write: failed to submit command %u", s.cidx);
#endif
            nvme_set_adapter_status(req, SC_REQUEST, ST_BUSY);
            goto error_cleanup_cids;
        }
#ifdef NVME_DBG_CMD
        cmn_err(CE_WARN, "nvme_scsi_read_write: command %u/%u submitted to SQ, tail now at %d", s.cidx+1, s.commands, soft->io_queue.sq_tail);
#endif
    }

error_cleanup_cids:
    {
        unsigned int j;
        /* Clean up all allocated CIDs */
        for (j = s.cidx; j < s.commands; j++) {
            nvme_io_cid_done(soft, s.cids[j], NULL);
        }
    }
error_cleanup_alenlist:
    /* Clean up alenlist */
    nvme_cleanup_alenlist(soft, s.need_unlock);

error:
    /* Atomically decrement refcount by 1 (for the initial +1 at start) */
    /* If refcount reaches 0, all commands completed before we got here - call sr_notify
     * sr_ha is already 0 (NULL) from the atomic decrement
     */
    if (atomicAddInt((int *)&req->sr_ha, -1) == 0) {
        /* All commands already completed - notify now */
        if (req->sr_notify) {
            (*req->sr_notify)(req);
        }
    }
}

/*
 * nvme_scsi_command: Main entry point for SCSI command translation
 */
void
nvme_scsi_command(scsi_request_t *req)
{
    nvme_soft_t *soft;
    vertex_hdl_t scsi_vhdl;
    scsi_lun_info_t *lun_info;
    uchar_t opcode;
    int rc = 0;

    /* Get LUN info to find controller */
    lun_info = scsi_lun_info_get(req->sr_lun_vhdl);
    if (!lun_info) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_command: no lun info");
#endif
        nvme_set_adapter_error(req);
        goto done;
    }

    /* Only LUN 0 is valid for target 0 (our NVMe device) */
    if (req->sr_lun != 0 && req->sr_target == 0) {
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "!nvme_scsi_command: invalid LUN %d for target 0", req->sr_lun);
#endif
        nvme_scsi_set_error(req, SCSI_SENSE_ILLEGAL_REQUEST, SCSI_ADSENSE_INVALID_LUN, 0);
        goto done;
    }

    /* Only target 0 is valid - return timeout for non-existent targets */
    if (req->sr_target != 0) {
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "!nvme_scsi_command: invalid target %d", req->sr_target);
#endif
        nvme_set_adapter_status(req, SC_TIMEOUT, ST_CHECK);
        goto done;
    }

    /* Get controller info and extract our soft state from SCI_INFO (like ql.c does) */
    {
        scsi_ctlr_info_t *ctlr_info = SLI_CTLR_INFO(lun_info);
        if (!ctlr_info) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_command: no controller info");
#endif
            nvme_set_adapter_error(req);
            goto done;
        }
        soft = (nvme_soft_t *)SCI_INFO(ctlr_info);
        if (!soft) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_command: no soft state in SCI_INFO");
#endif
            nvme_set_adapter_error(req);
            goto done;
        }
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_command: soft=%p req=%p notify=%p target=%d lun=%d opcode=0x%x sr_status=0x%x",
            soft, req, req->sr_notify, req->sr_target, req->sr_lun, req->sr_command[0], req->sr_status);
#endif
    /* Decode SCSI opcode */
    opcode = req->sr_command[0];

    switch (opcode) {
    case SCSIOP_TEST_UNIT_READY:
        rc = nvme_scsi_test_unit_ready(soft, req);
        break;

    case SCSIOP_INQUIRY:
        rc = nvme_scsi_inquiry(soft, req);
        break;

    case SCSIOP_SEND_DIAGNOSTIC:
        rc = nvme_scsi_send_diagnostic(soft, req);
        break;

    case SCSIOP_READ_CAPACITY_10:
        rc = nvme_scsi_read_capacity(soft, req);
        break;

    case SCSIOP_READ_6:
    case SCSIOP_READ_10:
    case SCSIOP_READ_16:
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_command: READ command - calling read_write handler");
#endif
        nvme_scsi_read_write(soft, req);
        return; // notify always called by nvme_scsi_read_write
    case SCSIOP_WRITE_6:
    case SCSIOP_WRITE_10:
    case SCSIOP_WRITE_16:
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_command: WRITE command - calling read_write handler");
#endif
        nvme_scsi_read_write(soft, req);
        return; // notify always called by nvme_scsi_read_write

    case SCSIOP_MODE_SENSE_6:
        rc = nvme_scsi_mode_sense(soft, req);
        break;

    case SCSIOP_SYNC_CACHE:
        rc = nvme_scsi_sync_cache(soft, req);
        break;

    default:
        cmn_err(CE_WARN, "nvme_scsi_command: unsupported opcode 0x%x", opcode);
        nvme_set_adapter_error(req);
        break;
    }

done:
    if (req->sr_notify) {
        req->sr_ha = NULL;
        (*req->sr_notify)(req);
    }
}


/*
 * nvme_scsi_alloc: Allocate SCSI device resources
 */
int
nvme_scsi_alloc(vertex_hdl_t lun_vhdl, int opt, void (*cb)())
{
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_alloc: allocating device");
#endif
    return SCSIALLOCOK;
}

/*
 * nvme_scsi_free: Free SCSI device resources
 */
void
nvme_scsi_free(vertex_hdl_t lun_vhdl, void (*cb)())
{
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_free: freeing device");
#endif
}

/*
 * nvme_init_scsi_target_info: Initialize the SCSI target info structure
 * This can be called multiple times to ensure the structure is always valid
 */
void
nvme_init_scsi_target_info(nvme_soft_t *soft)
{
    /* Set pointers to our persistent buffers */
    soft->tinfo.si_inq = soft->inq_data;
    soft->tinfo.si_sense = soft->sense_data;

    /* Set capability flags */
    soft->tinfo.si_ha_status = SRH_TAGQ | SRH_QERR0 | SRH_ALENLIST | SRH_MAPUSER | SRH_WIDE;
    soft->tinfo.si_maxq = 32;  /* Max queue depth */
    soft->tinfo.si_qdepth = 0;
    soft->tinfo.si_qlimit = 0;
}

/*
 * nvme_scsi_info: Return SCSI device information
 */
struct scsi_target_info *
nvme_scsi_info(vertex_hdl_t lun_vhdl)
{
    nvme_soft_t *soft;
    scsi_lun_info_t *lun_info;
    vertex_hdl_t ctlr_vhdl;

    /* Get controller soft state */
    lun_info = scsi_lun_info_get(lun_vhdl);
    if (!lun_info) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_info: no lun_info");
#endif
        return NULL;
    }

    /* Get our soft state from SCI_INFO (like ql.c does) */
    {
        scsi_ctlr_info_t *ctlr_info = SLI_CTLR_INFO(lun_info);
        if (!ctlr_info) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_info: no controller info");
#endif
            return NULL;
        }
        soft = (nvme_soft_t *)SCI_INFO(ctlr_info);
        if (!soft) {
#ifdef NVME_DBG
            cmn_err(CE_WARN, "nvme_scsi_info: no soft state in SCI_INFO");
#endif
            return NULL;
        }
    }

    /* Re-initialize the target info structure every time to work around corruption */
    nvme_init_scsi_target_info(soft);

    /* Return pointer to persistent info structure in soft state */
    return &soft->tinfo;
}

/*
 * nvme_scsi_dump: Crash dump support
 */
int
nvme_scsi_dump(vertex_hdl_t ctlr_vhdl)
{
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_dump: dump requested");
#endif
    return 0;
}

/*
 * nvme_scsi_abort: Abort a SCSI command
 */
int
nvme_scsi_abort(scsi_request_t *req)
{
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_abort: abort requested");
#endif
    return 1;
}

/*
 * nvme_scsi_ioctl: Handle SCSI ioctl commands
 */
int
nvme_scsi_ioctl(vertex_hdl_t ctlr_vhdl, unsigned int cmd, struct scsi_ha_op *op)
{
    nvme_soft_t *soft;
    scsi_ctlr_info_t *ctlr_info;
    int targ;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_ioctl: ioctl 0x%x", cmd);
#endif
    /* Get soft state for operations that need it */
    ctlr_info = scsi_ctlr_info_get(ctlr_vhdl);
    if (!ctlr_info) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_ioctl: no controller info");
#endif
        return EINVAL;
    }
    soft = (nvme_soft_t *)SCI_INFO(ctlr_info);
    if (!soft) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_scsi_ioctl: no soft state");
#endif
        return EINVAL;
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_scsi_ioctl: soft=%p", soft);
#endif
    switch (cmd) {
    case SOP_RESET:
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_ioctl: SOP_RESET");
#endif
        /* For NVMe, we don't need to reset the controller for this operation */
        return 0;

    case SOP_SCAN:
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_ioctl: SOP_SCAN");
#endif
        /* NVMe namespace is already discovered at attach time */
        return 0;

    case SOP_MAKE_CTL_ALIAS:
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_ioctl: SOP_MAKE_CTL_ALIAS - creating controller aliases");
#endif
        /* ioconfig uses this to create /dev/scsi/scN aliases */
        return 0;

    case SOP_QUIESCE_STATE: {
        int state;
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_ioctl: SOP_QUIESCE_STATE");        
#endif
        /* Return current quiesce state */
        state = soft->quiesce_state;
        copyout(&state, (void *)op->sb_addr, sizeof(int));
        return 0;
    }
    case SOP_GET_SCSI_PARMS:
    {
        struct scsi_parms sp;
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_scsi_ioctl: SOP_GET_SCSI_PARMS");
#endif
        /* Build SCSI parameters structure */
        bzero(&sp, sizeof(struct scsi_parms));

        /* NVMe has very fast selection (always ready) */
        sp.sp_selection_timeout = 250;  /* 250 microseconds */

        /* NVMe doesn't have a traditional SCSI host ID, use 7 (common default) */
        sp.sp_scsi_host_id = 7;

        /* NVMe is not differential, it's PCIe */
        sp.sp_is_diff = 0;

        /* Report target 0 (our NVMe namespace as SCSI target) */
        for (targ = 0; targ < 16; targ++) {
            if (targ == 0) {
                /* Target 0 is our NVMe device */
                sp.sp_target_parms[targ].stp_is_present = 1;
                /* NVMe is always "synchronous" (no negotiation needed) */
                sp.sp_target_parms[targ].stp_is_sync = 1;
                /* NVMe data width - report as wide (64-bit PCIe) */
                sp.sp_target_parms[targ].stp_is_wide = 1;
                /* NVMe has no traditional sync period/offset */
                sp.sp_target_parms[targ].stp_sync_period = 0;
                sp.sp_target_parms[targ].stp_sync_offset = 0;
            } else {
                /* No other targets present */
                sp.sp_target_parms[targ].stp_is_present = 0;
            }
        }

        /* Copy to userspace using copyout */
        copyout(&sp, (void *)op->sb_addr, sizeof(struct scsi_parms));
        return 0;
    }
    default:
        cmn_err(CE_WARN, "nvme_scsi_ioctl: unknown ioctl 0x%x", cmd);
        return EINVAL;
    }
}

/*
 * nvme_aborted_fifo_add: Add an aborted command to the FIFO
 *
 * Stores key identifying fields from the scsi_request that remain constant
 * across retries (CDB, buffer, flags, etc.)
 */
void
nvme_aborted_fifo_add(nvme_soft_t *soft, scsi_request_t *req)
{
    nvme_aborted_cmd_t *entry;
    uint_t idx;

    mutex_lock(&soft->aborted_lock, PZERO);

    /* Get next write position (circular FIFO, overwrites oldest) */
    idx = soft->aborted_head;
    soft->aborted_head = (idx + 1) % NVME_ABORT_FIFO_SIZE;

    entry = &soft->aborted_cmds[idx];

    /* Copy key fields that identify the request */
    bcopy(req->sr_command, entry->cdb,
          req->sr_cmdlen < SCSI_MAX_CDB_LEN ? req->sr_cmdlen : SCSI_MAX_CDB_LEN);
    entry->sr_flags = req->sr_flags;
    entry->sr_buffer = req->sr_buffer;
    entry->sr_buflen = req->sr_buflen;
    entry->sr_bp = req->sr_bp;
    entry->abort_time = lbolt;

    /* Mark entry as valid in bitmap */
    soft->aborted_bitmap |= (1U << idx);

    mutex_unlock(&soft->aborted_lock);

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_aborted_fifo_add: stored aborted cmd at idx=%u "
            "buffer=%p buflen=%u bp=%p flags=0x%x",
            idx, entry->sr_buffer, entry->sr_buflen, entry->sr_bp, entry->sr_flags);
#endif
}

/*
 * nvme_aborted_fifo_find_and_remove: Check if request matches an aborted command
 *
 * Searches the FIFO for a matching entry based on CDB, buffer, buflen, bp, and flags.
 * If found, marks the entry as invalid and returns 1 (indicating this is a retry).
 * Otherwise returns 0 (not a retry).
 */
int
nvme_aborted_fifo_find_and_remove(nvme_soft_t *soft, nvme_rwcmd_state_t *ps)
{
    nvme_aborted_cmd_t *entry;
    uint_t i;
    int found = 0;

    mutex_lock(&soft->aborted_lock, PZERO);

    /* Early exit if no valid entries in bitmap */
    if (soft->aborted_bitmap == 0) {
        mutex_unlock(&soft->aborted_lock);
        return 0;
    }

    /* Search the entire FIFO */
    for (i = 0; i < NVME_ABORT_FIFO_SIZE; i++) {
        /* Skip invalid entries (check bitmap in one comparison) */
        if (!(soft->aborted_bitmap & (1U << i))) {
            continue;
        }

        entry = &soft->aborted_cmds[i];

        /* Match on: buffer address, buflen, bp, and flags */
        if (entry->sr_buffer == ps->req->sr_buffer &&
            entry->sr_buflen == ps->req->sr_buflen &&
            entry->sr_bp == ps->req->sr_bp &&
            entry->sr_flags == ps->req->sr_flags) {

            /* Also verify CDB matches (compare up to sr_cmdlen bytes) */
            if (bcmp(entry->cdb, ps->req->sr_command,
                     ps->req->sr_cmdlen < SCSI_MAX_CDB_LEN ? ps->req->sr_cmdlen : SCSI_MAX_CDB_LEN) == 0) {
                /* Found a match - this is a retry */
                /* Clear bit in bitmap to mark as invalid */
                soft->aborted_bitmap &= ~(1U << i);
                found = 1;
#ifdef NVME_DBG
                cmn_err(CE_NOTE, "nvme_aborted_fifo_find_and_remove: RETRY DETECTED at idx=%u "
                        "buffer=%p buflen=%u bp=%p flags=0x%x",
                        i, entry->sr_buffer, entry->sr_buflen, entry->sr_bp, entry->sr_flags);
#endif
                break;
            }
        }
    }

    mutex_unlock(&soft->aborted_lock);

    return found;
}
