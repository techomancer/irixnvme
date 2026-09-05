/*
 * nvme_cmd.c - NVMe Command Construction and Submission
 *
 * Functions for building and submitting NVMe commands to the controller.
 */

#include "nvmedrv.h"

/*
 * nvme_submit_cmd: Submit a command to a queue
 *
 * Returns:
 *   0 on success
 *   -1 if queue is full
 */
int
nvme_submit_cmd(nvme_soft_t *soft, nvme_queue_t *q, nvme_command_t *cmd)
{
    uint_t next_tail;
    nvme_command_t *sq_entry;

    mutex_lock(&q->lock, PZERO);

    /* Calculate next tail position */
    next_tail = (q->sq_tail + 1) & q->size_mask;

    /*
     * Full check.  We deliberately do NOT use the SQ Head value the
     * controller reports in completion entries: a single bogus SQHD (seen
     * on a MAXIO-based Patriot P300) left sq_head == sq_tail + 1 with
     * nothing in flight, the ring looked permanently full, and since no
     * command could be submitted no completion ever arrived to correct
     * it.  'outstanding' is maintained by the driver itself (incremented
     * here, decremented per completion) and is the true number of ring
     * slots in use.  A ring of N entries holds at most N-1 commands.
     */
    if ((uint_t)atomicAddInt((int *)&q->outstanding, 0) >= q->size - 1) {
        cmn_err(CE_WARN, "nvme_submit_cmd: queue %d is full (outstanding=%d, head=%u, tail=%u)",
                q->qid, q->outstanding, q->sq_head, q->sq_tail);
        mutex_unlock(&q->lock);
        return -1;
    }

    sq_entry = &q->sq[q->sq_tail];
#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_submit_cmd: Writing to SQ[%u] at %p", q->sq_tail, sq_entry);
#endif
    /* Write command to submission queue entry */
    NVME_MEMWR(&sq_entry->cdw0, cmd->cdw0);
    NVME_MEMWR(&sq_entry->nsid, cmd->nsid);
    NVME_MEMWR(&sq_entry->cdw2, cmd->cdw2);
    NVME_MEMWR(&sq_entry->cdw3, cmd->cdw3);
    NVME_MEMWR(&sq_entry->mptr_lo, cmd->mptr_lo);
    NVME_MEMWR(&sq_entry->mptr_hi, cmd->mptr_hi);
    NVME_MEMWR(&sq_entry->prp1_lo, cmd->prp1_lo);
    NVME_MEMWR(&sq_entry->prp1_hi, cmd->prp1_hi);
    NVME_MEMWR(&sq_entry->prp2_lo, cmd->prp2_lo);
    NVME_MEMWR(&sq_entry->prp2_hi, cmd->prp2_hi);
    NVME_MEMWR(&sq_entry->cdw10, cmd->cdw10);
    NVME_MEMWR(&sq_entry->cdw11, cmd->cdw11);
    NVME_MEMWR(&sq_entry->cdw12, cmd->cdw12);
    NVME_MEMWR(&sq_entry->cdw13, cmd->cdw13);
    NVME_MEMWR(&sq_entry->cdw14, cmd->cdw14);
    NVME_MEMWR(&sq_entry->cdw15, cmd->cdw15);
#ifdef IP30
    heart_dcache_wb_inval((caddr_t)sq_entry, sizeof(nvme_command_t));
#endif

#ifdef NVME_DBG_CMD
    /* Dump what we just wrote to the SQ */
    nvme_dump_sq_entry(sq_entry, "After writing to SQ");
#endif /* NVME_DBG_CMD */
    /* Advance tail */
    q->sq_tail = next_tail;

    /* Increment outstanding command counter */
    atomicAddInt(&q->outstanding, 1);

#ifdef NVME_DBG_EXTRA
    cmn_err(CE_NOTE, "nvme_submit_cmd: Ringing doorbell at offset 0x%x with value %u (outstanding=%d)",
            q->sq_doorbell, q->sq_tail, q->outstanding);
#endif
    /* Ring doorbell to notify controller */
    NVME_WR(soft, q->sq_doorbell, q->sq_tail);
    pciio_write_gather_flush(soft->pci_vhdl); // make sure these post on IP30

#ifdef NVME_DBG_EXTRA
    /* Verify the doorbell was written */
    cmn_err(CE_NOTE, "nvme_submit_cmd: Doorbell readback = 0x%08x",
            NVME_RD(soft, q->sq_doorbell));
#endif
    mutex_unlock(&q->lock);

#ifdef NVME_COMPLETION_INTERRUPT
    /* Start watchdog timer for I/O queue to catch missed interrupts */
    if (q->qid != 0) {  /* Only for I/O queue, not admin queue */
        nvme_watchdog_start(soft, q);
    }
#endif
#ifdef NVME_COMPLETION_THREAD
    /* If interrupts are disabled, wake polling thread to check for completions */
    if (!soft->interrupts_enabled) {
        nvme_kick_poll_thread(soft);
    }
#endif
#ifdef NVME_COMPLETION_MANUAL
    /* Wait a bit and check if completion arrived (in case interrupt isn't working) */
    

    /* Manually check for completions */
    {
        uint_t old_head = q->cq_head;
        int num_processed = 0;
        int np;
        while ((num_processed += nvme_process_completions(soft, q)) == 0) {
                us_delay(1000); /* 1 millisecond */
        }
        while ((np = nvme_process_completions(soft, q)) > 0) {
                num_processed += np;
                us_delay(1000); /* 1 millisecond */
        }

        cmn_err(CE_WARN, "nvme_submit_cmd: after 1ms delay, manually processed %d completions (cq_head %d->%d)  int count=%d",
                num_processed, old_head, q->cq_head, nvme_intcount);
    }
#endif

    return 0;
}

/*
 * nvme_admin_identify_controller: Send Identify Controller command
 *
 * Uses utility buffer to retrieve controller identification data.
 * Stores serial, model, firmware, and number of namespaces in soft state.
 */
int
nvme_admin_identify_controller(nvme_soft_t *soft)
{
    nvme_command_t cmd;
    nvme_identify_controller_t *id_ctrl;

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_identify_controller: sending command");
#endif
    /* Clear utility buffer */
    bzero(soft->utility_buffer, NBPP);
#ifdef IP30
    heart_dcache_wb_inval((caddr_t)soft->utility_buffer, sizeof(NBPP));
#else
    dki_dcache_wbinval((caddr_t)soft->utility_buffer, sizeof(NBPP));
#endif

    /* Build Identify Controller command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_ADMIN_IDENTIFY | (NVME_ADMIN_CID_IDENTIFY_CONTROLLER << 16);

    /* NSID: not used for controller identify */
    cmd.nsid = 0;

    /* PRP1: physical address of utility buffer (controller data destination) */
    cmd.prp1_lo = PHYS64_LO(soft->utility_buffer_phys);
    cmd.prp1_hi = PHYS64_HI(soft->utility_buffer_phys);

    /* PRP2: not needed (data is only 4KB) */
    cmd.prp2_lo = 0;
    cmd.prp2_hi = 0;

    /* CDW10: CNS = 0x01 for Identify Controller */
    cmd.cdw10 = NVME_CNS_CONTROLLER;

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_identify_controller: Built command:");
    cmn_err(CE_NOTE, "  cdw0=0x%08x (opcode=0x%02x, cid=0x%04x)",
            cmd.cdw0, cmd.cdw0 & 0xFF, (cmd.cdw0 >> 16) & 0xFFFF);
    cmn_err(CE_NOTE, "  nsid=0x%08x", cmd.nsid);
    cmn_err(CE_NOTE, "  prp1=0x%08x%08x (virt=%p, phys=0x%llx)",
            cmd.prp1_hi, cmd.prp1_lo,
            soft->utility_buffer, soft->utility_buffer_phys);
    cmn_err(CE_NOTE, "  cdw10=0x%08x (CNS)", cmd.cdw10);
#endif
    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_identify_controller: failed to submit command (queue full?)");
#endif
        return 0;  /* Failure */
    }
#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_identify_controller: command submitted, waiting for completion");
#endif
    return 1;  /* Success - command submitted */
}

/*
 * nvme_admin_identify_namespace: Send Identify Namespace command
 *
 * Uses utility buffer to retrieve namespace identification data for namespace 1.
 * Stores namespace size and block size in soft state for later use by SCSI emulation.
 */
int
nvme_admin_identify_namespace(nvme_soft_t *soft)
{
    nvme_command_t cmd;

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_identify_namespace: sending command for NSID 1");
#endif
    /* Clear utility buffer */
    bzero(soft->utility_buffer, NBPP);
#ifdef IP30
    heart_dcache_wb_inval((caddr_t)soft->utility_buffer, sizeof(NBPP));
#else
    dki_dcache_wbinval((caddr_t)soft->utility_buffer, sizeof(NBPP));
#endif

    /* Build Identify Namespace command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_ADMIN_IDENTIFY | (NVME_ADMIN_CID_IDENTIFY_NAMESPACE << 16);

    cmd.nsid = 1;

    /* PRP1: physical address of utility buffer (namespace data destination) */
    cmd.prp1_lo = PHYS64_LO(soft->utility_buffer_phys);
    cmd.prp1_hi = PHYS64_HI(soft->utility_buffer_phys);

    /* PRP2: not needed (data is only 4KB) */
    cmd.prp2_lo = 0;
    cmd.prp2_hi = 0;

    /* CDW10: CNS = 0x00 for Identify Namespace */
    cmd.cdw10 = NVME_CNS_NAMESPACE;

    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_identify_namespace: failed to submit command (queue full?)");
#endif
        return 0;  /* Failure */
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_admin_identify_namespace: command submitted, waiting for completion");
#endif
    return 1;  /* Success - command submitted */
}

/*
 * nvme_admin_get_log_page_error: Send Get Log Page command for Error Information
 *
 * Uses utility buffer to retrieve error log entries.
 * The completion handler will decode and dump the first entry.
 */
int
nvme_admin_get_log_page_error(nvme_soft_t *soft)
{
    nvme_command_t cmd;

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_get_log_page_error: sending command");
#endif
    /* Clear utility buffer */
    bzero(soft->utility_buffer, NBPP);
#ifdef IP30
    heart_dcache_wb_inval((caddr_t)soft->utility_buffer, sizeof(NBPP));
#else
    dki_dcache_wbinval((caddr_t)soft->utility_buffer, sizeof(NBPP));
#endif

    /* Build Get Log Page command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_ADMIN_GET_LOG_PAGE | (NVME_ADMIN_CID_GET_LOG_PAGE_ERROR << 16);

    /* NSID: 0xFFFFFFFF for global log pages */
    cmd.nsid = 0xFFFFFFFF;

    /* PRP1: physical address of utility buffer (log data destination) */
    cmd.prp1_lo = PHYS64_LO(soft->utility_buffer_phys);
    cmd.prp1_hi = PHYS64_HI(soft->utility_buffer_phys);

    /* PRP2: not needed (requesting only 4KB) */
    cmd.prp2_lo = 0;
    cmd.prp2_hi = 0;

    /* CDW10: Log Page Identifier (7:0) and NUMDL (31:16)
     * NUMDL = Number of Dwords Lower - 1 (0-based)
     * For 4KB = 1024 dwords, NUMDL = 1023 = 0x3FF
     */
    cmd.cdw10 = NVME_LOG_PAGE_ERROR_INFO | (0x3FF << 16);

    /* CDW11: NUMDU (15:0) = Number of Dwords Upper (upper 16 bits of dword count)
     * For 4KB, NUMDU = 0
     */
    cmd.cdw11 = 0;

    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_get_log_page_error: failed to submit command (queue full?)");
#endif
        return 0;  /* Failure */
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_admin_get_log_page_error: command submitted, waiting for completion");
#endif
    return 1;  /* Success - command submitted */
}

/*
 * nvme_admin_get_features: Send Get Features command
 *
 * Retrieves the specified feature from the controller.
 * The feature value is returned in DW0 of the completion queue entry.
 *
 * Arguments:
 *   soft - Controller soft state
 *   fid  - Feature Identifier (NVME_FEAT_*)
 *   sel  - Select value (NVME_FEAT_SEL_*)
 *
 * Returns: 1 on success (command submitted), 0 on failure
 */
int
nvme_admin_get_features(nvme_soft_t *soft, uchar_t fid, uchar_t sel)
{
    nvme_command_t cmd;

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_get_features: FID=0x%02x SEL=0x%02x", fid, sel);
#endif

    /* Build Get Features command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16)
     * Encode FID in CID so completion handler can identify which feature */
    cmd.cdw0 = NVME_ADMIN_GET_FEATURES | (NVME_ADMIN_CID_GET_FEATURES(fid) << 16);

    /* NSID: Some features are namespace-specific and require NSID=1
     * Per NVMe 1.0e spec:
     *   - Error Recovery (0x05): namespace-specific
     *   - LBA Range Type (0x03): namespace-specific
     *   - All others: controller-level (use NSID=0) */
    if (fid == NVME_FEAT_ERROR_RECOVERY || fid == NVME_FEAT_LBA_RANGE_TYPE) {
        cmd.nsid = 1;  /* Namespace 1 */
    } else {
        cmd.nsid = 0;  /* Controller-level feature */
    }

    /* CDW10: FID (7:0), SEL (9:8) */
    cmd.cdw10 = fid | ((uint_t)sel << 8);

    /* CDW11-15: Feature-specific, set to 0 for Get Features */
    cmd.cdw11 = 0;
    cmd.cdw12 = 0;
    cmd.cdw13 = 0;
    cmd.cdw14 = 0;
    cmd.cdw15 = 0;

    /* PRP: not needed for most features (data returned in completion DW0) */
    cmd.prp1_lo = 0;
    cmd.prp1_hi = 0;
    cmd.prp2_lo = 0;
    cmd.prp2_hi = 0;

    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_get_features: failed to submit command (queue full?)");
#endif
        return 0;  /* Failure */
    }

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_get_features: command submitted, waiting for completion");
#endif
    return 1;  /* Success - command submitted */
}

/*
 * nvme_admin_set_features: Send Set Features command
 *
 * Sets the specified feature on the controller.
 *
 * Arguments:
 *   soft  - Controller soft state
 *   fid   - Feature Identifier (NVME_FEAT_*)
 *   value - Feature value to set (32-bit)
 *
 * Returns: 1 on success (command submitted), 0 on failure
 */
int
nvme_admin_set_features(nvme_soft_t *soft, uchar_t fid, uint_t value)
{
    nvme_command_t cmd;

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_set_features: FID=0x%02x value=0x%08x", fid, value);
#endif

    /* Build Set Features command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_ADMIN_SET_FEATURES | (NVME_ADMIN_CID_SET_FEATURES << 16);

    /* NSID: not used for most features */
    cmd.nsid = 0;

    /* CDW10: FID (7:0) */
    cmd.cdw10 = fid;

    /* CDW11: Feature value (feature-dependent) */
    cmd.cdw11 = value;

    /* CDW12-15: Feature-specific, set to 0 */
    cmd.cdw12 = 0;
    cmd.cdw13 = 0;
    cmd.cdw14 = 0;
    cmd.cdw15 = 0;

    /* PRP: not needed for most features */
    cmd.prp1_lo = 0;
    cmd.prp1_hi = 0;
    cmd.prp2_lo = 0;
    cmd.prp2_hi = 0;

    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_set_features: failed to submit command (queue full?)");
#endif
        return 0;  /* Failure */
    }

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_admin_set_features: command submitted, waiting for completion");
#endif
    return 1;  /* Success - command submitted */
}

/*
 * nvme_admin_query_features: Query common controller features
 *
 * Queries a set of common features and stores them in the soft state.
 * This should be called during controller initialization after Identify.
 *
 * Features queried:
 *   - Arbitration
 *   - Power Management
 *   - Temperature Threshold
 *   - Error Recovery
 *   - Volatile Write Cache
 *   - Number of Queues
 *   - Interrupt Coalescing
 *   - Write Atomicity
 *   - Async Event Config
 *
 * Returns: 1 on success, 0 on failure
 */
int
nvme_admin_query_features(nvme_soft_t *soft)
{
    /* List of features to query (FID only - MIPS Pro C can't handle non-const struct init) */
    static const uchar_t feature_ids[] = {
        NVME_FEAT_ARBITRATION,
        NVME_FEAT_POWER_MANAGEMENT,
        NVME_FEAT_TEMPERATURE_THRESHOLD,
        NVME_FEAT_ERROR_RECOVERY,
        NVME_FEAT_VOLATILE_WRITE_CACHE,
        NVME_FEAT_NUMBER_OF_QUEUES,
        NVME_FEAT_INTERRUPT_COALESCING,
        NVME_FEAT_WRITE_ATOMICITY,
        NVME_FEAT_ASYNC_EVENT_CONFIG
    };
    static const char *feature_names[] = {
        "Arbitration",
        "Power Management",
        "Temperature Threshold",
        "Error Recovery",
        "Volatile Write Cache",
        "Number of Queues",
        "Interrupt Coalescing",
        "Write Atomicity",
        "Async Event Config"
    };
    int num_features = sizeof(feature_ids) / sizeof(feature_ids[0]);
    int i;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_admin_query_features: querying %d features", num_features);
#endif

    /* Query each feature in sequence */
    for (i = 0; i < num_features; i++) {
        /* Submit Get Features command with SEL_SUPPORTED to discover capabilities */
        if (!nvme_admin_get_features(soft, feature_ids[i], NVME_FEAT_SEL_SUPPORTED)) {
            cmn_err(CE_WARN, "nvme_admin_query_features: failed to submit Get Features for %s (FID 0x%02x)",
                    feature_names[i], feature_ids[i]);
            return 0;
        }

#ifndef NVME_COMPLETION_MANUAL
        nvme_wait_for_queue_idle(soft, &soft->admin_queue, 5000);
#endif

        /* Feature value is stored in soft state by the completion handler
         * based on the FID encoded in the CID */

#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_admin_query_features: %s (FID 0x%02x) queried",
                feature_names[i], feature_ids[i]);
#endif
    }

    cmn_err(CE_NOTE, "nvme: Queried %d controller features", num_features);
    return 1;
}

/*
 * nvme_admin_create_cq: Create I/O Completion Queue
 */
int
nvme_admin_create_cq(nvme_soft_t *soft, ushort_t qid, ushort_t qsize,
                     alenaddr_t phys_addr, ushort_t vector)
{
    nvme_command_t cmd;

    bzero(&cmd, sizeof(cmd));

    cmd.cdw0 = NVME_ADMIN_CREATE_CQ;
    cmd.cdw0 |= NVME_ADMIN_CID_CREATE_CQ << 16;

    cmd.prp1_lo = PHYS64_LO(phys_addr);
    cmd.prp1_hi = PHYS64_HI(phys_addr);
    cmd.cdw10 = ((qsize - 1) << 16) | qid;
    cmd.cdw11 = NVME_QUEUE_PHYS_CONTIG 
#ifdef NVME_COMPLETION_INTERRUPT
              | NVME_QUEUE_IRQ_ENABLED | (vector << 16)
#endif
            ;

    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_create_cq: failed to submit command");
#endif
        return 0;
    }
    return 1;
}

/*
 * nvme_admin_create_sq: Create I/O Submission Queue
 */
int
nvme_admin_create_sq(nvme_soft_t *soft, ushort_t qid, ushort_t qsize,
                     alenaddr_t phys_addr, ushort_t cqid)
{
    nvme_command_t cmd;

    bzero(&cmd, sizeof(cmd));

    cmd.cdw0 = NVME_ADMIN_CREATE_SQ;
    cmd.cdw0 |= NVME_ADMIN_CID_CREATE_SQ << 16;

    cmd.prp1_lo = PHYS64_LO(phys_addr);
    cmd.prp1_hi = PHYS64_HI(phys_addr);
    cmd.cdw10 = ((qsize - 1) << 16) | qid;
    cmd.cdw11 = NVME_QUEUE_PHYS_CONTIG | (cqid << 16);

    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_create_sq: failed to submit command");
#endif
        return 0;
    }
    return 1;
}

/*
 * nvme_admin_delete_sq: Delete I/O Submission Queue
 */
int
nvme_admin_delete_sq(nvme_soft_t *soft, ushort_t qid)
{
    nvme_command_t cmd;

    bzero(&cmd, sizeof(cmd));

    cmd.cdw0 = NVME_ADMIN_DELETE_SQ;
    cmd.cdw0 |= NVME_ADMIN_CID_DELETE_SQ << 16;
    cmd.cdw10 = qid;

    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_delete_sq: failed to submit command");
#endif
        return 0;
    }
    return 1;
}

/*
 * nvme_admin_delete_cq: Delete I/O Completion Queue
 */
int
nvme_admin_delete_cq(nvme_soft_t *soft, ushort_t qid)
{
    nvme_command_t cmd;

    bzero(&cmd, sizeof(cmd));

    cmd.cdw0 = NVME_ADMIN_DELETE_CQ;
    cmd.cdw0 |= NVME_ADMIN_CID_DELETE_CQ << 16;
    cmd.cdw10 = qid;

    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_admin_delete_cq: failed to submit command");
#endif
        return 0;
    }
    return 1;
}

/*
 * nvme_admin_abort_command: Abort a command
 *
 * Issues an Abort command to the admin queue to abort a specific command.
 * The CID encoding uses bit 15 set to indicate abort, with the lower 8 bits
 * containing the CID being aborted.
 *
 * Per NVMe spec, CDW10 contains:
 *   Bits 31:16: Command ID of command to abort
 *   Bits 15:0:  Submission Queue ID where command was submitted (always 1 for I/O queue)
 *
 * Arguments:
 *   soft - Controller state
 *   cid  - Command ID to abort (from I/O queue, 0-255)
 *
 * Returns:
 *   1 on success (abort command submitted)
 *   0 on failure (could not submit)
 */
int
nvme_admin_abort_command(nvme_soft_t *soft, ushort_t cid)
{
    nvme_command_t cmd;
    ushort_t abort_cid;

    bzero(&cmd, sizeof(cmd));

    /* Encode CID for abort: bit 15 set, lower 8 bits = original CID */
    abort_cid = NVME_ADMIN_CID_MAKE_ABORT(cid);

    /* Build Abort command */
    cmd.cdw0 = NVME_ADMIN_ABORT;
    cmd.cdw0 |= abort_cid << 16;

    /* CDW10: SQID (15:0) = 1 (I/O queue), CID (31:16) = command to abort */
    cmd.cdw10 = 1 | (cid << 16);

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_admin_abort_command: aborting CID %d (abort_cid=0x%x)",
            cid, abort_cid);
#endif

    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
        cmn_err(CE_WARN, "nvme_admin_abort_command: failed to submit abort for CID %d", cid);
        return 0;
    }

    return 1;
}

/*
 * nvme_io_build_rw_command: Build NVMe Read/Write command from SCSI request
 *
 * Translates SCSI READ/WRITE commands (READ6, READ10, READ16, WRITE6, WRITE10, WRITE16)
 * into NVMe Read/Write commands. Parses the SCSI CDB and fills in the NVMe command
 * structure with opcode, namespace ID, LBA, and block count.
 *
 * For multi-command transfers (cmd_index > 0), the LBA and block count are adjusted
 * based on the command index and max_transfer_blocks.
 *
 * PRP entries are NOT set by this function - they must be filled in separately
 * by calling nvme_build_prps_from_alenlist().
 *
 * Arguments:
 *   soft      - Controller state
 *
 * Returns:
 *   1 on success
 *   0 on failure (unsupported CDB opcode, caller set sense data)
 *  -1 on failure (we set sense data)
 */
int
nvme_io_build_rw_command(nvme_soft_t *soft, nvme_rwcmd_state_t *ps)
{
    nvme_command_t *cmd = &(ps->cmd);

    /* Adjust LBA and block count based on command index for multi-command transfers */
    __uint64_t lba = ps->lba + ps->cidx * ps->max_transfer_blocks;
    uint_t num_blocks = ps->num_blocks - ps->cidx * ps->max_transfer_blocks;

    /* Clear command structure */
    bzero(cmd, sizeof(*cmd));

    if (num_blocks > ps->max_transfer_blocks) {
        num_blocks = ps->max_transfer_blocks;
    }

    /* Build NVMe command header - CID will be set by caller */
    if (ps->flags & NF_WRITE) {
        cmd->cdw0 = NVME_CMD_WRITE;
    } else {
        cmd->cdw0 = NVME_CMD_READ;
    }

    cmd->cdw0 |= (ps->cids[ps->cidx] << 16);

    /* Set namespace ID (hardcoded to 1) */
    cmd->nsid = 1;

    /* Set LBA (CDW10 = lower 32 bits, CDW11 = upper 32 bits) */
    cmd->cdw10 = (__uint32_t)(lba & 0xFFFFFFFF);
    cmd->cdw11 = (__uint32_t)(lba >> 32);

    /* Set number of logical blocks (0-based, so subtract 1) */
    cmd->cdw12 = (num_blocks > 0) ? (num_blocks - 1) : 0;

    /* Force Unit Access: instruct the controller to commit this write to
     * non-volatile NAND storage before returning the completion entry.
     * Controlled at runtime via NVME_IOC_SET_FUA; also enabled at boot
     * when NVME_FUA_WRITES is compiled in.  Applied to writes only. */
    if ((ps->flags & NF_WRITE) && soft->fua_enabled) {
        cmd->cdw12 |= NVME_CDW12_FUA;
    }

    /* Remaining fields already zeroed by bzero() above */

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_io_build_rw_command: %s cidx=%u LBA=%llu blocks=%u",
            (ps->flags & NF_WRITE) ? "WRITE" : "READ", ps->cidx, lba, num_blocks);
#endif
    return 1;
}

/*
 * nvme_get_translated_addr: Get and translate next page from alenlist
 *
 * This helper combines alenlist_get with pciio_dmatrans_addr to fetch
 * the next page from an alenlist and translate it to a PCI bus address.
 *
 * Arguments:
 *   soft         - Controller state (for pci_vhdl)
 *   alenlist     - Alenlist to fetch from (cursor must be initialized)
 *   maxlength    - Maximum bytes to fetch (typically nvme_page_size)
 *   out_address  - Output: Translated PCI bus address (physical address)
 *   out_length   - Output: Length of this segment in bytes
 *   flags        - whether the operatio is read or write
 *
 * Returns:
 *   0 on success
 *   -1 on failure (alenlist exhausted or DMA translation failed)
 */
int
nvme_get_translated_addr(nvme_soft_t *soft, alenlist_t alenlist, size_t maxlength,
                        alenaddr_t *out_address, size_t *out_length, int flags)
{
    alenaddr_t address;
    size_t length;
    size_t page_size = soft->nvme_page_size;

    size_t cursor_pos;
    size_t offset;

    /* Clamp to nvme_page_size — callers may pass chunk_size which can be larger */
    if (maxlength > page_size)
        maxlength = page_size;

    /*
     * Every PRP entry except PRP1 must be NVMe-page aligned, and PRP1 may
     * carry an offset but must not cross the end of its page.  The alenlist
     * does not know about the NVMe page size (it can be smaller than NBPP
     * on IP30/IP35, and compacted lists can merge physically contiguous
     * pages), so a naive alenlist_get(page_size) can hand us a segment that
     * straddles an NVMe page boundary.
     *
     * The old code only corrected this for the very first segment of a
     * request; for a multi-command request whose buffer is not page
     * aligned, command 2..N started with a straddling PRP1 and the drive
     * silently transferred the wrong bytes.
     *
     * Fix: fetch, and if the segment straddles a page, rewind the cursor to
     * where we were and fetch again with the exact remaining-in-page length.
     */
    cursor_pos = alenlist_cursor_offset(alenlist, NULL);

    if (alenlist_get(alenlist, NULL, maxlength, &address, &length, 0) != ALENLIST_SUCCESS) {
        return -1;
    }

    offset = (size_t)(address & (page_size - 1));
    if (offset != 0 && (offset + length) > page_size) {
        if (alenlist_cursor_init(alenlist, cursor_pos, NULL) != ALENLIST_SUCCESS) {
            cmn_err(CE_WARN, "nvme_get_translated_addr: cursor rewind to %u failed",
                    (uint_t)cursor_pos);
            return -1;
        }
        if (alenlist_get(alenlist, NULL, page_size - offset, &address, &length, 0)
                != ALENLIST_SUCCESS) {
            return -1;
        }
    }

    /* Translate to PCI bus address with explicit cast to quiet warnings */
    address = pciio_dmatrans_addr(soft->pci_vhdl, NULL, (paddr_t)address, length,
                                  PCIIO_DMA_DATA | DMATRANS64 | PCIIO_BYTE_STREAM
#if defined(IP30) || defined(IP35)
                                  | ((flags & NF_WRITE) ? PCIIO_PREFETCH : PCIBR_BARRIER)
#endif
                                );

    if (!address) {
        return -1;
    }

    *out_address = address;
    *out_length = length;
    return 0;
}

/*
 * nvme_prepare_alenlist: Prepare alenlist from SCSI request for PRP building
 *
 * This function handles the complexity of converting a SCSI request with various
 * data buffer formats into an alenlist that can be walked to build PRPs.
 *
 * Handles multiple data buffer modes:
 * - SRF_ALENLIST: User virtual address with alenlist in bp->b_private
 * - SRF_MAPBP: Buffer pointer that needs conversion via buf_to_alenlist
 * - SRF_MAP/SRF_MAPUSER: Kernel virtual address (with alignment checks for USERMAP)
 *
 * Arguments:
 *   soft            - Controller state
 *   ps              - rw command builder state
 *
 * Returns:
 *   1 on success
 *   0 or -1 on failure (logs error via cmn_err)
 */
int
nvme_prepare_alenlist(nvme_soft_t *soft, nvme_rwcmd_state_t *ps)
{
    scsi_request_t *req = ps->req;
    ps->alenlist = NULL;
    ps->alenlist_type = NVME_ALENLIST_SUPPLIED;

    if (req->sr_buffer == NULL && !(req->sr_flags & (SRF_ALENLIST | SRF_MAPBP))) {
        cmn_err(CE_WARN, "nvme_prepare_alenlist: NULL buffer with flags:0x%x buflen:%u buffer:%p", req->sr_flags, req->sr_buflen, req->sr_buffer);
        return 0;
    }

    /*
     * Determine data buffer source and build/extract alenlist
     * Use pre-allocated alenlist for MAPBP/MAP, use provided one for ALENLIST
     */
    if (req->sr_flags & SRF_ALENLIST) {
        /*
         * User virtual address case - alenlist already created by upper layer
         * and stored in the buffer's b_private field. Use it directly.
         */
        if (!IS_KUSEG(req->sr_buffer)) {
            cmn_err(CE_WARN, "nvme_prepare_alenlist: SRF_ALENLIST but address not KUSEG");
            return 0;
        }

        ps->alenlist = (alenlist_t)(((buf_t *)(req->sr_bp))->b_private);
        if (!ps->alenlist) {
            cmn_err(CE_WARN, "nvme_prepare_alenlist: SRF_ALENLIST but no alenlist in bp->b_private");
            return 0;
        }

#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_prepare_alenlist: using user alenlist from bp->b_private");
#endif
        ps->alenlist_type = NVME_ALENLIST_SUPPLIED;
    } else {
        /*
         * For MAPBP/MAP/MAPUSER: Choose allocation strategy based on request size
         * Small requests: Use dynamic alenlist (no lock contention)
         * Large requests: Use shared pre-allocated alenlist (needs lock)
         */
  
        if (req->sr_buflen < NVME_ALENLIST_SMALL_PAGES * NBPP) {
            /* Small request - allocate dedicated alenlist to avoid lock contention */
            ps->alenlist = alenlist_create(0);
            if (ps->alenlist) {
                ps->alenlist_type = NVME_ALENLIST_DYNAMIC;
            } else {
                /* Allocation failed - fall back to shared alenlist */
                mutex_lock(&soft->alenlist_lock, PZERO);
                ps->alenlist = soft->alenlist;
                ps->alenlist_type = NVME_ALENLIST_SHARED;
            }
        } else {
            /* Large request - use shared pre-allocated alenlist with lock */
            mutex_lock(&soft->alenlist_lock, PZERO);
            ps->alenlist = soft->alenlist;
            ps->alenlist_type = NVME_ALENLIST_SHARED;
        }

        if (req->sr_flags & SRF_MAPBP) {
            /*
             * Buffer-based mapping case - convert buf_t to alenlist
             */
            if (BP_ISMAPPED(((buf_t *)(req->sr_bp)))) {
                cmn_err(CE_WARN, "nvme_prepare_alenlist: SRF_MAPBP but buffer is already mapped");
                nvme_cleanup_alenlist(soft, ps);
                return 0;
            }

            /*
             * Cache flush for buf_t - always use bp_dcache_wbinval for buf_t
             * This handles both DMA read and write cases properly
             * If upper layer tells us to flush we flush, use war version first 
             * We have to use normal version afterwards because war version reads back!
             */
            if (req->sr_flags & SRF_FLUSH) {
#ifdef HEART_INVALIDATE_WAR
                if (req->sr_flags & SRF_DIR_IN) {
                    /* is this one appropriate for writes? it does not seem so becauuse it doesnt do writeback!. so lets call it only for reads. */
                    bp_heart_invalidate_war((buf_t *)(req->sr_bp));
                } else
#endif
                bp_dcache_wbinval((buf_t *)(req->sr_bp));
            }

            /* Convert buf_t to alenlist (buf_to_alenlist clears the alenlist first) */
            if (buf_to_alenlist(ps->alenlist, (buf_t *)(req->sr_bp), AL_NOCOMPACT) == NULL) {
                cmn_err(CE_WARN, "nvme_prepare_alenlist: buf_to_alenlist failed");
                nvme_cleanup_alenlist(soft, ps);
                return 0;
            }

#ifdef NVME_DBG
            cmn_err(CE_NOTE, "nvme_prepare_alenlist: converted buf_t to alenlist (SRF_MAPBP)");
#endif
        } else if (req->sr_flags & (SRF_MAP | SRF_MAPUSER)) {
            /*
             * Virtual address case - convert to alenlist
             * Use IS_KUSEG() to determine actual address type
             */
            int is_user_addr = IS_KUSEG(req->sr_buffer);
#ifdef NVME_DBG
            cmn_err(CE_NOTE, "nvme_prepare_alenlist: MAP flags:0x%02X is_user:%d", req->sr_flags, is_user_addr);
#endif
            /* Verify dword (4-byte) alignment - required for DMA */
            if (((__psunsigned_t)req->sr_buffer & 0x3) != 0) {
#ifdef NVME_DBG
                cmn_err(CE_WARN, "nvme_prepare_alenlist: buffer not dword-aligned (addr=0x%lx)",
                        (__psunsigned_t)req->sr_buffer);
#endif
                nvme_cleanup_alenlist(soft, ps);
                nvme_set_adapter_status(req, SC_ALIGN, ST_GOOD);
                return -1;
            }
            if ((req->sr_buflen & 0x3) != 0) {
#ifdef NVME_DBG
                cmn_err(CE_WARN, "nvme_prepare_alenlist: length not dword-aligned (len=%u)",
                        req->sr_buflen);
#endif
                nvme_cleanup_alenlist(soft, ps);
                nvme_set_adapter_status(req, SC_ALIGN, ST_GOOD);
                return -1;
            }

            /*
             * Cache flush - direction determines the cache operation
             * also in case workaround is needed use the workaround version firs
             * but follow with normal version because workaround rereads the cache lines
             */
            if (req->sr_flags & SRF_FLUSH) {
                if (req->sr_flags & SRF_DIR_IN) {
#ifdef HEART_INVALIDATE_WAR
                    heart_invalidate_war(req->sr_buffer, req->sr_buflen);
#endif
                    dki_dcache_inval(req->sr_buffer, req->sr_buflen);
                } else {
                    dki_dcache_wbinval(req->sr_buffer, req->sr_buflen);
                }
            }

            /* Convert to alenlist based on address type */
            if (is_user_addr) {
                /* User virtual address - use uvaddr_to_alenlist */
                if (uvaddr_to_alenlist(ps->alenlist, (uvaddr_t)req->sr_buffer,
                                       req->sr_buflen, 0) == NULL) {
                    cmn_err(CE_WARN, "nvme_prepare_alenlist: uvaddr_to_alenlist failed");
                    nvme_cleanup_alenlist(soft, ps);
                    return 0;
                }
#ifdef NVME_DBG
                cmn_err(CE_NOTE, "nvme_prepare_alenlist: converted uvaddr to alenlist (KUSEG)");
#endif
            } else {
                /* Kernel virtual address - use kvaddr_to_alenlist */
                if (kvaddr_to_alenlist(ps->alenlist, (caddr_t)req->sr_buffer,
                                       req->sr_buflen, AL_NOCOMPACT) == NULL) {
                    cmn_err(CE_WARN, "nvme_prepare_alenlist: kvaddr_to_alenlist failed");
                    nvme_cleanup_alenlist(soft, ps);
                    return 0;
                }
#ifdef NVME_DBG
                cmn_err(CE_NOTE, "nvme_prepare_alenlist: converted kvaddr to alenlist (!KUSEG)");
#endif
            }
        } else {
            /*
             * Unknown or unsupported buffer mode
             * One of SRF_ALENLIST, SRF_MAPBP, SRF_MAP, or SRF_MAPUSER must be set
             */
            cmn_err(CE_WARN, "nvme_prepare_alenlist: no valid buffer mapping flag set (sr_flags=0x%x)",
                    req->sr_flags);
            nvme_cleanup_alenlist(soft, ps);
            return 0;
        }
    }

    /* Initialize alenlist cursor at offset 0 (cursor will track offset as we walk) */
    if (ps->alenlist != NULL) {
        alenlist_cursor_init(ps->alenlist, 0, NULL);
    }

    return 1;
}

/*
 * nvme_cleanup_alenlist: Cleanup alenlist based on allocation type
 *
 * Handles cleanup for different alenlist types:
 * - SUPPLIED: No cleanup needed (owned by caller)
 * - SHARED: Unlock the shared alenlist mutex
 * - DYNAMIC: Destroy the dynamically allocated alenlist
 */
void
nvme_cleanup_alenlist(nvme_soft_t *soft, nvme_rwcmd_state_t *ps)
{
    if (ps->alenlist_type == NVME_ALENLIST_SHARED) {
        mutex_unlock(&soft->alenlist_lock);
    } else if (ps->alenlist_type == NVME_ALENLIST_DYNAMIC) {
        alenlist_destroy(ps->alenlist);
    }
    /* NVME_ALENLIST_SUPPLIED requires no cleanup */
}

/*
 * nvme_build_prps_from_alenlist: Build PRP entries from prepared alenlist
 *
 * Walks an alenlist and builds PRP entries for an NVMe command. The alenlist cursor
 * maintains the offset automatically, so calling this multiple times for different
 * cmd_index values will walk through consecutive chunks of the buffer.
 *
 * PRP construction:
 * - Single page: PRP1 only
 * - Dual page: PRP1 + PRP2 as direct addresses
 * - Multi-page: PRP1 + PRP2 pointing to PRP list
 *
 * Arguments:
 *   soft      - Controller state
 *   ps        - rw command builder state
 *
 * Returns:
 *   1 on success
 *   0 on hard error (alenlist translation failure - caller should set error)
 *  -1 on resource exhaustion (BUSY status set internally, caller should retry)
 */
int
nvme_build_prps_from_alenlist(nvme_soft_t *soft, nvme_rwcmd_state_t *ps)
{
    scsi_request_t *req = ps->req;
    nvme_command_t *cmd = &(ps->cmd);
    alenaddr_t address;
    size_t length;
    uint_t chunk_size;

    /* If no data transfer, we're done */
    if (!ps->alenlist) {
        cmn_err(CE_WARN, "nvme_build_prps_from_alenlist: NULL alenlist sr_flags:0x%x cidx:%u buflen:%u savedbl:%u", req->sr_flags, ps->cidx, req->sr_buflen, ps->alenlist, ps->buflen);
        return 0;  /* Success - no PRPs needed */
    }
    if (!req->sr_buflen) {
        cmn_err(CE_WARN, "nvme_build_prps_from_alenlist: 0 length sr_flags:0x%x cidx:%u alenlist:%p savedbl:%u", req->sr_flags, ps->cidx, ps->alenlist, ps->buflen);
        return 0;  /* Success - no PRPs needed */
    }

    /* Calculate chunk size for this command (from the block count actually
     * being transferred, which may be less than sr_buflen) */
    chunk_size = (ps->num_blocks - ps->cidx * ps->max_transfer_blocks) * soft->block_size;
    if (chunk_size > ps->max_transfer_blocks * soft->block_size) {
        chunk_size = ps->max_transfer_blocks * soft->block_size;
    }

#ifdef NVME_DBG_CMD
    cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: cidx=%u chunk_size=%u buflen=%u",
            ps->cidx, chunk_size, req->sr_buflen);
#endif

    /* Get and translate the first page (possibly partial) for PRP1 */
    if (nvme_get_translated_addr(soft, ps->alenlist, chunk_size, &address, &length, ps->flags) != 0) {
        cmn_err(CE_WARN, "nvme_build_prps_from_alenlist: failed to get/translate first page");
        return 0;  /* Hard error - DMA translation failed */
    }

    /* Set PRP1 to first address */
    cmd->prp1_lo = PHYS64_LO(address);
    cmd->prp1_hi = PHYS64_HI(address);

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: PRP1=0x%llx len=%u", address, length);
#endif

    /* Subtract what we fetched */
    chunk_size -= length;

    /* Determine if we need PRP2 or a PRP list */
    if (chunk_size == 0) {
        /*
         * CASE 1: Single page transfer - PRP1 only
         */
#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: single page (PRP1 only)");
#endif
    } else if (chunk_size <= soft->nvme_page_size) {
        /*
         * CASE 2: Exactly 2 pages - use PRP2 directly (no PRP list needed)
         */
        if (nvme_get_translated_addr(soft, ps->alenlist, chunk_size, &address, &length, ps->flags) != 0) {
            cmn_err(CE_WARN, "nvme_build_prps_from_alenlist: failed to get/translate second page");
            return 0;  /* Hard error - DMA translation failed */
        }

        cmd->prp2_lo = PHYS64_LO(address);
        cmd->prp2_hi = PHYS64_HI(address);

#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: dual page (PRP2=0x%llx len=%u)", address, length);
#endif
    } else {
        /*
         * CASE 3: Multi-page transfer - need PRP list(s)
         */
        int num_prp_pages = 0;
        int pool_index;
        void *prp_virt = NULL;
        alenaddr_t prp_phys = 0;
        __uint32_t *prp_list_dwords = NULL;
        uint_t prp_index = soft->nvme_prp_entries-1;  /* Start at max to trigger allocation */
        nvme_cmd_info_t *ci = &soft->io_requests[ps->cids[ps->cidx]];

        /* Walk the alenlist page by page to fill PRP list */
        while (chunk_size > 0) {
            if (nvme_get_translated_addr(soft, ps->alenlist, chunk_size, &address, &length, ps->flags) != 0) {
                cmn_err(CE_WARN, "nvme_build_prps_from_alenlist: failed to get/translate page (remaining=%u)",
                        chunk_size);
                return 0;  /* Hard error - DMA translation failed */
            }

#ifdef NVME_DBG_EXTRA
            cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: processing page addr=0x%llx len=%u", address, length);
#endif
            /* Check if we need a new PRP list page */
            if (prp_index >= soft->nvme_prp_entries - 1) {
                /*
                 * PRP list pages were reserved for this CID in
                 * nvme_io_reserve() before we got here, so this can only
                 * fail if the reservation estimate was wrong (a bug).
                 */
                if (ci->nprp_used >= ci->nprp) {
                    cmn_err(CE_WARN, "nvme_build_prps_from_alenlist: CID %u needs more than the %d "
                            "reserved PRP pages (remaining=%u) - this is a driver bug",
                            ps->cids[ps->cidx], ci->nprp, chunk_size);
                    return 0;
                }
                pool_index = ci->prpidx[ci->nprp_used++];

                /* Calculate addresses for PRP list page */
                prp_virt = (void *)((caddr_t)soft->prp_pool + (pool_index * soft->nvme_page_size));
                prp_phys = soft->prp_pool_phys + (pool_index * soft->nvme_page_size);

#ifdef NVME_DBG
                cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: allocated PRP page %d: pool_index=%d, virt=%p, phys=0x%llx",
                        num_prp_pages, pool_index, prp_virt, prp_phys);
#endif
                if (num_prp_pages == 0) {
                    /* First PRP list page - set PRP2 to point to it */
                    cmd->prp2_lo = PHYS64_LO(prp_phys);
                    cmd->prp2_hi = PHYS64_HI(prp_phys);
                } else {
                    /* Subsequent page - chain from previous page's last entry */
                    NVME_MEMWR(&prp_list_dwords[(soft->nvme_prp_entries - 1) * 2], PHYS64_LO(prp_phys));
                    NVME_MEMWR(&prp_list_dwords[(soft->nvme_prp_entries - 1) * 2 + 1], PHYS64_HI(prp_phys));
#ifdef IP30
                    heart_dcache_wb_inval(prp_list_dwords, soft->nvme_prp_entries << 3);
#endif                    
#ifdef NVME_DBG
                    cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: chained page %d -> page %d (phys=0x%llx)",
                            num_prp_pages - 1, num_prp_pages, prp_phys);
#endif
                }

                /* Move to newly allocated page */
                num_prp_pages++;
                prp_index = 0;
                prp_list_dwords = (__uint32_t *)prp_virt;
            }

            NVME_MEMWR(&prp_list_dwords[prp_index * 2], PHYS64_LO(address));
            NVME_MEMWR(&prp_list_dwords[prp_index * 2 + 1], PHYS64_HI(address));

            prp_index++;
            chunk_size -= length;
        }
#ifdef IP30
        heart_dcache_wb_inval(prp_list_dwords, prp_index << 3);
#endif                    

#ifdef NVME_DBG
        cmn_err(CE_NOTE, "nvme_build_prps_from_alenlist: multi-page complete (%d PRP pages, %d entries in last page)",
                num_prp_pages, prp_index);
#endif
    }

    return 1;  /* Success */
}

/*
 * nvme_prp_pool_init: Initialize the PRP list pool
 *
 * Allocates a pool of pages for PRP lists (32 pages = 128KB).
 * Each page can hold up to 512 PRP entries (4096 bytes / 8 bytes per entry).
 *
 * Returns:
 *   0 on success
 *   -1 on failure
 */
int
nvme_prp_pool_init(nvme_soft_t *soft)
{
    int pages;

    /* Allocate pool memory (32 pages = 128KB) */
    pages = (int)btop(NVME_PRP_POOL_SIZE * soft->nvme_page_size);
#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_prp_pool_init: allocating PRP pool (%d pages, %d bytes)",
            pages, pages * NBPP);
#endif
    soft->prp_pool = kvpalloc(pages,
                              VM_UNCACHED | VM_PHYSCONTIG | VM_DIRECT | VM_NOSLEEP,
                              0);
    if (!soft->prp_pool) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_prp_pool_init: failed to allocate PRP pool");
#endif
        return -1;
    }

    /* Clear the pool memory */
    bzero(soft->prp_pool, pages * NBPP);

    /* Get DMA-translated physical address for the pool */
    soft->prp_pool_phys = pciio_dmatrans_addr(soft->pci_vhdl, 0,
                                              kvtophys(soft->prp_pool),
                                              pages * NBPP,
                                              PCIIO_DMA_CMD | DMATRANS64 | QUEUE_SWAP);
    if (!soft->prp_pool_phys) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_prp_pool_init: DMA translation failed");
#endif
        kvpfree(soft->prp_pool, pages);
        soft->prp_pool = NULL;
        return -1;
    }

    /* Initialize bitmap - all pages available (all bits set to 1) */
    {
        int w;
        for (w = 0; w < NVME_PRP_POOL_WORDS; w++)
            soft->prp_pool_bitmap[w] = 0xFFFFFFFFu;
    }
    soft->prp_pool_free_count = NVME_PRP_POOL_SIZE;

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_prp_pool_init: PRP pool allocated at virt=%p phys=0x%llx",
            soft->prp_pool, soft->prp_pool_phys);
#endif
    return 0;
}

/*
 * nvme_prp_pool_done: Free the PRP list pool
 *
 * Releases all resources allocated by nvme_prp_pool_init().
 * Should be called during driver shutdown.
 */
void
nvme_prp_pool_done(nvme_soft_t *soft)
{
    if (!soft->prp_pool) {
        return;  /* Pool was never initialized */
    }

#ifdef NVME_DBG
    cmn_err(CE_NOTE, "nvme_prp_pool_done: freeing PRP pool");
#endif
    /* Free the pool memory */
    kvpfree(soft->prp_pool, (int)btop(NVME_PRP_POOL_SIZE * soft->nvme_page_size));
    soft->prp_pool = NULL;
    soft->prp_pool_phys = 0;
    soft->prp_pool_free_count = 0;
}

/*
 * nvme_prp_pool_alloc_locked: Allocate a PRP list page from the pool
 *
 * Caller must hold soft->io_requests_lock.
 *
 * Returns:
 *   0..NVME_PRP_POOL_SIZE-1: Index of allocated page
 *   -1: No pages available
 */
int
nvme_prp_pool_alloc_locked(nvme_soft_t *soft)
{
    int w, b;
    __uint32_t word;

    for (w = 0; w < NVME_PRP_POOL_WORDS; w++) {
        word = soft->prp_pool_bitmap[w];
        if (word == 0)
            continue;
        for (b = 0; b < 32; b++) {
            if (word & (1u << b)) {
                soft->prp_pool_bitmap[w] &= ~(1u << b);
                soft->prp_pool_free_count--;
                return (w << 5) + b;
            }
        }
    }
    return -1;
}

/*
 * nvme_prp_pool_free_locked: Return a PRP list page to the pool
 *
 * Caller must hold soft->io_requests_lock.
 */
void
nvme_prp_pool_free_locked(nvme_soft_t *soft, int index)
{
    if (index < 0 || index >= NVME_PRP_POOL_SIZE) {
        cmn_err(CE_WARN, "nvme_prp_pool_free: invalid index %d", index);
        return;
    }
    soft->prp_pool_bitmap[index >> 5] |= (1u << (index & 31));
    soft->prp_pool_free_count++;
}

/*
 * nvme_prp_pages_for_bytes: Worst-case number of PRP list pages a single
 * NVMe command transferring 'bytes' bytes can need.
 *
 * The buffer may start at any offset inside a page, so a transfer of N
 * bytes can touch ceil(N / page) + 1 pages.  PRP1 covers the first page;
 * the remainder go in PRP2 (if exactly one) or in PRP list pages holding
 * (entries - 1) data pointers each (the last entry chains to the next
 * list page).  We round up conservatively.
 */
uint_t
nvme_prp_pages_for_bytes(nvme_soft_t *soft, uint_t bytes)
{
    uint_t page = soft->nvme_page_size;
    uint_t entries_per_list = soft->nvme_prp_entries - 1;
    uint_t data_pages;

    if (bytes == 0)
        return 0;
    if (soft->nvme_prp_entries < 2) {
        cmn_err(CE_PANIC, "nvme: nvme_prp_entries=%u is invalid", soft->nvme_prp_entries);
    }

    data_pages = (bytes + page - 1) / page + 1;   /* +1 for a possible offset */
    if (data_pages <= 2)
        return 0;                                  /* PRP1 + PRP2 direct */

    return (data_pages - 1 + entries_per_list - 1) / entries_per_list;
}

/*
 * nvme_compute_limits: Derive the per-command transfer limit.
 *
 * Called once Identify Controller (MDTS) and Identify Namespace (block
 * size) have both completed.  The limit is the smaller of:
 *   - MDTS (in units of CAP.MPSMIN pages), if non-zero
 *   - what NVME_CMD_MAX_PRPS chained PRP list pages can describe
 *   - a sanity cap so that mdts==0 drives don't get 32 MB commands
 *
 * The original code assumed 512-byte blocks when converting MDTS to
 * blocks, which is wrong for 4Kn-formatted namespaces and would have
 * produced commands larger than the drive allows.
 */
void
nvme_compute_limits(nvme_soft_t *soft)
{
    uint_t page = soft->nvme_page_size;
    uint_t max_bytes;
    uint_t prp_cap_pages;
    uint_t prp_cap_bytes;

    if (soft->mdts == 0) {
        max_bytes = 4u * 1024u * 1024u;            /* "unlimited": cap at 4 MB */
    } else {
        __uint64_t b = (__uint64_t)(1u << soft->mdts) * (1u << (soft->min_page_size + 12));
        if (b > 4u * 1024u * 1024u)
            b = 4u * 1024u * 1024u;
        max_bytes = (uint_t)b;
    }

    /* Largest transfer NVME_CMD_MAX_PRPS list pages can describe, minus one
     * page for a possible start offset. */
    prp_cap_pages = 1 + NVME_CMD_MAX_PRPS * (soft->nvme_prp_entries - 1) - 1;
    prp_cap_bytes = prp_cap_pages * page;
    if (max_bytes > prp_cap_bytes)
        max_bytes = prp_cap_bytes;

    /* Round down to a whole number of pages */
    max_bytes &= ~(page - 1);
    if (max_bytes < page)
        max_bytes = page;

    soft->max_transfer_bytes = max_bytes;
    if (soft->block_size == 0)
        soft->block_size = 512;                    /* identify namespace not done yet */
    soft->max_transfer_blocks = max_bytes / soft->block_size;
    soft->stats.max_transfer_blocks = soft->max_transfer_blocks;

    cmn_err(CE_NOTE, "nvme: per-command limit %u KB (%u blocks of %u bytes), "
            "PRP pool %u pages, CIDs %u",
            max_bytes / 1024, soft->max_transfer_blocks, soft->block_size,
            NVME_PRP_POOL_SIZE, soft->io_cid_max);
}

/*
 * nvme_io_reserve: Atomically reserve everything a request needs.
 *
 * Reserves 'commands' CIDs and the PRP list pages each command may need,
 * all under io_requests_lock, so that once this succeeds the request can
 * always be submitted without running out of anything half way through.
 *
 * Bitmap semantics: 0 = free, 1 = occupied
 *
 * Arguments:
 *   soft         - Controller state
 *   req          - SCSI request structure
 *   commands     - Number of CIDs to allocate
 *   prp_per_cmd  - PRP list pages needed by each command except the last
 *   prp_last_cmd - PRP list pages needed by the last command
 *   cids         - Output array (must have space for 'commands' entries)
 *
 * Returns:
 *   0 on success (all resources reserved, sr_ha refcount bumped by 'commands')
 *   -1 if resources are not available right now (nothing allocated)
 */
int
nvme_io_reserve(nvme_soft_t *soft, scsi_request_t *req, unsigned int commands,
                unsigned int prp_per_cmd, unsigned int prp_last_cmd,
                unsigned int *cids)
{
    unsigned int allocated = 0;
    unsigned int word_idx;
    unsigned int bit_idx;
    unsigned int word;
    unsigned int mask;
    unsigned int cid;
    unsigned int prp_total;
    unsigned int used;
    time_t timeout;
    int i, k;

    if (commands == 0 || commands > soft->io_cid_max) {
        return -1;
    }

    prp_total = (commands - 1) * prp_per_cmd + prp_last_cmd;

    /* Per-request timeout: what dksc asked for, but never less than our floor */
    timeout = req->sr_timeout;
    if (timeout <= 0)
        timeout = (time_t)NVME_IO_TIMEOUT_DEFAULT_SEC * HZ;
    if (timeout < (time_t)NVME_IO_TIMEOUT_MIN_SEC * HZ)
        timeout = (time_t)NVME_IO_TIMEOUT_MIN_SEC * HZ;

    mutex_lock(&soft->io_requests_lock, PZERO);

    /* Early rejection: check if we have enough of everything */
    if (soft->io_cid_free_count < commands ||
        soft->prp_pool_free_count < prp_total) {
        mutex_unlock(&soft->io_requests_lock);
        return -1;
    }

    /* Search for free bits in the bitmap */
    for (word_idx = 0; word_idx < (NVME_IO_QUEUE_SIZE/32) && allocated < commands; word_idx++) {
        word = soft->io_cid_bitmap[word_idx];

        if (word == 0xFFFFFFFF)
            continue;

        for (bit_idx = 0; bit_idx < 32 && allocated < commands; bit_idx++) {
            mask = 1u << bit_idx;
            if (!(word & mask)) {
                cid = ((word_idx << 5u) + bit_idx);
                if (cid >= soft->io_cid_max)
                    break;
                soft->io_cid_bitmap[word_idx] |= mask;
                cids[allocated] = cid;
                allocated++;
                word = soft->io_cid_bitmap[word_idx];
            }
        }
    }

    if (allocated < commands) {
        /* Should not happen given the free-count check, but be safe */
        for (i = 0; i < allocated; i++) {
            cid = cids[i];
            soft->io_cid_bitmap[cid >> 5u] &= ~(1u << (cid & 0x1F));
        }
        mutex_unlock(&soft->io_requests_lock);
        cmn_err(CE_WARN, "nvme_io_reserve: bitmap/free-count mismatch (requested %u, found %u)",
                commands, allocated);
        return -1;
    }

    soft->io_cid_free_count -= commands;

    /*
     * Initialize the CID slots while still holding the lock.  The timeout
     * scanner walks the bitmap under this lock and dereferences .req; the
     * old code set .req after unlocking, which let the scanner see a set
     * bit with a NULL req / stale start_time and abort (or crash on) a
     * command that had not even been built yet.
     */
    for (i = 0; i < commands; i++) {
        nvme_cmd_info_t *ci;
        unsigned int want = (i == commands - 1) ? prp_last_cmd : prp_per_cmd;

        cid = cids[i];
        ci = &soft->io_requests[cid];
        ci->req = req;
        ci->start_time = lbolt;
        ci->timeout = timeout;
        ci->aborted = 0;
        ci->nprp_used = 0;
        ci->nprp = 0;
        for (k = 0; k < NVME_CMD_MAX_PRPS; k++)
            ci->prpidx[k] = -1;
        for (k = 0; k < want && k < NVME_CMD_MAX_PRPS; k++) {
            int idx = nvme_prp_pool_alloc_locked(soft);
            if (idx < 0) {
                /* free-count said we had enough; treat as a bug but don't die */
                cmn_err(CE_WARN, "nvme_io_reserve: PRP pool underflow (free=%u)",
                        soft->prp_pool_free_count);
                break;
            }
            ci->prpidx[k] = idx;
            ci->nprp++;
        }
    }

    /* High-water marks for nvmectl stats */
    used = soft->io_cid_max - soft->io_cid_free_count;
    if (used > soft->stats.cid_max_used)
        soft->stats.cid_max_used = used;
    used = NVME_PRP_POOL_SIZE - soft->prp_pool_free_count;
    if (used > soft->stats.prp_max_used)
        soft->stats.prp_max_used = used;

    mutex_unlock(&soft->io_requests_lock);

    /* Atomically add the number of commands to sr_ha refcount */
    atomicAddInt((int *)&req->sr_ha, commands);

    return 0;
}

/*
 * nvme_io_cid_done: Free a CID and its PRP pages and retrieve req.
 *
 * Marks the CID as free, returns its PRP list pages to the pool and clears
 * the scsi_request pointer.  Decrements the reference count in req->sr_ha;
 * *last is set to 1 when the reference count reaches zero (all commands of
 * the request are done).
 *
 * Returns:
 *   scsi_request_t* that owned the CID (NULL if the CID was not in use)
 */
scsi_request_t *
nvme_io_cid_done(nvme_soft_t *soft, unsigned int cid, int *last)
{
    scsi_request_t *req;
    unsigned int word_idx;
    unsigned int bit_idx;
    unsigned int mask;
    unsigned int refcount;
    nvme_cmd_info_t *ci;
    int i;

    if (last)
        *last = 0;

    if (cid >= NVME_IO_QUEUE_SIZE) {
        cmn_err(CE_WARN, "nvme_io_cid_done: invalid CID %d", cid);
        return NULL;
    }

    word_idx = cid >> 5u;
    bit_idx = cid & 0x1F;
    mask = 1u << bit_idx;
    ci = &soft->io_requests[cid];

    mutex_lock(&soft->io_requests_lock, PZERO);

    if (!(soft->io_cid_bitmap[word_idx] & mask)) {
        /* Spurious / duplicate completion for a CID that is not in flight */
        mutex_unlock(&soft->io_requests_lock);
        return NULL;
    }

    req = ci->req;

    for (i = 0; i < NVME_CMD_MAX_PRPS; i++) {
        if (ci->prpidx[i] >= 0) {
            nvme_prp_pool_free_locked(soft, ci->prpidx[i]);
            ci->prpidx[i] = -1;
        }
    }
    ci->nprp = 0;
    ci->nprp_used = 0;
    ci->req = NULL;

    soft->io_cid_bitmap[word_idx] &= ~mask;
    soft->io_cid_free_count++;

    mutex_unlock(&soft->io_requests_lock);

    if (req != NULL) {
        refcount = atomicAddInt((int *)&req->sr_ha, -1);
#ifdef NVME_DBG_EXTRA
        cmn_err(CE_NOTE, "nvme_io_cid_done: CID %u done, refcount now %u", cid, refcount);
#endif
        if (last)
            *last = (refcount == 0);
    }

    return req;
}

/*
 * nvme_cmd_special_flush: Issue a special flush command (not tied to scsi_request)
 *
 * This is used for ordering guarantees when processing ordered or head-of-queue
 * commands. The flush uses a special CID (NVME_IO_CID_FLUSH) that won't conflict
 * with normal I/O CIDs (0-255).
 *
 * Returns:
 *   0 on success (command submitted)
 *   -1 on failure
 */
int
nvme_cmd_special_flush(nvme_soft_t *soft)
{
    nvme_command_t cmd;
    int rc;

    /* Build NVMe FLUSH command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_CMD_FLUSH | (NVME_IO_CID_FLUSH << 16);

    /* Set namespace ID to 1 (we always use namespace 1) */
    cmd.nsid = 1;

    /* Submit the command to the I/O queue */
    rc = nvme_submit_cmd(soft, &soft->io_queue, &cmd);
    if (rc != 0) {
#ifdef NVME_DBG
        cmn_err(CE_WARN, "nvme_cmd_special_flush: failed to submit special flush command");
#endif
        return -1;
    }

#ifdef NVME_DBG_EXTRA
    cmn_err(CE_NOTE, "nvme_cmd_special_flush: submitted special flush with CID 0x%x",
            NVME_IO_CID_FLUSH);
#endif
    return 0;
}

#ifdef NVME_TEST
void
nvme_cmd_admin_test(nvme_soft_t *soft, unsigned int i)
{
    nvme_command_t cmd;

    cmn_err(CE_NOTE, "nvme_admin_identify_controller: sending command");

    /* Clear utility buffer */
    bzero(soft->utility_buffer, NBPP);

    /* Build Identify Controller command */
    bzero(&cmd, sizeof(cmd));

    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_ADMIN_IDENTIFY | (i << 16);

    /* NSID: not used for controller identify */
    cmd.nsid = 0;

    /* PRP1: physical address of utility buffer (controller data destination) */
    cmd.prp1_lo = PHYS64_LO(soft->utility_buffer_phys);
    cmd.prp1_hi = PHYS64_HI(soft->utility_buffer_phys);

    /* PRP2: not needed (data is only 4KB) */
    cmd.prp2_lo = 0;
    cmd.prp2_hi = 0;

    /* CDW10: CNS = 0x01 for Identify Controller */
    cmd.cdw10 = NVME_CNS_CONTROLLER;

    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->admin_queue, &cmd) != 0) {
        cmn_err(CE_WARN, "nvme_cmd_admin_test: failed to submit command (queue full?)");
        return;  /* Failure */
    }
}

void
nvme_cmd_io_test(nvme_soft_t *soft, unsigned int i)
{
    nvme_command_t cmd;

    cmn_err(CE_NOTE, "nvme_admin_identify_controller: sending command");

    /* Clear utility buffer */
    bzero(soft->utility_buffer, NBPP);

    /* Build Identify Controller command */
    bzero(&cmd, sizeof(cmd));

    cmd.nsid = 1;

#if 1
    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_CMD_READ | (i << 16);
    cmd.cdw10 = 0; // lba low
    cmd.cdw11 = 0; // lba hi
    cmd.cdw12 = 0; // blocks
    cmd.prp1_lo = PHYS64_LO(soft->utility_buffer_phys);
    cmd.prp1_hi = PHYS64_HI(soft->utility_buffer_phys);
#endif

#if 0
    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_CMD_VERIFY | (i << 16);
    cmd.cdw10 = 0; // lba low
    cmd.cdw11 = 0; // lba hi
    cmd.cdw12 = 0; // blocks - 1, LR, FUA, PRINFO = 0
#endif

#if 0
    /* CDW0: Opcode (7:0), Flags (15:8), CID (31:16) */
    cmd.cdw0 = NVME_CMD_FLUSH | (i << 16);
#endif


    /* Submit command */
    if (nvme_submit_cmd(soft, &soft->io_queue, &cmd) != 0) {
        cmn_err(CE_WARN, "nvme_cmd_io_test: failed to submit command (queue full?)");
        return;  /* Failure */
    }
}

#endif
