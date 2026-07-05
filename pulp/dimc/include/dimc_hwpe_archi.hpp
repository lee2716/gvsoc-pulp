#ifndef __DIMC_HWPE_ARCHI__
#define __DIMC_HWPE_ARCHI__

#define N_CFG_REGS 48

#define DIMC_HWPE_BASE   0x0

#define DIMC_HWPE_TRIG               (DIMC_HWPE_BASE + 0x00)
#define DIMC_HWPE_ACQ                (DIMC_HWPE_BASE + 0x04)
#define DIMC_HWPE_FIN_JOBS           (DIMC_HWPE_BASE + 0x08)
#define DIMC_HWPE_STATUS             (DIMC_HWPE_BASE + 0x0C)
#define DIMC_HWPE_RUN_TASK           (DIMC_HWPE_BASE + 0x10)
#define DIMC_HWPE_SOFT_CLEAR         (DIMC_HWPE_BASE + 0x14)
#define DIMC_HWPE_CHK_STATE          (DIMC_HWPE_BASE + 0x18)

#define DIMC_HWPE_CFG_CI             (DIMC_HWPE_BASE + 0x1C)
#define DIMC_HWPE_CFG_BIAS           (DIMC_HWPE_BASE + 0x20)
#define DIMC_HWPE_CFG_VM             (DIMC_HWPE_BASE + 0x24)
#define DIMC_HWPE_CFG_BURST          (DIMC_HWPE_BASE + 0x28)

#define DIMC_HWPE_JOB_KB_SRC_ADDR    (DIMC_HWPE_BASE + 0x30)
#define DIMC_HWPE_JOB_FB_SRC_ADDR    (DIMC_HWPE_BASE + 0x34)
#define DIMC_HWPE_JOB_DST_ADDR       (DIMC_HWPE_BASE + 0x38)

#define DIMC_HWPE_KB_TOTAL_LENGTH    (DIMC_HWPE_BASE + 0x40)
#define DIMC_HWPE_KB_D0_LENGTH       (DIMC_HWPE_BASE + 0x44)
#define DIMC_HWPE_KB_D0_STRIDE       (DIMC_HWPE_BASE + 0x48)
#define DIMC_HWPE_KB_D1_LENGTH       (DIMC_HWPE_BASE + 0x4C)
#define DIMC_HWPE_KB_D1_STRIDE       (DIMC_HWPE_BASE + 0x50)

#define DIMC_HWPE_FB_TOTAL_LENGTH    (DIMC_HWPE_BASE + 0x60)
#define DIMC_HWPE_FB_D0_LENGTH       (DIMC_HWPE_BASE + 0x64)
#define DIMC_HWPE_FB_D0_STRIDE       (DIMC_HWPE_BASE + 0x68)

#define DIMC_HWPE_OUT_TOTAL_LENGTH   (DIMC_HWPE_BASE + 0x70)
#define DIMC_HWPE_OUT_D0_LENGTH      (DIMC_HWPE_BASE + 0x74)
#define DIMC_HWPE_OUT_D0_STRIDE      (DIMC_HWPE_BASE + 0x78)

#define DIMC_HWPE_NUM_MACROS         (DIMC_HWPE_BASE + 0x80)
#define DIMC_HWPE_ROW_SEL_BASE       (DIMC_HWPE_BASE + 0x84)
#define DIMC_HWPE_ROW_COUNT          (DIMC_HWPE_BASE + 0x88)

#define DIMC_HWPE_SEL_DIMC           (DIMC_HWPE_BASE + 0x90)
#define DIMC_HWPE_COMPE              (DIMC_HWPE_BASE + 0x94)
#define DIMC_HWPE_SIGN_MODE          (DIMC_HWPE_BASE + 0x98)
#define DIMC_HWPE_MCT                (DIMC_HWPE_BASE + 0x9C)
#define DIMC_HWPE_PSIN               (DIMC_HWPE_BASE + 0xA0)

// Streamer bandwidth (chunk/l1bw/sync) and kernel reuse are NOT MMIO: bandwidth
// is a fixed hardware property set from the systree / gvrun --param, and reuse is
// auto-detected in the FSM by comparing the KB/FB source address to the last job.
#define DIMC_HWPE_REG_MAX            (DIMC_HWPE_BASE + 0xA0)

#endif
