/*
 * Copyright (C) 2026 ETH Zurich, University of Bologna and Fondazione ChipsIT
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __DIMC_HWPE_ARCHI__
#define __DIMC_HWPE_ARCHI__

#define N_CFG_REGS 48

#define DIMC_HWPE_BASE   0x0

/* ================= Mandatory control block @ 0x00 =================
 * Standard HWPE control interface (hwpe-ctrl); byte-for-byte identical to
 * pulp/redmule. NOTE: hwpe_ctrl_regif_example.rdl places `autotrigger` at
 * 0x08, whereas redmule -- and this model -- expose FINISHED there.
 * 0x18/0x1C are reserved; 0x18 additionally serves as the command-vs-config
 * dispatch boundary in hwpe_slave().
 */
#define DIMC_HWPE_TRIG               (DIMC_HWPE_BASE + 0x00)  /* commit_trigger */
#define DIMC_HWPE_ACQ                (DIMC_HWPE_BASE + 0x04)  /* acquire        */
#define DIMC_HWPE_FIN_JOBS           (DIMC_HWPE_BASE + 0x08)  /* finished       */
#define DIMC_HWPE_STATUS             (DIMC_HWPE_BASE + 0x0C)  /* status         */
#define DIMC_HWPE_RUN_TASK           (DIMC_HWPE_BASE + 0x10)  /* running_job    */
#define DIMC_HWPE_SOFT_CLEAR         (DIMC_HWPE_BASE + 0x14)  /* soft_clear     */
#define DIMC_HWPE_CHK_STATE          (DIMC_HWPE_BASE + 0x18)  /* reserved + boundary */

/* ============ Generic / job-INDEPENDENT registers @ 0x20 ============
 * Compute configuration shared by every queued job. Latched at commit and
 * broadcast to all macros; NOT part of a per-job context.
 */
#define DIMC_HWPE_CFG_CI             (DIMC_HWPE_BASE + 0x20)  /* INT1/2/4/8      */
#define DIMC_HWPE_SIGN_MODE          (DIMC_HWPE_BASE + 0x24)  /* INT8 sign combo */
#define DIMC_HWPE_COMPE              (DIMC_HWPE_BASE + 0x28)  /* memory/compute  */
#define DIMC_HWPE_MCT                (DIMC_HWPE_BASE + 0x2C)  /* thermometric mask */
#define DIMC_HWPE_NUM_MACROS         (DIMC_HWPE_BASE + 0x30)
#define DIMC_HWPE_SEL_DIMC           (DIMC_HWPE_BASE + 0x34)
/* autotrigger (hwpe-ctrl semantics, RDL puts it at 0x08):
 *   0 (default) = automatically start the next queued job when one finishes
 *   1           = wait for an explicit trigger (commit_trigger 0x0 / 0x2)
 * It lives here rather than at 0x08 because -- like pulp/redmule -- this model
 * exposes `finished` at 0x08. Job-INDEPENDENT, so it is not banked per context. */
#define DIMC_HWPE_AUTOTRIGGER_N      (DIMC_HWPE_BASE + 0x38)

/* Output accumulator, one per inner block (rtl/accumulator.sv via cleopatra.sv).
 * Sums every result the block emits, across rows and across jobs.
 * Job-INDEPENDENT: ACC_CTRL must stay below JOB_BASE or it gets banked per
 * context and the accumulator turns per-job. */
#define DIMC_HWPE_ACC_CTRL           (DIMC_HWPE_BASE + 0x3C)  /* b0=enable b1=clear */
#define DIMC_HWPE_ACC_CTRL_EN_BIT    (1 << 0)
#define DIMC_HWPE_ACC_CTRL_CLR_BIT   (1 << 1)
#define DIMC_HWPE_ACC_VAL_0          (DIMC_HWPE_BASE + 0x8C)  /* read-only acc_o, block 0 */
#define DIMC_HWPE_ACC_VAL_1          (DIMC_HWPE_BASE + 0x90)  /* read-only acc_o, block 1 */

/* ============== Job-DEPENDENT registers @ 0x40 ==============
 * Per-job context: data pointers, stream geometry, per-job scalars.
 * This block is BANKED PER CONTEXT (DIMC_NB_CONTEXT copies): software ACQUIREs
 * a context, writes this block into it, then commits. A second job can be
 * offloaded while the first one is still running.
 */
#define DIMC_NB_CONTEXT              2
#define DIMC_HWPE_JOB_BASE           (DIMC_HWPE_BASE + 0x40)
#define DIMC_HWPE_NB_JOB_REGS        23   /* 0x40 .. 0x98 inclusive; 0x8C/0x90 are
                                          * intercepted as ACC_VAL and leave holes */
#define DIMC_HWPE_JOB_KB_SRC_ADDR    (DIMC_HWPE_BASE + 0x40)
#define DIMC_HWPE_JOB_FB_SRC_ADDR    (DIMC_HWPE_BASE + 0x44)
#define DIMC_HWPE_JOB_DST_ADDR       (DIMC_HWPE_BASE + 0x48)

#define DIMC_HWPE_KB_TOTAL_LENGTH    (DIMC_HWPE_BASE + 0x4C)
#define DIMC_HWPE_KB_D0_LENGTH       (DIMC_HWPE_BASE + 0x50)
#define DIMC_HWPE_KB_D0_STRIDE       (DIMC_HWPE_BASE + 0x54)
#define DIMC_HWPE_KB_D1_LENGTH       (DIMC_HWPE_BASE + 0x58)
#define DIMC_HWPE_KB_D1_STRIDE       (DIMC_HWPE_BASE + 0x5C)

#define DIMC_HWPE_FB_TOTAL_LENGTH    (DIMC_HWPE_BASE + 0x60)
#define DIMC_HWPE_FB_D0_LENGTH       (DIMC_HWPE_BASE + 0x64)
#define DIMC_HWPE_FB_D0_STRIDE       (DIMC_HWPE_BASE + 0x68)

#define DIMC_HWPE_OUT_TOTAL_LENGTH   (DIMC_HWPE_BASE + 0x6C)
#define DIMC_HWPE_OUT_D0_LENGTH      (DIMC_HWPE_BASE + 0x70)
#define DIMC_HWPE_OUT_D0_STRIDE      (DIMC_HWPE_BASE + 0x74)

#define DIMC_HWPE_ROW_SEL_BASE       (DIMC_HWPE_BASE + 0x78)
#define DIMC_HWPE_ROW_COUNT          (DIMC_HWPE_BASE + 0x7C)
#define DIMC_HWPE_CFG_BIAS           (DIMC_HWPE_BASE + 0x80)
#define DIMC_HWPE_PSIN               (DIMC_HWPE_BASE + 0x84)

/* Streamer bandwidth and kernel reuse are not MMIO: bandwidth is fixed in the
 * systree, and reuse is detected in the FSM by comparing the KB source address
 * to the previous job's. */
/* Partial-sum chain control. A dot product wider than one macro row (K > the
 * row width) is split into chunks that run back to back on the same macro:
 *   0 = start a fresh sum, the chunk's result replaces the accumulator
 *   1 = add this chunk to the accumulator the previous chunk left behind
 * The accumulator lives inside the macro (one entry per row), so chaining costs
 * no extra memory traffic. Software reads the final result from DST after the
 * last chunk. */
#define DIMC_HWPE_ACCUM_EN           (DIMC_HWPE_BASE + 0x88)

/* Per-row partial-sum input, the model of the RTL's ADDIN. With PSIN_EN set the
 * engine streams one 32-bit psum per kernel row from PSIN_SRC_ADDR and adds it to
 * that row's dot product; with it clear the per-job PSIN scalar is used instead
 * and nothing extra is loaded. This is what lets a K-chain keep several partial
 * sums alive at once, so weights can be reused across a batch. */
#define DIMC_HWPE_PSIN_EN            (DIMC_HWPE_BASE + 0x94)
#define DIMC_HWPE_JOB_PSIN_SRC_ADDR  (DIMC_HWPE_BASE + 0x98)

#define DIMC_HWPE_REG_MAX            (DIMC_HWPE_BASE + 0x98)

#endif
