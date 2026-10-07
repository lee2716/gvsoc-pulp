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

/* Software interface: the hwpe-ctrl control block and the job registers of
 * dual_DIMC/rtl/dimc_package.sv. One job is one matrix multiplication
 * Result[weight_rows][input_cols] = Weights[weight_rows][weight_cols] x Inputs[input_rows][input_cols],
 * all three row-major in memory, 8-bit operands, 32-bit results. */
#define N_CFG_REGS 8

#define DIMC_HWPE_TRIG               0x00  /* commit the acquired job and start it        */
#define DIMC_HWPE_ACQ                0x04  /* read: job id, or negative when no slot free */
#define DIMC_HWPE_FIN_JOBS           0x08  /* jobs finished                               */
#define DIMC_HWPE_STATUS             0x0C  /* 0 while a job runs, 1 after it              */
#define DIMC_HWPE_RUN_TASK           0x10  /* id of the running job                       */
#define DIMC_HWPE_SOFT_CLEAR         0x14  /* abort everything                            */
#define DIMC_HWPE_EXT_STATUS         0x18  /* bit 0 busy, bit 1 the last job was rejected */

#define DIMC_NB_CONTEXT              2     /* jobs the queue holds, the running one included */

/* Job registers at DIMC_IO_BASE + 4 * index. */
#define DIMC_IO_BASE                 0x20
#define DIMC_REG_INPUT_ADDR          0
#define DIMC_REG_KERNEL_ADDR         1
#define DIMC_REG_OUTPUT_ADDR         2
#define DIMC_REG_WEIGHT_ROWS         3     /* multiple of 32            */
#define DIMC_REG_WEIGHT_COLS         4     /* multiple of 128           */
#define DIMC_REG_INPUT_ROWS          5     /* = weight_cols             */
#define DIMC_REG_INPUT_COLS          6     /* multiple of 8             */
#define DIMC_REG_FORMAT              7     /* [1:0] mode (3 = INT8), [3:2] sign_8b */
#define DIMC_REG_BIAS                8     /* ADDIN, added once per 128-column tile */
#define DIMC_REG_COMPUTE_MASK        9
#define DIMC_REG_WRITE_MASK          10    /* eight words; stored, not modelled */
#define DIMC_NB_REGS                 18
#define DIMC_HWPE_REG_MAX            (DIMC_IO_BASE + 4 * DIMC_NB_REGS - 4)

/* One tile product (32 output rows x 128 columns x 8 vectors) as the engine runs it: the
 * descriptor the job sequencer writes into an engine context. Not software-visible. */
enum {
    DIMC_TP_KB_SRC,          /* first kernel row of inner block 0             */
    DIMC_TP_KB_ROW_STRIDE,   /* bytes from one kernel row to the next         */
    DIMC_TP_FB_SRC,          /* first input row of the tile                   */
    DIMC_TP_FB_ROW_STRIDE,
    DIMC_TP_DST,             /* first output row of inner block 0             */
    DIMC_TP_OUT_ROW_STRIDE,
    DIMC_TP_BIAS,
    DIMC_TP_TILE_LAST,       /* last tile product of its output tile: write the accumulators out */
    DIMC_TP_NB_BLOCKS,       /* inner blocks that take part                   */
    DIMC_TP_NB_REGS
};

#endif
