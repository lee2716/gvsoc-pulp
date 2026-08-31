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

#ifndef __DIMC_MACRO_HPP__
#define __DIMC_MACRO_HPP__

#include <cstdint>
#include <deque>

// DIMC macro design parameters
#define DIMC_MACRO_KB_LEN   32
#define DIMC_MACRO_KB_EW    128
#define DIMC_MACRO_FB_EW    128
// Pipeline depth: 4 cycles per spatz_DIMC.sv (1 result/cycle throughput after fill)
#define DIMC_MACRO_LATENCY  4
// Kernel, feature and partial-sum storage is double banked: one bank feeds the
// running job while the other takes the next one's operands.
#define DIMC_MACRO_NB_BANKS 2

struct DimcPipeEntry {
    int32_t  psout;
    int      job_row;
    int      cycles_remaining;
};

// Compute mode
#define DIMC_COMPE_MEM      0
#define DIMC_COMPE_COMPUTE  1

// Ci precision mode
#define DIMC_CI_1BIT        0
#define DIMC_CI_2BIT        1
#define DIMC_CI_4BIT        2
#define DIMC_CI_8BIT        3

// sign_8b, read only at INT8: bit 0 signs the kernel, bit 1 the feature.
#define DIMC_SIGN_UU        0
#define DIMC_SIGN_SU        1
#define DIMC_SIGN_US        2
#define DIMC_SIGN_SS        3

class Dimc_Macro {
    public:
        Dimc_Macro();

        void reset();

        // Memory mode (COMPE = 0)
        void write_row(int row, const uint8_t *src);
        void read_row(int row, uint8_t *dst) const;
        void write_fb(const uint8_t *src);
        void write_psin_row(int row, const uint8_t *src);

        // Compute mode (COMPE = 1)
        int32_t compute_PP(int row_sel);
        int32_t final_compute(int32_t bias);

        // Pipelined scheduling: issue is non-blocking unless pipeline is full;
        // tick advances the pipeline; has_ready/drain pop the front entry
        void issue(int row, int job_row, int32_t bias);
        void tick();
        bool can_accept() const;
        bool has_ready() const;
        DimcPipeEntry drain();

        // Runtime configuration
        uint8_t  compe        = DIMC_COMPE_COMPUTE;  // latched, never acted on
        uint8_t  ci           = DIMC_CI_8BIT;
        uint8_t  sign_8b      = DIMC_SIGN_UU;
        uint16_t compute_mask = 0;   // bits masked off the 1024-bit row
        // Per-row partial-sum input, mirroring the RTL's ADDIN, which is sampled
        // together with the row address on every compute trigger
        // (spatz_DIMC.sv:224). psin_scalar is the legacy per-job constant and is
        // what compute_PP uses while psin_rows is off.
        int32_t  psin_scalar = 0;
        uint8_t  psin_rows   = 0;                        // 1 = take psin from psin_buf
        // Two banks of everything a compute trigger reads, so the next job's
        // data can land while this one still runs. The kernel and the feature
        // side need their own pair of pointers: on a reuse job the kernel is
        // not reloaded and has to stay where it is, while the features and the
        // partial sums change every job.
        uint8_t  kb_cur = 0, kb_fill = 0;
        uint8_t  fb_cur = 0, fb_fill = 0;
        int32_t  psin_buf[DIMC_MACRO_NB_BANKS][DIMC_MACRO_KB_LEN] = {{0}};

        // Buffers
        uint8_t  KB[DIMC_MACRO_NB_BANKS][DIMC_MACRO_KB_LEN][DIMC_MACRO_KB_EW];
        uint8_t  FB[DIMC_MACRO_NB_BANKS][DIMC_MACRO_FB_EW];

        // Outputs
        int32_t  psout = 0;
        uint8_t  sout  = 0;   // computed by final_compute, never wired out

        // Which in-flight job this macro is working on, and the kernel base it
        // last pulled in. Per macro because once two jobs overlap the macros
        // are on different ones, and a single engine-wide last_kb_src would be
        // overwritten by whichever macro filled most recently -- every reuse
        // would then miss and reload weights that were already resident.

        // "the operands for the job I am executing are in cur". A flag of its
        // own, not a comparison on the fill's beat counters: those live in the
        // fill cursor and a prefetch resets its own copy under this macro at
        // any time.
        bool     exec_ready = false;
        // Job-relative timestamps, instrumentation only.
        uint32_t trace_fill_start = 0, trace_fill_done = 0;
        uint32_t trace_compute_start = 0, trace_compute_end   = 0;
        uint32_t job_slot   = 0;
        uint32_t last_kb_src = 0xFFFFFFFFu;
        bool     skip_kb    = false;
        // Cycle its last preload beat lands. A macro may not compute before it:
        // can_accept() only tracks kb_ready/fb_ready, which are raised at the
        // feature beat, and the partial sums arrive after that.
        uint64_t fill_done_cycle = 0;
        // Rows pushed into this macro's pipe. Per macro, not per block, so a
        // macro that finished filling does not wait for its sibling.
        uint32_t rows_issued = 0;
        // Rows this macro has retired from its pipe into out_buf. The store
        // watermark: beat k may go once rows_retired covers the rows it carries.
        uint32_t rows_retired = 0;

        // Staging for the beat in flight. Per macro, not per block: once two
        // macros fill concurrently a shared buffer would let one overwrite the
        // other's half-assembled row. Three preload paths use it -- a kernel
        // row, a feature vector, and the whole per-row partial-sum block -- and
        // all three end at exactly DIMC_MACRO_KB_EW, which nothing else
        // declares, so assert it.
        uint8_t  row_buffer[DIMC_MACRO_KB_EW] = {0};
        static_assert(DIMC_MACRO_FB_EW <= DIMC_MACRO_KB_EW,
                      "row_buffer holds a feature vector; FB_EW must fit KB_EW");
        static_assert(DIMC_MACRO_KB_LEN * 4 <= DIMC_MACRO_KB_EW,
                      "row_buffer holds row_count 32-bit partial sums");

        std::deque<DimcPipeEntry> pipe;
        // kb_ready and fb_ready are both raised when the FEATURE vector lands,
        // which is the last thing a macro waits for; on a reuse job no kernel
        // moves at all. The pair means "this macro has what it needs", not
        // "the kernel arrived".
        bool     kb_ready         = false;
        bool     fb_ready         = false;
};

#endif
