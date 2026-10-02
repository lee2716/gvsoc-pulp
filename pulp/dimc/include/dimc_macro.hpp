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
// Pipeline depth: 4 stages, matching pipeline_mode[0:3] in spatz_dimc.sv
// (1 result/cycle throughput once the pipe is full)
#define DIMC_MACRO_LATENCY  4
// The dual's out_fifo is not fall-through: a result pushed in the cycle READYN goes low
// can be popped one cycle later.
#define DIMC_OUT_FIFO_DELAY 1
// Bytes one 32-bit result occupies in L1, for the results and for the partial sums
// read back. 4: packed, 8 results per 32 B port word (output packer, not in the RTL).
// 32: one sign-extended result per port word, as in the RTL. Result r sits at dst + SLOT * r.
#define DIMC_OUT_SLOT_BYTES 4
// The dual's out_fifo storage (64 x 32 b), used as one FIFO per macro of half that depth;
// a port word leaves straight from the head entries. Not in the RTL, which has one push
// through `sel` and one pop; same storage.
#define DIMC_OUT_FIFO_DEPTH 64
// One out_fifo per macro of half that depth.
// A macro's next-job kernel is fetched into the dual's weight FIFO as soon as no macro of
// the dual still owes kernel sections of its current program; the program then skips the
// sections already fetched. This holds for batched jobs too: the program fetches vector 0's
// partial sums and feature first, the kernel is fetched ahead of them.
// A macro that holds the next job's operands issues that job's rows before the running
// job closes. Their results wait in its out_fifo until the write-back reaches that job;
// issue stops while the out_fifo has no free entry, since a full out_fifo drops results.
// A macro is at most one job ahead of the running one. A macro that has written back every
// result of the running job writes its next job's results to that job's destination at
// once (a second address generator per macro, the next job's context), instead of holding
// them in its out_fifo until the running job closes.
// While the engine is idle and jobs are committed but held (commit-only), the kernel of the
// first held job is fetched into each dual's weight FIFO for its first macro; it is written
// into the macro, one section per cycle, once the job runs. Kernels do not depend on the
// job's inputs; features and partial sums are still fetched when the job starts.
// Macro write rules: no section is written into a macro in a cycle in which it computes
// (each waits for the cycle after the row trigger it depends on), and a macro takes one
// kernel or feature section per cycle, the feature section first. Partial sums go to the
// ADDIN sets outside the macro and are not counted.
// The dual's weight and input FIFOs: 256 b sections, one kernel and two feature vectors
// deep, written into a macro one section per cycle when its write port is open. One input
// FIFO per dual shared by both macros, in fetch order, one pop per cycle; a head section its
// macro cannot take yet holds the ones behind it.
#define DIMC_WGT_FIFO_DEPTH 128
#define DIMC_INP_FIFO_DEPTH 8
// N > 0: a dual whose weight FIFO holds at most N sections of the macro whose kernel is
// being written books the outer port for its kernel feed before any dual's input feed; the
// other duals, and every dual otherwise, book input then kernel. 0: always input then kernel.
#ifndef DIMC_KB_FEED_FIRST
#define DIMC_KB_FEED_FIRST 2
#endif
// N > 0: DIMC_KB_FEED_FIRST yields in a cycle where some macro's next input section is for
// a vector it may start within N rows (the macro waits, or is about to wait, for its inputs).
#ifndef DIMC_KB_FEED_YIELD
#define DIMC_KB_FEED_YIELD 6
#endif
// Shared input FIFO: rows a feature section may be fetched ahead of the triggers it waits for.
#ifndef DIMC_INP_FETCH_LEAD
#define DIMC_INP_FETCH_LEAD 32
#endif
// Kernel and feature storage is single-banked, as in the macro: spatz_dimc.sv
// declares one kernel_mem [31:0] and one feature_buf, and the two low bits of
// RA/WA/FA select one of a row's four 256-bit sections rather than a bank. A
// kernel section is therefore written into a macro only after the macro has triggered
// every row of its earlier jobs; fetches into the dual's FIFOs may run ahead.

struct DimcPipeEntry {
    int32_t  psout;
    int      job_row;
    int      cycles_remaining;
    int      set;              // result set of the job the row belongs to (job id & 1)
    int      run;              // the job's vector (batched descriptor) the row belongs to
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
        void write_psin_row(int row, const uint8_t *src, int set = 0);

        // Compute mode (COMPE = 1)
        int32_t compute_PP(int row_sel);
        void    final_compute();

        // Pipelined scheduling: issue is non-blocking unless pipeline is full;
        // tick advances the pipeline; has_ready/drain pop the front entry
        void issue(int row, int job_row, int set = 0, int run = 0);
        void tick();
        bool can_accept() const;
        bool has_ready() const;
        DimcPipeEntry drain();

        // Runtime configuration
        uint8_t  compe        = DIMC_COMPE_COMPUTE;  // latched, never acted on
        uint8_t  ci           = DIMC_CI_8BIT;
        uint8_t  sign_8b      = DIMC_SIGN_UU;
        uint16_t compute_mask = 0;   // bits masked off the 1024-bit row
        // Per-row partial-sum input, mirroring the RTL's ADDIN, which is
        // sampled together with the row address on every compute trigger
        // (spatz_dimc.sv) and is a port there, not a stored array. psin_scalar
        // is the per-job constant, and is what compute_PP uses while psin_rows
        // is off.
        int32_t  psin_scalar = 0;
        uint8_t  psin_rows   = 0;                        // 1 = take psin from psin_buf
        // Two partial-sum sets, alternating per run: set (job * NB_VEC + run) & 1.
        // They stand for what arrives on ADDIN from outside the macro, so the next
        // run's can be queued while this run's rows still use theirs; psin_sel picks
        // the set of the row being issued.
        int32_t  psin_buf_set[2][DIMC_MACRO_KB_LEN] = {{0}};
        uint8_t  psin_sel = 0;

        // Buffers
        uint8_t  KB[DIMC_MACRO_KB_LEN][DIMC_MACRO_KB_EW];
        uint8_t  FB[DIMC_MACRO_FB_EW];

        // Outputs
        int32_t  psout = 0;
        uint8_t  sout  = 0;   // computed by final_compute, never wired out

        // Set when the operands for the job this macro is executing have
        // landed. A flag of its own rather than a comparison on the fill
        // cursor's beat counters, which belong to the block and are reset per
        // phase.
        bool     exec_ready = false;
        // Job whose operands KB/FB hold, as the monotonic job id stamped at commit, not
        // the context slot: slot indices repeat, and a match on one would skip the fill.
        static constexpr uint32_t JOB_NONE = 0xFFFFFFFFu;
        uint32_t filled_job = JOB_NONE;
        // The job whose rows this macro is issuing; rows_issued counts for it.
        uint32_t issue_job  = JOB_NONE;
        // Shape of the job in filled_job, latched when its operands land, so the
        // macro can issue it before the engine has made it the running job.
        uint32_t job_rows = 0, job_row_base = 0;
        uint32_t job_nb_vec = 1;     // vectors (runs) of the job in filled_job
        // Runs of issue_job whose rows are all triggered; rows_issued counts inside the
        // current run.
        uint32_t runs_issued = 0;
        // The run whose feature is in the feature buffer, and when it landed.
        uint32_t fb_run = 0;
        uint64_t fb_ready_cycle = 0;
        // This macro's own fill program: the job it is filling and that job's context.
        // Per macro, so a macro that has finished all its runs can load the next job
        // while its sibling still runs the current one.
        uint32_t fill_job = JOB_NONE;
        uint32_t fill_slot = 0;
        // Fill programs started for this macro whose job it has not finished triggering.
        uint32_t owed = 0;
        // The write side of that program: the job whose sections are being popped from the
        // dual's FIFOs into this macro, its context, and how many have been written.
        uint32_t write_job = JOB_NONE;
        uint32_t write_slot = 0;
        // Contexts of filled_job and issue_job.
        uint32_t filled_slot = 0, issue_slot = 0;
        uint32_t written = 0;
        // Kernel of job kpf_job being streamed into the weight FIFO: kfetched sections so
        // far, by the program or ahead of it; kw of them written into the macro, and
        // f0 = vector 0's feature written, for the job being written (write_job).
        uint32_t kpf_job  = JOB_NONE;
        uint32_t kpf_slot = 0;
        uint32_t kfetched = 0;
        uint32_t kw = 0;
        bool     f0 = false;
        bool     stamped = false;
        // Cycle each row's partial sum lands; a row waits only for its own beat, since
        // ADDIN is taken with the row trigger.
        uint64_t psin_row_ready[2][DIMC_MACRO_KB_LEN] = {{0}};
        // Cycle after the kernel rows and vector 0's feature are written. No row issues
        // before it; the partial sums are gated per row (psin_row_ready).
        uint64_t fill_done_cycle = 0;
        // Rows pushed into this macro's pipe. Per macro, not per block, so a
        // macro that finished filling does not wait for its sibling.
        uint32_t rows_issued = 0;
        // Cycle of the last row trigger. No section is written into the macro in that cycle.
        int64_t  last_trigger_cycle = -1;
        // Cycle of the last kernel or feature section written into the macro.
        int64_t  last_write_cycle = -1;
        // Rows retired from the pipe per result set, with the set's job: two jobs can be
        // in flight on one macro. Store watermark: beat k may go once the count covers its rows.
        uint32_t rows_retired_set[2] = {0, 0};
        uint32_t set_job[2]          = {0xFFFFFFFFu, 0xFFFFFFFFu};

        // Staging for a feature vector assembled from its sections. Per macro, not
        // per block: once two macros fill concurrently a shared buffer would let one
        // overwrite the other's half-assembled vector.
        uint8_t  row_buffer[DIMC_MACRO_KB_EW] = {0};
        // Kernel rows are assembled apart from features: the two FIFOs pop independently,
        // so a feature's sections can arrive between a kernel row's.
        uint8_t  kb_row_buffer[DIMC_MACRO_KB_EW] = {0};
        static_assert(DIMC_MACRO_FB_EW <= DIMC_MACRO_KB_EW,
                      "row_buffer holds a feature vector; FB_EW must fit KB_EW");

        std::deque<DimcPipeEntry> pipe;
        // kb_ready and fb_ready are both raised when the FEATURE vector lands,
        // which is the last thing a macro waits for; on a reuse job no kernel
        // moves at all. The pair means "this macro has what it needs", not
        // "the kernel arrived".
        bool     kb_ready         = false;
        bool     fb_ready         = false;
};

#endif
