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
// The dual's out_fifo storage (64 x 32 b), split evenly into one FIFO per macro; a port
// word leaves straight from the head entries. The RTL has one FIFO, pushed from the macro
// `sel` picks, and one pop.
#define DIMC_OUT_FIFO_DEPTH 64
// Kernel sections are also fetched ahead of a macro's program (kernel_prefetch) on outer-port
// cycles the programs leave free: the kernel under way first, else the lowest-index macro's
// kernel of its current job (program not yet at the kernel) or of its next job. One macro's
// kernel is fetched at a time; the program then skips the sections already fetched.
// A macro holding the next job's operands issues that job's rows before the running job
// closes. Their results wait in its out_fifo until the write-back reaches that job, and it
// issues only while out_fifo plus pipe stay below its out_fifo's depth, since a full out_fifo
// drops results. A macro is at most one job ahead of the running one.
// While the engine is idle with committed jobs held (commit-only), each dual fetches the first
// held job's kernel for its first macro into the weight FIFO; it is written into the macro
// once the job runs. Features and partial sums are fetched once the job starts.
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

        // issue pushes a row into the pipe (ignored unless can_accept); tick advances
        // the pipe; has_ready tests and drain pops the front entry.
        void issue(int row, int job_row, int set = 0, int run = 0);
        void tick();
        bool can_accept() const;
        bool has_ready() const;
        DimcPipeEntry drain();

        // Runtime configuration
        uint8_t  ci           = DIMC_CI_8BIT;
        uint8_t  sign_8b      = DIMC_SIGN_UU;
        uint16_t compute_mask = 0;   // bits masked off the 1024-bit row
        // ADDIN, which spatz_dimc.sv samples with the row address on each trigger.
        // psin_scalar is the per-job constant compute_PP adds while psin_rows is 0.
        int32_t  psin_scalar = 0;
        uint8_t  psin_rows   = 0;                        // 1 = take the row's psin from psin_buf_set
        // Two partial-sum sets, alternating per run across jobs: set (run_base + run) & 1, run_base
        // counting the runs of every job committed before.
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

        // Job whose operands KB/FB hold, set once its kernel and vector 0's feature are written
        // (JOB_NONE before). The job id stamped at commit, not the context slot: slots repeat.
        static constexpr uint32_t JOB_NONE = 0xFFFFFFFFu;
        uint32_t filled_job = JOB_NONE;
        // The job whose rows this macro is issuing; rows_issued counts for it.
        uint32_t issue_job  = JOB_NONE;
        // Shape of the job in filled_job, latched when its operands land, so the
        // macro can issue it before the engine has made it the running job.
        uint32_t job_rows = 0, job_row_base = 0;
        uint32_t job_nb_vec = 1;     // vectors (runs) of the job in filled_job
        uint32_t job_run_base = 0;   // runs of the jobs committed before filled_job
        // Runs of issue_job whose rows are all triggered; rows_issued counts inside the
        // current run.
        uint32_t runs_issued = 0;
        // The run whose feature is in the feature buffer, and the first cycle a row may use it.
        uint32_t fb_run = 0;
        uint64_t fb_ready_cycle = 0;
        // This macro's own fill program: the job it is filling and that job's context.
        // Per macro, so a macro whose program is spent can start the next job's while
        // its sibling still fills or runs the current one.
        uint32_t fill_job = JOB_NONE;
        uint32_t fill_slot = 0;
        // Fill programs started for this macro whose job it has not finished triggering.
        uint32_t owed = 0;
        // The write side of that program: the job whose sections are being popped from the
        // dual's FIFOs into this macro, and its context.
        uint32_t write_job = JOB_NONE;
        uint32_t write_slot = 0;
        // Sections written for write_job.
        uint32_t written = 0;
        // Kernel being fetched into the weight FIFO, by the program or ahead of it: job kpf_job,
        // kfetched sections so far. For write_job: kw kernel sections written, f0 = vector 0's
        // feature written, stamped = handed to compute as filled_job.
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
        // Rows of the current run of issue_job pushed into this macro's pipe.
        uint32_t rows_issued = 0;
        // Cycle of the last row trigger. No section is written into the macro in that cycle.
        int64_t  last_trigger_cycle = -1;
        // Cycle of the last kernel or feature section written into the macro.
        int64_t  last_write_cycle = -1;
        // Rows retired from the pipe per result set (job id & 1), with the set's job: two jobs
        // can be in flight on one macro. rows_done() closes the job on these counts.
        uint32_t rows_retired_set[2] = {0, 0};
        uint32_t set_job[2]          = {0xFFFFFFFFu, 0xFFFFFFFFu};

        // Staging for a feature vector assembled from its sections. Per macro: the dual's
        // input FIFO interleaves both macros' sections.
        uint8_t  row_buffer[DIMC_MACRO_KB_EW] = {0};
        // Kernel rows are assembled apart from features: the two FIFOs pop independently,
        // so a feature's sections can arrive between a kernel row's.
        uint8_t  kb_row_buffer[DIMC_MACRO_KB_EW] = {0};
        static_assert(DIMC_MACRO_FB_EW <= DIMC_MACRO_KB_EW,
                      "row_buffer holds a feature vector; FB_EW must fit KB_EW");

        std::deque<DimcPipeEntry> pipe;
        // Both set when a feature vector is written, cleared at job start for a macro not
        // on the new job; can_accept needs both. Neither tracks the kernel.
        bool     kb_ready         = false;
        bool     fb_ready         = false;
};

#endif
