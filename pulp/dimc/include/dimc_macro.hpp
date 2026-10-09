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
// Bytes one 32-bit result occupies in L1: packed, 8 results per 256-bit port word.
#define DIMC_OUT_SLOT_BYTES 4
// The dual's out_fifo (64 x 32 b): results of the macro that computes, popped one per cycle
// into the dual's accumulators.
#define DIMC_OUT_FIFO_DEPTH 64
// The dual's accumulators, as in cleopatra.sv: one 32-bit register per (vector, row), so a job
// carries at most DIMC_ACC_VECTORS vectors.
#define DIMC_ACC_VECTORS    8
// The engine runs one tile product at a time (32 output rows x 128 columns x 8 vectors); in its
// code a "job" is a tile product. The two macros of a dual take turns on the same output rows:
// tile product j runs on macro j % 2 while the other macro is loaded with the next one's kernel
// and first vector. A macro starts once the other has triggered every row of the tile product
// before and every result of it is in the accumulators. Results stay in the accumulators until
// the last tile product of an output tile (TILE_LAST), which writes them out and clears them;
// nothing computes meanwhile.
// A TILE_LAST tile product is written out two cycles after its last result is accumulated
// (PRESENT, then the write request), and the next one starts DIMC_PRESENT_END_CYCLES after the
// last word is acknowledged (sink done, result handshake, accumulator clear, tile advance, run).
#define DIMC_PRESENT_START_CYCLES 2
#define DIMC_PRESENT_END_CYCLES   5
// Macro write rules: no section is written into a macro in a cycle in which it computes
// (each waits for the cycle after the row trigger it depends on), and a macro takes one
// kernel or feature section per cycle, the feature section first.
// The dual's weight and input FIFOs: 256 b sections, one kernel and eight feature vectors
// deep, written into a macro one section per cycle when its write port is open. One input
// FIFO per dual shared by both macros, in job order, one pop per cycle; a job's vectors are
// all fetched before its macro starts.
#define DIMC_WGT_FIFO_DEPTH 128
#define DIMC_INP_FIFO_DEPTH 32
// The outer block has one 512-bit port to L1 for reads and writes. A mux gives each dual 256 b
// of it, and a dual puts one request on its half per cycle: a result word, else an input row,
// else a kernel section.
#define DIMC_PORT_BYTES      64
#define DIMC_DUAL_PORT_BYTES (DIMC_PORT_BYTES / 2)
// Cycles the dual takes to switch macros after the last result of a job is accumulated: the
// result count is compared a cycle later, then one cycle swaps the macros (dimc_datapath.sv).
#define DIMC_SWITCH_CYCLES 2
// 1: the macros switch as in dimc_datapath.sv, after the last result of a tile product is
// accumulated. 0: the other macro triggers the cycle after the last trigger, and the out_fifo
// takes each macro's results as they leave its pipe (`sel` follows the pipeline).
#ifndef DIMC_SWITCH_GAP
#define DIMC_SWITCH_GAP 1
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
        // ADDIN, which spatz_dimc.sv samples with the row address on each trigger: the job's
        // bias.
        int32_t  addin = 0;

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
        // Runs of issue_job whose rows are all triggered; rows_issued counts inside the
        // current run.
        uint32_t runs_issued = 0;
        // The run whose feature is in the feature buffer, and the first cycle a row may use it.
        uint32_t fb_run = 0;
        uint64_t fb_ready_cycle = 0;
        // This macro's own fill program: the job it is filling and that job's context.
        // Per macro, so a macro whose program is spent can start the next job's while
        // the other macro runs the current one.
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
        // Kernel being fetched into the weight FIFO: job kpf_job,
        // kfetched sections so far. For write_job: kw kernel sections written, f0 = vector 0's
        // feature written, stamped = handed to compute as filled_job.
        uint32_t kpf_job  = JOB_NONE;
        uint32_t kpf_slot = 0;
        uint32_t kfetched = 0;
        uint32_t ffetched = 0;       // feature sections of fill_job fetched
        uint32_t kw = 0;
        bool     f0 = false;
        bool     stamped = false;
        // Cycle after the kernel rows and vector 0's feature are written. No row issues
        // before it.
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
