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

#ifndef __DIMC_TRACER_HPP__
#define __DIMC_TRACER_HPP__

#include <vp/vp.hpp>
#include <cstdint>
#include <vector>

class Dimc_HWPE;

// Why a macro did or did not work in a cycle, traced per macro as
// block_<b>/macro_<m>/why so every cycle of a job has a named cause.
enum dimc_why_t : uint8_t {
    DIMC_WHY_IDLE             = 0,    // no job running
    // One cause per fill stream, matching the beat kinds preload_block issues.
    DIMC_WHY_LOAD_KB          = 1,    // filling the kernel buffer: the weights
    DIMC_WHY_LOAD_FB          = 2,    // filling the feature buffer: the inputs
    DIMC_WHY_LOAD_PSIN        = 3,    // filling the partial sums
    DIMC_WHY_COMPUTE          = 4,
    DIMC_WHY_WRITE_BACK       = 5,
    DIMC_WHY_WAIT_INNER_PORT  = 6,    // heuristic: a lower-index macro still owes fill sections
    DIMC_WHY_WAIT_OUTER_PORT  = 7,    // this macro's beat was due, the shared outer port was booked
    DIMC_WHY_WAIT_DEPTH       = 8,    // outstanding_depth beats in flight, or the weight FIFO full
    DIMC_WHY_WAIT_FILL_ACK    = 9,    // last fill beat issued, its response not in yet
    DIMC_WHY_PIPE_DRAIN       = 10,   // rows issued, results still inside the macro pipeline
    DIMC_WHY_WAIT_STORE_ORDER = 11,   // results ready, the block writes an earlier macro back first
    DIMC_WHY_WAIT_BEAT_ACK    = 12,   // every beat issued, a response still in flight
    DIMC_WHY_DONE_WAIT_JOB    = 13,   // this macro is finished, the job waits for another macro or block
    DIMC_WHY_FSM_STEP         = 14,   // a phase cycle with no work
    DIMC_WHY_WAIT_COMPUTE     = 15,   // a section fetch or write held: beat_writable refused it
    DIMC_WHY_WAIT_PSUM        = 16,   // the partial sums' producer run is not written back yet
    DIMC_WHY_WAIT_WGT_FIFO    = 17,   // its kernel sections wait behind another macro's in the dual's weight FIFO
    DIMC_WHY_UNKNOWN          = 255   // must never appear
};

// fill_grant / outer_grant value when nobody was served this cycle. fill_grant is the macro
// a FIFO section was written into; outer_grant the block that booked the outer port.
#define DIMC_GRANT_NONE 255

// Which buffer a fill beat went to, traced as block_<b>/macro_<m>/load_kind.
enum dimc_load_kind_t : uint8_t {
    DIMC_LOAD_NONE = 0,
    DIMC_LOAD_KB   = 1,               // weights
    DIMC_LOAD_FB   = 2,               // inputs
    DIMC_LOAD_PSIN = 3,
    // A kernel section went into the macro that cycle while partial sums went into its ADDIN
    // sets (DIMC_LOAD_KB_FB: with a feature section, no longer produced).
    DIMC_LOAD_KB_FB   = 4,
    DIMC_LOAD_KB_PSIN = 5
};
// outer_port/use: a write-back beat, and the packing of one cycle's beats. Two bits per kind
// (0..2 beats of 32 B): kernel in bits 1:0, feature 3:2, partial sums 5:4, write-back 7:6.
#define DIMC_PORT_WB 6

// Why the engine has no job, traced as idle_why.
enum dimc_idle_why_t : uint8_t {
    DIMC_IDLE_WHY_BUSY        = 0,
    DIMC_IDLE_WHY_WAIT_COMMIT = 1,    // nothing queued: waiting for software
    DIMC_IDLE_WHY_START       = 2     // a job is queued, its start event is one cycle away
};

// Everything the DIMC model reports and never acts on: the VCD events, the
// activity observed in each cycle, the wait causes, per-job timestamps and the
// accounting printed when a job closes.
//
// The engine calls a hook where it has decided something and passes what it
// decided. The tracer keeps its own copy of what it needs and reads engine
// state through a const reference, so it cannot change what the engine does;
// it never schedules an event, so tracing costs no simulated cycle.
class Dimc_Tracer {
    public:
        explicit Dimc_Tracer(const Dimc_HWPE &dimc);

        // Size the per-block state and register every VCD event on `owner`.
        // Called once the engine has sized its inner blocks: events are
        // registered by address, so nothing may resize them afterwards.
        void build(vp::Component &owner, vp::Trace &text,
                   uint32_t nb_blocks, uint32_t nb_macros);
        // Time-zero values, so no track starts part-way in.
        void reset();

        // ---- hooks ----
        void commit(uint32_t job_id);
        void job_queued();                        // start event scheduled
        void job_start(uint64_t now_cycle);
        void state(uint32_t next_state);          // an FSM transition
        void fill_beat(uint32_t blk, uint32_t macro, uint32_t within,
                       bool macro_filled, bool feature_done, int lat);
        void fill_skip(uint32_t blk, uint8_t why);
        // The kernel feed was refused (DIMC_WHY_WAIT_DEPTH / DIMC_WHY_WAIT_OUTER_PORT).
        void kernel_skip(uint32_t blk, uint8_t why);
        // The input feed was refused (DIMC_WHY_WAIT_DEPTH / DIMC_WHY_WAIT_OUTER_PORT).
        void input_skip(uint32_t blk, uint8_t why);
        void row_issued(uint32_t blk, uint32_t macro, uint32_t rows_issued,
                        uint32_t row_count);
        void store_beat(uint32_t blk, uint32_t macro, int lat, uint32_t out_beats);
        void store_skip(uint32_t blk, uint8_t why);
        // `who` is the inner block that got the outer port this cycle; `kind` is what the
        // beat carries: DIMC_LOAD_KB / _FB / _PSIN for a fetch, DIMC_PORT_WB for a write-back.
        void outer_port_booked(uint32_t who, uint8_t kind);
        // Publish one block's levels and wait causes for this cycle, then clear.
        void end_cycle(uint32_t blk);
        // Publish what the outer port carried this cycle, then clear: after the cycle's
        // last booking, the next jobs' kernel prefetch included.
        void port_cycle();
        void job_closed(uint64_t now_cycle);      // the per-job report
        void job_end();                           // busy falls

    private:
        uint8_t macro_why(uint32_t blk, uint32_t m) const;

        const Dimc_HWPE &dimc;
        vp::Trace *text = nullptr;
        uint32_t cur_state = 0;

        struct Macro {
            vp::Trace load_event, comp_event, wb_event, why_event, load_kind_event;
            uint8_t   loaded = 0, computed = 0, wrote_back = 0;
            uint8_t   load_kind = DIMC_LOAD_NONE;   // which buffer this cycle's beat filled
            // Job-relative cycles, for the report.
            uint32_t  fill_start = 0, fill_done = 0;
            uint32_t  compute_start = 0, compute_end = 0;
            uint32_t  load_done = 0;
        };
        struct Block {
            vp::Trace beat_event;     // fill beat_index
            vp::Trace rows_event;     // macro 0's rows_issued
            vp::Trace load_event, comp_event, wb_event;   // OR of the macros
            bool      loaded = false, computed = false, wrote_back = false;
        uint8_t   fill_grant = DIMC_GRANT_NONE;  // macro a FIFO section was written into this cycle
        vp::Trace fill_grant_event;
            // Why this cycle's fill or store beat did not go, 0 when it went or
            // was not due.
            uint8_t   fill_skip = 0, store_skip = 0;
            uint8_t   kernel_skip = 0;          // this cycle's kernel-feed refusal
            uint8_t   input_skip = 0;           // this cycle's input-feed refusal
            uint64_t  kernel_port_refused = ~0ull;   // DIMC cycle the kernel feed last lost the outer port
            // Last store beat's latency times the beat count; report only.
            uint32_t  out_beat_lat_est = 0;
            std::vector<Macro>   macros;
            std::vector<uint8_t> why;    // scratch for end_cycle
        };
        std::vector<Block> blocks;

        vp::Trace state_event;       // FSM state, one byte
        vp::Trace busy_event;        // 1 while a job runs
        vp::Trace job_event;         // id of the running job
        vp::Trace commit_event;      // id of each job as software commits it
        vp::Trace idle_why_event;    // dimc_idle_why_t
        vp::Trace outer_grant_event; // which inner block booked the outer port this cycle, 255 = none
        uint8_t   outer_grant = DIMC_GRANT_NONE;
        vp::Trace next_free_event;   // outer port: first free cycle after a booking
        vp::Trace port_use_event;    // what the outer port carried this cycle (DIMC_PORT_WB)
        uint8_t   port_use = 0;

        // ---- accounting ----
        // fsm_timestamp counts engine cycles and cannot see the gaps between
        // jobs; now_cycle is simulated time and reconciles with the core's
        // mcycle.
        uint64_t job_entry_cycle  = 0;
        uint64_t last_job_end     = 0;
        uint64_t acc_busy_cycles  = 0;
        uint64_t acc_gap_cycles   = 0;
        uint64_t acc_ideal_cycles = 0;
        // Sections written from the dual's FIFOs into the macros. The hook is passed
        // latency 1, so acc_beat_lat equals acc_beats; store beats are not counted.
        uint64_t acc_beats = 0, acc_beat_lat = 0, acc_stall_full = 0;
        uint32_t jobs_measured    = 0;
};

#endif
