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
    DIMC_WHY_LOAD             = 1,
    DIMC_WHY_COMPUTE          = 2,
    DIMC_WHY_WRITE_BACK       = 3,
    DIMC_WHY_WAIT_INNER_PORT  = 4,    // a lower-index macro of the block takes the fill beat
    DIMC_WHY_WAIT_OUTER_PORT  = 5,    // this macro's beat was due, the shared outer port was booked
    DIMC_WHY_WAIT_DEPTH       = 6,    // the block had outstanding_depth beats in flight
    DIMC_WHY_WAIT_FILL_ACK    = 7,    // last fill beat issued, its response not in yet
    DIMC_WHY_PIPE_DRAIN       = 8,    // rows issued, results still inside the macro pipeline
    DIMC_WHY_WAIT_STORE_ORDER = 9,    // results ready, the block writes an earlier macro back first
    DIMC_WHY_WAIT_BEAT_ACK    = 10,   // every beat issued, a response still in flight
    DIMC_WHY_DONE_WAIT_JOB    = 11,   // this macro is finished, the job waits for another macro or block
    DIMC_WHY_FSM_STEP         = 12,   // a phase cycle with no work
    DIMC_WHY_UNKNOWN          = 255   // must never appear
};

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
        void row_issued(uint32_t blk, uint32_t macro, uint32_t rows_issued,
                        uint32_t row_count);
        void store_beat(uint32_t blk, uint32_t macro, int lat, uint32_t out_beats);
        void store_skip(uint32_t blk, uint8_t why);
        void outer_port_booked();
        // Publish one block's levels and wait causes for this cycle, then clear.
        void end_cycle(uint32_t blk);
        void job_closed(uint64_t now_cycle);      // the per-job report
        void job_end();                           // busy falls

    private:
        uint8_t macro_why(uint32_t blk, uint32_t m) const;

        const Dimc_HWPE &dimc;
        vp::Trace *text = nullptr;
        uint32_t cur_state = 0;

        struct Macro {
            vp::Trace load_event, comp_event, wb_event, why_event;
            uint8_t   loaded = 0, computed = 0, wrote_back = 0;
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
            // Why this cycle's fill or store beat did not go, 0 when it went or
            // was not due.
            uint8_t   fill_skip = 0, store_skip = 0;
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
        vp::Trace next_free_event;   // outer port: first free cycle after a booking

        // ---- accounting ----
        // fsm_timestamp counts engine cycles and cannot see the gaps between
        // jobs; now_cycle is simulated time and reconciles with the core's
        // mcycle.
        uint64_t job_entry_cycle  = 0;
        uint64_t last_job_end     = 0;
        uint64_t acc_busy_cycles  = 0;
        uint64_t acc_gap_cycles   = 0;
        uint64_t acc_ideal_cycles = 0;
        // Fill beats and the latency L1 returned for them; store beats are
        // not counted.
        uint64_t acc_beats = 0, acc_beat_lat = 0, acc_stall_full = 0;
        uint32_t jobs_measured    = 0;
};

#endif
