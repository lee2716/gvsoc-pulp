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

#ifndef __DIMC_HPP__
#define __DIMC_HPP__

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>
#include <cstdint>
#include <vector>
#include <queue>
#include <deque>

#include <dimc_hwpe_archi.hpp>
#include <dimc_macro.hpp>

typedef uint64_t strobe_t;

// A job is offloaded with the acquire/commit protocol, then the FSM runs
// preload -> compute -> store.
enum dimc_hwpe_state_t {
    DIMC_IDLE,
    DIMC_STARTING,
    DIMC_COMPUTING,
    DIMC_STORING,
    DIMC_FINISHED
};

class Dimc_HWPE;

class Dimc_HWPE_Streamer {
    public:
        Dimc_HWPE_Streamer(Dimc_HWPE* dimc, bool is_write);
        Dimc_HWPE_Streamer();
        void configure(
                uint32_t base_addr,
                uint32_t tot_len,
                uint32_t d0_len,
                uint32_t d0_stride,
                uint32_t d1_len,
                uint32_t d1_stride,
                uint32_t d2_len,
                uint32_t d2_stride,
                uint32_t d3_stride
        );
        bool is_done();
        // Issue ONE beat (inner_port_bytes wide) and return the cycle count after
        // which its response is due. The caller keeps the timestamp in a pending
        // queue instead of blocking, so several beats can be in flight at once.
        int issue_beat(int width, void* buf);
        // Address of the next beat, linear or strided; see the .cpp.
        uint32_t walk_addr() const;

    private:
        Dimc_HWPE*  dimc;
        vp::IoReq*  req;

        uint32_t    pos;
        uint32_t    tot_iters;

        uint32_t    base_addr;
        uint32_t    tot_len;
        uint32_t    d0_len;
        uint32_t    d0_stride;
        uint32_t    d1_len;
        uint32_t    d1_stride;
        uint32_t    d2_len;
        uint32_t    d2_stride;
        uint32_t    d3_stride;
        bool        is_write;
};


// ---- rtl/accumulator.sv, instantiated by rtl/cleopatra.sv ----
// Sums every result popped from the block's output FIFO into one register.
// The FIFO is 32 bits wide, the same as PSOUT, so nothing is truncated.
class Dimc_OutAccum {
    public:
        void push(int32_t psout)
        {
            if (!this->enable) return;
            this->acc += psout;
            this->count++;
        }

        void clear()
        {
            this->acc = 0;
            this->count = 0;
        }

        int32_t  acc    = 0;   // acc_q
        uint32_t count  = 0;   // values summed, trace only
        uint8_t  enable = 0;   // acc_enable_i
};

// ---- Inner block: num_macros macros on one inner (L1) port ----
// A plain class, not a vp::Component. One control plane drives N of these.
class Dimc_InnerBlock {
    public:
        // One set of streamers per macro, not per block. A block-wide streamer
        // walks its macros' data back to back, which is only correct while every
        // macro of the block is on the same job; giving each macro its own
        // address generator is what lets them run different jobs at once.
        std::vector<Dimc_HWPE_Streamer> weight_stream;
        std::vector<Dimc_HWPE_Streamer> input_stream;
        std::vector<Dimc_HWPE_Streamer> out_stream;
        std::vector<Dimc_HWPE_Streamer> psin_stream;   // per-row psums, when PSIN_EN
        std::vector<Dimc_Macro> macros;

        // One cursor per streaming activity. Fill and store run concurrently
        // and must not share position state: a cursor per activity keeps that
        // separation in the type rather than in a convention.
        //
        // ONE linear beat index is the only position state; (macro, row,
        // offset) are decoded from it on demand, so they cannot drift apart.
        struct Cursor {
            uint32_t beat_index = 0;    // linear position, whole block
            uint32_t beat_total = 0;
            std::vector<uint32_t> macro_beat_index;   // per macro
            std::vector<uint32_t> macro_beat_total;
            void reset(uint32_t nb_macros)
            {
                this->beat_index = 0;
                this->beat_total = 0;
                this->macro_beat_index.assign(nb_macros, 0);
                this->macro_beat_total.assign(nb_macros, 0);
            }
        };
        Cursor fill;        // the running job's own operands
        Cursor store;       // its results

        // Outstanding L1 requests. This one IS shared on purpose: it is a
        // property of the inner port, not of the activity using it, so fill and
        // store contend for the same budget.
        std::queue<uint64_t> port_pending;

        uint32_t rows_issued;                    // compute: rows pushed into pipes

        // Per-phase completion, so the phase ends only when EVERY block is done.
        bool phase_done;

        // Results and per-block reporting.
        std::vector<std::vector<uint8_t>> out_buf;
        std::vector<uint32_t> load_done;   // per-macro L1-load completion cycle
        // Last store beat's latency times the beat count: an estimate, and it
        // only ever reaches a trace line. Not a measured output latency.
        uint32_t out_beat_lat_est;

        // Accumulates across jobs, so reset_job_state() must not touch it.
        Dimc_OutAccum out_accum;

        // VCD events for this block, registered by the parent as
        // block_<i>/<leaf>. Cycle values are fsm_timestamp, not simulated time.
        vp::Trace beat_event;    // beat_index, the linear cursor of the phase
        vp::Trace rows_event;    // rows_issued during COMPUTING
        // High for the cycles the block is moving operands in, and for the
        // cycles it is issuing compute rows. Counters cannot show that the two
        // run at once; these two levels overlap in the waveform exactly when
        // the load of one macro is hidden under the compute of another.
        vp::Trace load_active_event;
        vp::Trace comp_active_event;
        bool loaded_this_cycle  = false;   // a fill beat issued
        bool computed_this_cycle = false;  // a compute row issued

        // Clear everything the engine tracks for one job. Called from the
        // constructor, from reset(), and at every job start, so the three sites
        // cannot drift apart.
        void reset_job_state()
        {
            this->fill.reset(this->macros.size());
            this->store.reset(this->macros.size());
            this->rows_issued = 0;
            this->phase_done = false;
            this->out_beat_lat_est = 0;
            this->load_done.clear();
            while (!this->port_pending.empty()) this->port_pending.pop();
        }
};

class Dimc_HWPE : public vp::Component {
    public:
        Dimc_HWPE(vp::ComponentConf &config);

        void reset(bool active);

        // HWPE RF
        uint32_t register_file[N_CFG_REGS];

        // Streamer master port (data path to L1/TCDM)
        vp::IoMaster stream_mst;

        // HWPE slave port (memory-mapped register interface)
        vp::IoSlave hwpe_slv;

        // Completion interrupt line (standard HWPE done_irq)
        vp::WireMaster<bool> irq;

        // ---- Inner blocks (nb_inner_blocks of them) ----
        // Each owns its streamers and macros. nb_inner_blocks == 1 is a single
        // block on the inner port, with no outer port.
        std::vector<Dimc_InnerBlock> inner_blocks;

        uint8_t sel_dimc;

        // Configuration
        uint32_t num_macros;
        // Streamer bandwidth: fixed hardware properties, set once from the
        // systree / gvrun --param (no per-trigger MMIO override).
        uint32_t inner_port_bytes;    // inner (L1) port bytes/cycle, e.g. 32 banks*4=128
        // ---- Outer block ----
        // An inner block is num_macros macros on one inner port; an outer block
        // is nb_inner_blocks of them. Every block reaches memory through its own
        // inner port, and the latency of an access is whatever the L1 bank and
        // the crossbar return -- the accelerator adds none of its own, which is
        // how magia_v2 models RedMulE.
        uint32_t nb_inner_blocks;     // inner blocks in the outer block (D-tile default 2)
        // Reuse auto-detect: the KB (weight) source address of the last loaded
        // job. A trigger whose KB address matches reuses the resident weights
        // (skips that load), like a real weight cache. 0xFFFFFFFF = none yet.
        uint32_t last_kb_src;

        // ---- Job accounting ----
        // Reported per job the way magia_v2's LightRedmule reports a GEMM
        // (light_redmule.cpp:1409): absolute start and end so the gap to the
        // next job reads straight off consecutive lines, the period in cycles,
        // and a utilisation against an analytic ideal.
        //
        // Two clocks, measuring different things. fsm_timestamp advances only
        // inside the phase iterators, so it counts the cycles the engine
        // worked and cannot see the gaps between jobs. clock.get_cycles() is
        // simulated time, keeps running while the engine is idle, and is what
        // reconciles with the core's own mcycle.
        //
        // The FSM phases are not a partition of the work: preload_iter drives
        // the fill, the compute and the store in one loop, so a job whose
        // compute hides entirely under its fill spends no cycles in the
        // COMPUTING state. busy_cycles is what an outside observer measures;
        // the beat counters say what filled it.
        uint64_t phase_entry_ts  = 0;    // fsm_timestamp at the current phase's start
        uint64_t job_entry_cycle = 0;    // simulated cycle at this job's start
        uint64_t last_job_end    = 0;    // simulated cycle the previous job ended
        uint64_t acc_starting = 0, acc_computing = 0, acc_storing = 0;
        uint64_t acc_busy_cycles = 0;    // simulated cycles inside a job
        uint64_t acc_gap_cycles  = 0;    // simulated cycles between jobs
        uint64_t acc_ideal_cycles = 0;   // analytic minimum for those jobs
        // Beats the inner port issued and the latency L1 returned for them.
        // Their average is the per-beat cost the phase lengths are built from.
        uint64_t acc_beats = 0, acc_beat_lat = 0, acc_stall_full = 0;
        uint32_t jobs_measured   = 0;

        // Traces
        vp::Trace trace;

        // VCD event traces for waveform and Perfetto profiling. Written from the
        // FSM handlers; an event costs no cycle. Mark a value stale by writing
        // the next one -- event_highz() is dropped by the Perfetto converter.
        vp::Trace state_event;   // three-phase FSM, one byte
        vp::Trace busy_event;    // 1 while a job runs, drawn as one Perfetto slice
        vp::Trace job_event;     // id of the running job, to line up with software

        // Internal state
        vp::reg_32 state;

        // ---- Standard HWPE offload/context bookkeeping ----
        uint32_t running_job;   // id of the job currently executing (RUNNING_JOB reg)
        uint32_t next_job_id;   // id handed out by the next ACQUIRE
        uint32_t finished_jobs; // count of completed jobs (FINISHED reg)
        bool     job_running;   // the engine is executing a committed job

        // ---- Job queue (standard HWPE offload) ----
        // Several jobs can be offloaded while one runs. The engine still runs
        // one at a time, so the per-block state and the scratch below hold the
        // running job only.
        // hwpe_ctrl_target.sv (a52dc9cd): there is ONE live bundle of
        // job-dependent registers, and COMMIT snapshots it into a job FIFO of
        // depth NB_CONTEXT. Software cannot address a queue slot at all -- and
        // must not have to: ACQUIRE hands back a job id, never a slot index.
        // ctx_regs is that FIFO's storage; live_regs is what software writes.
        uint32_t live_regs[DIMC_HWPE_NB_JOB_REGS];
        uint32_t ctx_regs[DIMC_NB_CONTEXT][DIMC_HWPE_NB_JOB_REGS];
        bool     ctx_busy[DIMC_NB_CONTEXT];    // acquired or committed, not yet retired
        uint32_t ctx_job_id[DIMC_NB_CONTEXT];  // job id stamped at commit
        int      acquired_ctx;                 // context SW is currently filling (-1 none)
        int      running_ctx;                  // context the engine executes (-1 none)
        // Committed-but-not-yet-running contexts, in commit order. A real FIFO
        // rather than the two scalars this used to be: with DIMC_NB_CONTEXT
        // above 2 the software can queue several jobs ahead, and the pair could
        // only ever hold one plus a spare.
        std::deque<int> ctx_queue;

        int      ctx_alloc();                  // reserve a free context, -1 if none
        uint32_t job_reg(uint32_t addr) const; // read a job-dep reg of the RUNNING ctx
        void     start_next_job();             // launch the pending context, if any

        // ---- Engine phase iterators ----
        // Called once per cycle. Each sets *latency=1 and returns true when its
        // phase is done. The makespan is not returned: it accrues in
        // fsm_timestamp and is read from the trace.
        //   preload_iter : real per-beat load of every macro's KB + FB
        //   compute_iter : run each macro's matvec through its 4-deep pipeline
        //   store_iter   : real per-beat output drain, then report fsm_timestamp
        bool preload_iter(int *latency);
        bool compute_iter(int *latency);
        bool store_iter(int *latency);

        // Scratch shared across the three phases of one job (same for all blocks:
        // one control plane issues one job shape to every inner block).
        // Everything derived once per job. One per context, so two jobs can be
        // in flight and each macro reads the one it is actually working on.
        struct JobGeom {
            uint32_t num_active, row_count, row_base, compute_cyc;
            int32_t  bias;
            bool     skip_kb;
        uint32_t kb_src;      // stamped onto the macros only once the fill lands
            uint32_t psin_rows;
            uint32_t beats_per_macro, kb_beats_per_row;
            uint32_t fb_beats_per_macro, psin_beats_per_macro, out_beats;
        };
        JobGeom job_geom[DIMC_NB_CONTEXT];
        // Which slot each pipeline stage reads. They are the same index while
        // one job is in flight; splitting them is what lets the streamers fill
        // job k+1's banks while the macros still compute job k.
        uint32_t fill_slot = 0;
        uint32_t exec_slot = 0;
        // hwpe-ctrl keeps one context running while software prepares the
        // other. This is the data-path half of that: once the running job stops
        // needing the streamers, the queued context's kernels and features are
        // pulled into the macros' spare banks, so when it starts there is
        // nothing left to load.
        // Per-block step functions. Each advances ONE inner block by one cycle's
        // worth of work and returns true when that block finished the phase; the
        // phase wrapper owns fsm_timestamp and ends only when all blocks are done.
        // The clamped (num_active, row_count) pair for the committed job.
        // fsm_start_handler needs it to slice the descriptor between blocks and
        // preload_iter to size the phase; the two must agree, or the streamer
        // walks a different number of bytes than the FSM issues beats for.
        void job_shape(uint32_t *num_active, uint32_t *row_count) const;

        // Clear the per-phase cursors on every block. Called at the end of each
        // phase, so the next one starts from a known state.
        void phase_end_reset();

        // Common tail of a preload or store beat: charge the access latency,
        // advance the cursor, publish it.
        void beat_issued(Dimc_InnerBlock &blk, Dimc_InnerBlock::Cursor &cursor,
                         int lat);

        void preload_block(Dimc_InnerBlock &blk, uint32_t blk_id,
                           Dimc_InnerBlock::Cursor &cursor);
        void plan_fill(int ctx);
        // Latch one context's job shape into its geometry slot.
        void latch_geom(int ctx);
        uint32_t job_reg_ctx(int ctx, uint32_t addr) const;
        // Advance every macro of a block that has finished its own fill.
        void compute_indep(Dimc_InnerBlock &blk);
        void store_block(Dimc_InnerBlock &blk, uint32_t blk_id);
        // Move every row a macro has finished into that macro's output buffer.
        // compute_indep needs this both at the top of a cycle and once more
        // when the last row retires, so it lives in one place.
        void drain_ready_rows(Dimc_InnerBlock &blk);
        // Emit this cycle's load and compute levels for one block, then clear
        // them. Called once per block per cycle so the two traces are levels
        // rather than one-cycle spikes.
        void publish_activity(Dimc_InnerBlock &blk);

        // ---- Cycle-accurate engine ----
        // One cycle per fsm_event. TCDM accesses are async: on issue we record
        // when the response is due, and each cycle we retire the beats that
        // came back. outstanding_depth caps in-flight requests, so slow memory
        // back-pressures the engine.
        //
        // The load walks (macro, row, offset). Keeping three counters in sync
        // broke the first attempt, so the position is one linear beat index,
        // decoded on demand.
        uint32_t outstanding_depth;              // max in-flight TCDM beats per block
        uint64_t fsm_timestamp;                  // free-running engine cycle count
        uint64_t job_start_cycle;                // fsm_timestamp when this job began
        // Derived per-job beat geometry (same for every block, computed once when
        // the phase starts).
        bool     phase_planned;                  // geometry latched for this phase

    private:
        static vp::IoReqStatus hwpe_slave(vp::Block *__this, vp::IoReq *req);

        static void fsm_start_handler(vp::Block *__this, vp::ClockEvent *event);
        static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
        static void fsm_end_handler(vp::Block *__this, vp::ClockEvent *event);

        void fsm_loop();
        int  fsm();

        vp::ClockEvent *fsm_start_event;
        vp::ClockEvent *fsm_event;
        vp::ClockEvent *fsm_end_event;
};

#endif
