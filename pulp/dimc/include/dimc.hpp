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

// ---- Outer port: shared L2 port with temporal state ----
// Same idiom as interco/router's BandwidthLimiter: the port remembers when it
// is free again, so a client arriving before that is delayed.
// Serialization on a shared port emerges from this, it is not computed.
class Dimc_OuterPort {
    public:
        // `bandwidth` in bytes/cycle, `latency` = fixed per-burst round trip.
        void configure(uint32_t bandwidth, uint32_t latency);
        // Reserve the port for `bytes` starting no earlier than `now`; returns
        // the cycle at which the transfer has landed.
        int64_t request(int64_t now, uint64_t bytes);
        // First cycle at which the port can start new work. Byte-granular, so a
        // 32-byte beat on a 64 B/cycle port occupies half a cycle and two of
        // them share one -- rounding each request up to a whole cycle would
        // halve the port whenever a beat is narrower than it.
        int64_t busy_until() const;
        void reset();

        int64_t  cursor_bytes;      // bytes committed; /bandwidth gives the cycle
        int64_t  next_free_cycle;   // <- the state interco/router calls next_burst_cycle
        uint32_t bandwidth_bytes;
        uint32_t burst_latency;

        // VCD event, registered by the parent as outer_port_<i>/next_free.
        // Rising steps in it are exactly the contention this port models.
        vp::Trace free_event;
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

        // Cycle-accurate cursor. ONE linear beat index is the only position
        // state; (macro, row, offset) are decoded from it on demand, so they
        // cannot drift apart across cycles.
        std::queue<uint64_t> pending_req_queue;  // cycle at which each beat returns
        uint32_t beat_index;                     // linear position in the phase
        uint32_t beat_total;                     // beats this phase must issue
        // The prefetch runs concurrently with the current job's STORING, and
        // STORING's completion test is beat_index >= beat_total. A prefetch
        // sharing those would push the store past its own finish line and the
        // remaining output beats would never issue. It gets its own cursor.
        int64_t  prefetch_data_ready_cycle = 0;        // ditto: STORING's drain test reads
                                                 // data_ready_cycle, so the prefetch
                                                 // must not write it
        // The store gets its own cursor too. beat_index belongs to the fill,
        // and store beats now issue while the fill's phase is still current.
        uint32_t store_beat_index = 0;
        uint32_t store_beat_total = 0;
        uint32_t prefetch_beat_index = 0;
        uint32_t prefetch_beat_total = 0;
        std::queue<uint64_t> prefetch_pending;
        uint32_t rows_issued;                    // compute: rows pushed into pipes

        // Per-phase completion, so the phase ends only when EVERY block is done.
        bool phase_done;

        // Cycle at which this block's outer-port transfer has landed: the fill
        // during STARTING, the result drain during STORING. The block cannot
        // touch its inner port before it.
        int64_t data_ready_cycle;
        bool    fill_requested;

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
        vp::Trace ready_event;   // data_ready_cycle, when the outer fill lands
        vp::Trace drain_event;   // data_ready_cycle, when the outer drain lands

        // Clear everything the engine tracks for one job. Called from the
        // constructor, from reset(), and at every job start, so the three sites
        // cannot drift apart.
        void reset_job_state()
        {
            this->beat_index = 0;
            this->beat_total = 0;
            this->rows_issued = 0;
            this->phase_done = false;
            this->fill_requested = false;
            this->data_ready_cycle = 0;
            this->out_beat_lat_est = 0;
            this->load_done.clear();
            while (!this->pending_req_queue.empty()) this->pending_req_queue.pop();
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

        // ---- Outer ports ----
        // shared=1: one port, all blocks contend on it (fills serialize via
        // next_free_cycle). shared=0: one port per block, fills run in parallel.
        // A topology choice, not a formula branch.
        std::vector<Dimc_OuterPort> outer_ports;
        Dimc_OuterPort *block_port(uint32_t blk);
        // Bytes one inner block pulls through the outer port for this job.
        uint64_t block_working_set() const;

        uint8_t sel_dimc;

        // Configuration
        uint32_t num_macros;
        // Streamer bandwidth: fixed hardware properties, set once from the
        // systree / gvrun --param (no per-trigger MMIO override).
        uint32_t inner_port_bytes;    // inner (L1) port bytes/cycle, e.g. 32 banks*4=128
        uint32_t port_sync_cycles;          // per-beat wrapper sync cycle (0/1)
        uint32_t tcdm_burst_latency;  // round-trip latency per streamer burst (bandwidth-independent)
        // ---- Outer block ----
        // An inner block is num_macros macros on one inner port. An outer block
        // is nb_inner_blocks of them, each a real Dimc_InnerBlock reaching L2
        // through a Dimc_OuterPort. With a shared port, block b+1's fill waits
        // for block b's, so it lands later.
        // nb_inner_blocks==1 creates no port and matches a lone inner block.
        uint32_t nb_inner_blocks;     // inner blocks in the outer block (D-tile default 2)
        uint32_t outer_port_shared;     // 1 = all blocks contend on ONE L2 port (fills serialize);
                                       // 0 = one L2 port per block (fills proceed in parallel)
        uint32_t outer_port_bytes;         // L2 port bytes/cycle (0 sentinel => same as inner_port_bytes)
        uint32_t l2_burst_latency;     // Dimc_OuterPort::burst_latency
        // Reuse auto-detect: the KB (weight) source address of the last loaded
        // job. A trigger whose KB address matches reuses the resident weights
        // (skips that load), like a real weight cache. 0xFFFFFFFF = none yet.
        bool     cross_job_prefetch;
        // Outer-port admission. Store-and-forward (0): a block issues no inner
        // beat until its whole working set has landed. Cut-through (1): each
        // beat is admitted as soon as the port has the bandwidth for it, so two
        // blocks stream together and saturate the port instead of taking turns.
        bool     outer_cut_through;
        uint32_t last_kb_src;

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
        int      queue_head() const { return this->ctx_queue.empty() ? -1
                                            : this->ctx_queue.front(); }

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
        int  prefetch_ctx   = -1;
        bool prefetch_ready = false;
        bool job_prefetched = false;   // the running job arrived pre-filled

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
        void beat_issued(Dimc_InnerBlock &blk, int lat, bool prefetch);

        void preload_block(Dimc_InnerBlock &blk, uint32_t blk_id, bool prefetch);
        bool compute_block(Dimc_InnerBlock &blk);
        void background_fill();
        void plan_fill(int ctx);
        void configure_fill_streams(int ctx);
        // Latch one context's job shape into its geometry slot.
        void latch_geom(int ctx);
        uint32_t job_reg_ctx(int ctx, uint32_t addr) const;
        // Advance every macro of a block that has finished its own fill.
        void compute_indep(Dimc_InnerBlock &blk);
        void store_block(Dimc_InnerBlock &blk, uint32_t blk_id);
        // Move every row a macro has finished into that macro's output buffer.
        // compute_block needs this both at the top of a cycle and once more when
        // the last row retires, so it lives in one place.
        void drain_ready_rows(Dimc_InnerBlock &blk);

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
