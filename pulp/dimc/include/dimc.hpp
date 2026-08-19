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
        void reset();

        int64_t  next_free_cycle;   // <- the state interco/router calls next_burst_cycle
        uint32_t bandwidth_bytes;
        uint32_t burst_latency;

        // VCD event, registered by the parent as outer_port_<i>/next_free.
        // Rising steps in it are exactly the contention this port models.
        vp::Trace free_event;
};

// ---- rtl/accumulator.sv, instantiated by rtl/cleopatra.sv ----
// Sums every result popped from the block's output FIFO into one register.
// The FIFO is 24 bits wide, so each value wraps to 24 bits before being added.
class Dimc_OutAccum {
    public:
        void push(int32_t psout)
        {
            if (!this->enable) return;
            this->acc += Dimc_OutAccum::to_int24(psout);
            this->count++;
        }

        void clear()
        {
            this->acc = 0;
            this->count = 0;
        }

        // 24-bit FIFO word, sign-extended back to the host's int32.
        static int32_t to_int24(int32_t v)
        {
            int32_t masked = v & 0xFFFFFF;
            return (masked & 0x800000) ? (masked - 0x1000000) : masked;
        }

        int32_t  acc    = 0;   // acc_q
        uint32_t count  = 0;   // values summed, trace only
        uint8_t  enable = 0;   // acc_enable_i
};

// ---- Inner block: num_macros macros on one inner (L1) port ----
// A plain class, not a vp::Component. One control plane drives N of these.
class Dimc_InnerBlock {
    public:
        // Data path: this block's own three streamers and its own macros.
        Dimc_HWPE_Streamer weight_stream;
        Dimc_HWPE_Streamer input_stream;
        Dimc_HWPE_Streamer out_stream;
        Dimc_HWPE_Streamer psin_stream;   // per-row partial sums in, when PSIN_EN
        std::vector<Dimc_Macro> macros;

        // Cycle-accurate cursor. ONE linear beat index is the only position
        // state; (macro, row, offset) are decoded from it on demand, so they
        // cannot drift apart across cycles.
        std::queue<uint64_t> pending_req_queue;  // cycle at which each beat returns
        uint32_t beat_index;                     // linear position in the phase
        uint32_t beat_total;                     // beats this phase must issue
        uint32_t rows_issued;                    // compute: rows pushed into pipes
        uint8_t  row_buffer[DIMC_MACRO_KB_EW];   // accumulates the row in flight

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
        uint32_t out_lat;

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
            this->out_lat = 0;
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

        // ---- Dual context (standard HWPE job queue) ----
        // The job-dependent registers are banked DIMC_NB_CONTEXT times, so a
        // second job can be offloaded while the first runs. The engine still
        // runs one job at a time, so the per-block state and the scratch below
        // are not banked: they hold the running job only.
        uint32_t ctx_regs[DIMC_NB_CONTEXT][DIMC_HWPE_NB_JOB_REGS];
        bool     ctx_busy[DIMC_NB_CONTEXT];    // acquired or committed, not yet retired
        uint32_t ctx_job_id[DIMC_NB_CONTEXT];  // job id stamped at commit
        int      acquired_ctx;                 // context SW is currently filling (-1 none)
        int      running_ctx;                  // context the engine executes (-1 none)
        int      next_ctx;                  // committed and waiting to run (-1 none)
        int      spare_ctx;                   // 2nd committed job behind pending (-1 none)

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
        uint32_t job_num_active, job_row_count, job_row_base, job_compute_cyc;
        int32_t  job_bias;
        bool     job_skip_kb;

        // Per-block step functions. Each advances ONE inner block by one cycle's
        // worth of work and returns true when that block finished the phase; the
        // phase wrapper owns fsm_timestamp and ends only when all blocks are done.
        bool preload_block(Dimc_InnerBlock &blk);
        bool compute_block(Dimc_InnerBlock &blk);
        bool store_block(Dimc_InnerBlock &blk);
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
        uint64_t fsm_timestamp;                  // cycles elapsed inside this job
        // Derived per-job beat geometry (same for every block, computed once when
        // the phase starts).
        uint32_t psin_beats_per_macro;   // 0 while PSIN_EN is clear
        uint32_t job_psin_rows;          // latched PSIN_EN for the running job
        uint32_t kb_beats_per_row;               // beats to move one kernel row
        uint32_t fb_beats_per_macro;             // beats to move one feature row
        uint32_t beats_per_macro;                // kb rows*kb_beats + fb_beats
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
