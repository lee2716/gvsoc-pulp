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
#include <dimc_tracer.hpp>

typedef uint64_t strobe_t;

// A job is offloaded with the acquire/commit protocol, then the FSM runs
// STARTING (fill, compute and write-back together) and, only while results are
// still unwritten once every row has retired, STORING. Value 2 is unused.
enum dimc_hwpe_state_t {
    DIMC_IDLE     = 0,
    DIMC_STARTING = 1,
    DIMC_STORING  = 3,
    DIMC_FINISHED = 4
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
        // Issue one beat of at most inner_port_bytes and return the cycles until its
        // response; the caller queues the due cycle, so several beats can be in flight.
        // A read at a 64 B-aligned address fetches the whole 64 B word: the second
        // section is kept in `pair`, and the next beat, if it is that section, returns
        // NO_REQUEST without touching the memory.
        static const int NO_REQUEST = -1;
        int issue_beat(int width, void* buf);
        // The next beat is the kept second section: no request, no port.
        bool pair_ready() const;
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
        uint8_t     pair[32];
        uint32_t    pair_addr;
        bool        pair_valid;
};


// ---- Outer port: the tile's shared port towards memory ----
// A bandwidth limiter in the same idiom as interco/router's: the port remembers
// when it is free again, so a client arriving before that waits. Accounting is
// byte-granular; every request books the whole port word, so a port carries one
// request (one address) per cycle.
class Dimc_OuterPort {
    public:
        void    configure(uint32_t bandwidth);
        void    reset();
        // First cycle at which the port can start new work.
        int64_t busy_until() const;
        // Reserve `bytes` starting no earlier than `now`.
        void    request(int64_t now, uint64_t bytes);

        int64_t  cursor_bytes    = 0;   // bytes committed; /bandwidth gives the cycle
        uint32_t bandwidth_bytes = 1;
};

// ---- rtl/accumulator.sv, instantiated by rtl/cleopatra.sv ----
// One register summing every result that enters the block's out_fifo. cleopatra.sv has 256
// accumulators, cleopatra_ctrl.sv selecting the next one per out_fifo pop; not modelled.
// The out_fifo is 32 bits wide, the same as PSOUT, so nothing is truncated.
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
        // One streamer per operand and per macro, so the macros of a block can be on
        // different jobs. Fill and write-back book separate outer-port directions.
        std::vector<Dimc_HWPE_Streamer> weight_stream;
        std::vector<Dimc_HWPE_Streamer> input_stream;
        std::vector<Dimc_HWPE_Streamer> out_stream;
        std::vector<Dimc_HWPE_Streamer> psin_stream;   // per-row psums, when PSIN_EN
        std::vector<Dimc_Macro> macros;

        // Fill and store run concurrently, each with its own cursor. The store counts the
        // block's port words in beat_index/beat_total; the fill keeps each macro's position in
        // its own program in macro_beat_index/macro_beat_total.
        struct Cursor {
            uint32_t beat_index = 0;
            uint32_t beat_total = 0;
            std::vector<uint32_t> macro_beat_index;
            std::vector<uint32_t> macro_beat_total;
            // Some macro's program still owes sections.
            bool owed() const
            {
                for (size_t m = 0; m < this->macro_beat_index.size(); m++)
                    if (this->macro_beat_index[m] < this->macro_beat_total[m]) return true;
                return false;
            }
            void reset(uint32_t nb_macros)
            {
                this->beat_index = 0;
                this->beat_total = 0;
                this->macro_beat_index.assign(nb_macros, 0);
                this->macro_beat_total.assign(nb_macros, 0);
            }
        };
        Cursor fill;        // each macro's fill program, possibly for a later job
        Cursor store;       // the running job's results

        // Every outstanding L1 request of the block, fill and store; only the tracer reads it.
        std::queue<uint64_t> port_pending;
        // The running job's store beats: the job closes when these are acknowledged.
        std::queue<uint64_t> store_pending;
        // Per-feed budgets: kernel sections, and the input feed (feature and partial sums).
        std::queue<uint64_t> kb_pending;
        std::queue<uint64_t> in_pending;
        // The dual's weight and input FIFOs: sections fetched from L1, each bound for one
        // macro's program position, popped in order once landed (not fall-through: the
        // cycle after) and once that macro's write port takes it.
        struct FeedEntry {
            uint32_t macro, slot, job, within, bytes;
            uint64_t ready;
            uint8_t  data[32];
        };
        std::deque<FeedEntry> wgt_fifo;
        bool kb_fed_first = false;   // DIMC_KB_FEED_FIRST: this cycle's kernel feed already ran
        // The dual's one input FIFO: both macros' sections in fetch order.
        std::deque<FeedEntry> inp_fifo;
        // Per-macro out_fifo: results in push order. A port word leaves from the head
        // entries; out_results counts the running job's results that have left.
        // OutEntry::macro is always 0, never read.
        struct OutEntry { int32_t psout; uint16_t row; uint16_t run; uint16_t macro; };
        std::vector<std::deque<OutEntry>> out_fifo;
        std::vector<uint32_t> out_results;
        // Write-back position per macro: a macro's runs leave in vector order, and a run
        // counts once its last beat is acknowledged.
        struct RunDone { uint64_t due; uint32_t job, macro, run; };
        std::deque<RunDone> run_pending;
        struct Retired { int64_t job = -1; uint32_t runs = 0; };
        std::vector<Retired> retired;    // per macro

        // In DIMC_STORING: every result of the running job written and acknowledged, as of
        // the last cycle; the block then stops writing back.
        bool phase_done;

        // Accumulates across jobs, so reset_job_state() must not touch it.
        Dimc_OutAccum out_accum;

        // Clear what the engine tracks for one job, at every job start. keep_fill leaves the
        // fill cursor, the in-flight beats and the input FIFO alone: the fill runs ahead into
        // the next job. The weight FIFO may hold a held job's kernel and is not cleared here.
        void reset_job_state(bool keep_fill)
        {
            if (!keep_fill) {
                this->fill.reset(this->macros.size());
                this->inp_fifo.clear();
                while (!this->port_pending.empty()) this->port_pending.pop();
                while (!this->kb_pending.empty()) this->kb_pending.pop();
                while (!this->in_pending.empty()) this->in_pending.pop();
            }
            while (!this->store_pending.empty()) this->store_pending.pop();
            this->store.reset(this->macros.size());
            for (uint32_t &n : this->out_results) n = 0;
            this->phase_done = false;
        }
        // Construction and reset(): the weight FIFO too.
        void reset_all()
        {
            this->wgt_fifo.clear();
            this->reset_job_state(false);
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

        // Each owns its streamers and macros, and reaches L1 through its own inner port.
        std::vector<Dimc_InnerBlock> inner_blocks;

        // Configuration
        uint32_t num_macros;
        // Port bandwidths, fixed hardware properties from the systree.
        uint32_t inner_port_bytes;    // one inner block's port, bytes/cycle
        uint32_t outer_port_bytes;    // the tile's shared outer port, bytes/cycle
        // One outer-port budget per direction, each shared by all inner blocks: stream-in
        // (the three source streams) and stream-out (the sink) never block one another.
        Dimc_OuterPort outer_port_in;   // fills: feature, partial sums, kernel rows
        Dimc_OuterPort outer_port_out;  // write-back
        // An outer block is nb_inner_blocks inner blocks whose beats all pass through the shared
        // outer port. An access costs what L1 returns, plus the wait for that port.
        uint32_t nb_inner_blocks;     // inner blocks in the outer block (2 on the D-tile)

        // Text trace, for diagnostics and the tracer's reports.
        vp::Trace trace;

        // Every VCD signal and per-cycle observation; read-only towards the engine.
        Dimc_Tracer tracer;

        // Internal state
        vp::reg_32 state;

        // ---- Standard HWPE offload/context bookkeeping ----
        uint32_t running_job;   // id of the job currently executing (RUNNING_JOB reg)
        uint32_t next_job_id;   // id handed out by the next ACQUIRE
        uint32_t finished_jobs; // count of completed jobs (FINISHED reg)
        bool     job_running;   // the engine is executing a committed job

        // Job queue (hwpe_ctrl_target.sv): ONE live bundle of job-dependent registers, which
        // COMMIT snapshots into a job FIFO of depth NB_CONTEXT. ACQUIRE hands back a job id,
        // never a slot index. ctx_regs is that FIFO's storage; live_regs is what software writes.
        uint32_t live_regs[DIMC_HWPE_NB_JOB_REGS];
        uint32_t ctx_regs[DIMC_NB_CONTEXT][DIMC_HWPE_NB_JOB_REGS];
        bool     ctx_busy[DIMC_NB_CONTEXT];    // acquired or committed, not yet retired
        uint32_t ctx_job_id[DIMC_NB_CONTEXT];  // job id stamped at commit
        // CFG_CI, SIGN_8B and COMPUTE_MASK as they stood at commit. A macro takes them with
        // the job's operands, since it may trigger that job's rows while an earlier job
        // still runs.
        uint8_t  ctx_ci[DIMC_NB_CONTEXT];
        uint8_t  ctx_sign_8b[DIMC_NB_CONTEXT];
        uint16_t ctx_compute_mask[DIMC_NB_CONTEXT];
        int      acquired_ctx;                 // context SW is currently filling (-1 none)
        int      running_ctx;                  // context the engine executes (-1 none)
        // Committed-but-not-yet-running contexts, in commit order.
        std::deque<int> ctx_queue;

        int      ctx_alloc();                  // reserve a free context, -1 if none
        uint32_t job_reg(uint32_t addr) const; // read a job-dep reg of the RUNNING ctx
        void     start_next_job();             // launch the pending context, if any

        // One cycle of fill, compute and write-back on every block; `phase` is the FSM
        // state it runs for (DIMC_IDLE: the job-end cycle).
        void engine_cycle(uint32_t phase);
        // First cycle of a job: its geometry, fill programs and store.
        void plan_job();
        // Every row of the running job retired, on every block.
        bool rows_done() const;
        // Every result beat issued and acknowledged, on every block.
        bool store_done() const;

        // One job's shape, the same on every block; one per context, so a queued job's shape
        // survives until the engine reaches it.
        struct JobGeom {
            uint32_t num_active, row_count, row_base, compute_cyc;
            uint32_t psin_dep;    // PSIN_DEP: jobs back to the partial sums' producer
            uint32_t psin_rows;
            uint32_t beats_per_macro, kb_beats_per_row;
            uint32_t fb_beats_per_macro, psin_beats_per_macro, out_beats;
            // Batched descriptor: vectors per job and their strides (1 / 0 = one vector).
            uint32_t nb_vec, fb_vec_stride, ps_vec_stride, out_vec_stride;
            uint32_t kb_sections() const { return row_count * kb_beats_per_row; }
        };
        // The job id whose geometry job_geom[ctx] holds, so a job is latched once
        // even when its macros start their fill programs at different times.
        uint32_t geom_job[DIMC_NB_CONTEXT];
        // Where beat `within` of a macro's fill program falls: its kind, the run
        // (vector) it belongs to, and its index inside that segment.
        struct BeatPos { uint8_t kind; uint32_t run; uint32_t sub; };
        BeatPos  beat_pos(const JobGeom &g, uint32_t within) const;
        void     size_fill(int ctx);
        void     plan_macro_fill(uint32_t blk_id, uint32_t m, int ctx);
        void     configure_macro_streams(uint32_t blk_id, uint32_t m, int ctx, uint32_t run,
                                         bool kernel, bool feature, bool psum);
        void     configure_out_stream(uint32_t blk_id, uint32_t m, uint32_t run, uint32_t ctx);
        bool     psum_ready(const Dimc_InnerBlock &blk, uint32_t m, const JobGeom &g,
                            uint32_t job, uint32_t run) const;
        // Byte offset of macro slot (blk_id, m) inside one operand's block.
        uint32_t slot_offset(const JobGeom &g, uint32_t blk_id, uint32_t m, uint8_t kind) const;
        JobGeom job_geom[DIMC_NB_CONTEXT];
        // The running job's context.
        uint32_t exec_slot = 0;
        // Some block's fill program owes sections or its FIFOs hold any; it may be a later job's.
        bool     fill_active = false;
        bool     holds_unissued(const Dimc_Macro &m) const;
        void     write_feed(Dimc_InnerBlock &blk, uint32_t blk_id, std::deque<Dimc_InnerBlock::FeedEntry> &fifo);
        bool     beat_writable(const Dimc_Macro &mc, uint32_t job, const BeatPos &pos) const;
        bool     psin_set_free(const Dimc_Macro &mc, uint32_t job, uint32_t run) const;
        void     advance_fill();
        uint32_t block_order(uint32_t i) const;
        void     retire_block(Dimc_InnerBlock &blk);
        uint64_t out_dropped = 0;   // results the out_fifo lost (as the RTL would)
        void     fill_feed(Dimc_InnerBlock &blk, uint32_t blk_id, bool kernel_feed);
        bool     program_next(const Dimc_InnerBlock &blk, uint32_t m, BeatPos &pos) const;
        uint32_t pick_kernel(const Dimc_InnerBlock &blk, uint8_t &why) const;
        uint32_t pick_input(const Dimc_InnerBlock &blk, uint8_t &why) const;
        uint8_t  refusal(const std::queue<uint64_t> &pending, const Dimc_OuterPort &port) const;
        bool     kernel_prefetch(Dimc_InnerBlock &blk, uint32_t blk_id);
        bool     fetch_kernel_ahead(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m, int ctx,
                                    bool trace_refusal);
        void     fetch_kernel_section(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m, int ctx,
                                      uint32_t job, uint32_t sub);
        void     push_section(Dimc_InnerBlock &blk, uint32_t blk_id, Dimc_HWPE_Streamer &st,
                              Dimc_InnerBlock::FeedEntry &e,
                              std::deque<Dimc_InnerBlock::FeedEntry> &fifo,
                              std::queue<uint64_t> &pending, uint8_t kind);
        uint32_t kernel_in_progress(const Dimc_InnerBlock &blk) const;
        void     fill_beat(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t macro);

        void fetch_kernels_first();
        uint32_t rows_before_feature(const Dimc_Macro &mc, uint32_t job, uint32_t run) const;
        bool input_urgent() const;
        // Latch one context's job shape into its geometry slot.
        void latch_geom(int ctx);
        void ensure_geom(int ctx);
        uint32_t job_reg_ctx(int ctx, uint32_t addr) const;
        // Move retired rows into the out_fifos, then trigger every macro whose operands have landed.
        void compute_indep(Dimc_InnerBlock &blk, uint32_t blk_id);
        void store_block(Dimc_InnerBlock &blk, uint32_t blk_id);
        int  store_word(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m);

        // One cycle per fsm_event. An L1 access records when its response is due; each cycle
        // retires the ones that came back. outstanding_depth caps the requests in flight.
        uint32_t outstanding_depth;              // max in-flight requests per feed (kernel, input, store) per block
        uint64_t fsm_timestamp;                  // free-running engine cycle count
        uint64_t job_start_cycle;                // fsm_timestamp when this job began

    private:
        static vp::IoReqStatus hwpe_slave(vp::Block *__this, vp::IoReq *req);

        static void fsm_start_handler(vp::Block *__this, vp::ClockEvent *event);
        static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
        static void fsm_end_handler(vp::Block *__this, vp::ClockEvent *event);
        static void held_handler(vp::Block *__this, vp::ClockEvent *event);
        bool held_kernel_step(Dimc_InnerBlock &blk, uint32_t blk_id, int ctx);

        void fsm_loop();

        vp::ClockEvent *fsm_start_event;
        vp::ClockEvent *fsm_event;
        vp::ClockEvent *fsm_end_event;
        vp::ClockEvent *held_event;   // one cycle of the idle kernel preload
};

#endif
