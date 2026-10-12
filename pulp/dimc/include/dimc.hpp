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


// ---- A dual's half of the port towards L1: a bandwidth limiter that remembers when it is free ----
class Dimc_Port {
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

// ---- Inner block: num_macros macros on one inner (L1) port ----
// A plain class, not a vp::Component. One control plane drives N of these.
class Dimc_InnerBlock {
    public:
        // One kernel and one feature streamer per macro, so the two macros of a block can be on
        // different jobs; the output streamer is the first entry of out_stream. Fill and
        // write-back share the dual's half of the port.
        std::vector<Dimc_HWPE_Streamer> weight_stream;
        std::vector<Dimc_HWPE_Streamer> input_stream;
        std::vector<Dimc_HWPE_Streamer> out_stream;
        Dimc_Port port;
        std::vector<Dimc_Macro> macros;

        // Fill and store run concurrently, each with its own cursor. The fill keeps each
        // macro's position in its own program; the store counts the block's port words.
        struct FillCursor {
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
                this->macro_beat_index.assign(nb_macros, 0);
                this->macro_beat_total.assign(nb_macros, 0);
            }
        };
        struct StoreCursor {
            uint32_t beat_index = 0;
            uint32_t beat_total = 0;
            void reset()
            {
                this->beat_index = 0;
                this->beat_total = 0;
            }
        };
        FillCursor  fill;   // each macro's fill program, possibly for a later job
        StoreCursor store;  // the running job's results

        // Every outstanding L1 request of the block, fill and store; only the tracer reads it.
        std::queue<uint64_t> port_pending;
        // The running job's store beats: the job closes when these are acknowledged.
        std::queue<uint64_t> store_pending;
        // Per-feed budgets: kernel sections, and the input feed (feature vectors).
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
        // The dual's one input FIFO: both macros' sections in fetch order.
        std::deque<FeedEntry> inp_fifo;
        // The dual's out_fifo: results of the computing macro in push order, one popped per
        // cycle into acc[run * DIMC_MACRO_KB_LEN + row].
        struct OutEntry { int32_t psout; uint16_t row; uint16_t run; uint8_t set; };
        std::deque<OutEntry> out_fifo;
        int32_t  acc[DIMC_ACC_VECTORS * DIMC_MACRO_KB_LEN] = {0};
        int64_t  last_acc_cycle = -1000;   // cycle of the last result accumulated
        uint32_t acc_results[2] = {0, 0};  // results accumulated per result set (job id & 1)
        int64_t  present_ack_cycle = -1000; // cycle the last output word was acknowledged
        // The input tile being read: 128 rows of 8 bytes, up to four rows per read request,
        // which the streamer transposes into the job's 8 vectors (dimc_streamer.sv, input_tile_q).
        uint32_t tile_job = 0xFFFFFFFFu;
        uint32_t tile_rows = 0;
        uint64_t tile_ready = 0;           // first cycle after the last row's response
        uint8_t  tile[DIMC_MACRO_FB_EW][DIMC_ACC_VECTORS];
        // Rows of the running job's tile written to L1 so far.
        uint32_t out_written = 0;
        // In DIMC_STORING: every result of the running job written and acknowledged, as of
        // the last cycle; the block then stops writing back.
        bool phase_done;

        // Clear what the engine tracks for one job, at every job start. keep_fill leaves the
        // fill cursor, the in-flight beats and the input FIFO alone: the fill runs ahead into
        // the next job. The weight FIFO is not cleared here.
        void reset_job_state(bool keep_fill)
        {
            if (!keep_fill) {
                this->fill.reset(this->macros.size());
                this->inp_fifo.clear();
                this->tile_job = 0xFFFFFFFFu;
                this->tile_rows = 0;
                while (!this->port_pending.empty()) this->port_pending.pop();
                while (!this->kb_pending.empty()) this->kb_pending.pop();
                while (!this->in_pending.empty()) this->in_pending.pop();
            }
            while (!this->store_pending.empty()) this->store_pending.pop();
            this->store.reset();
            this->out_written = 0;
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
        uint32_t inner_port_bytes;    // one 256-bit section, bytes
        // An access costs what L1 returns, plus the wait for the inner block's port.
        uint32_t nb_inner_blocks;     // inner blocks in the outer block (2 on the D-tile)

        // Text trace, for diagnostics and the tracer's reports.
        vp::Trace trace;

        // Every VCD signal and per-cycle observation; read-only towards the engine.
        Dimc_Tracer tracer;

        // Internal state
        vp::reg_32 state;

        // ---- Jobs, as software queues them (hwpe-ctrl, 2 contexts) ----
        struct Gemm {
            uint32_t id;
            uint32_t reg[DIMC_NB_REGS];
        };
        uint32_t gemm_live[DIMC_NB_REGS];   // the registers software writes
        std::deque<Gemm> gemm_queue;        // committed, not started
        Gemm     gemm;                      // the running job
        bool     gemm_running;
        bool     gemm_acquired;             // ACQUIRE read, commit pending
        bool     gemm_error;                // the last job was rejected
        uint32_t next_gemm_id;
        uint32_t finished_jobs;
        // Tile products of the running job: the next one's tile indices, how many are
        // committed to the engine, and how many it has.
        uint32_t gemm_k, gemm_q, gemm_l, gemm_issued, gemm_total;
        bool     gemm_valid(const Gemm &g) const;
        void     gemm_step();               // start jobs, commit tile products, end jobs
        void     gemm_commit_tile(int ctx);
        void     gemm_finish(bool error);

        // ---- The engine's own queue of tile products ----
        uint32_t running_job;   // id of the tile product the engine runs
        uint32_t next_job_id;
        bool     job_running;
        uint32_t ctx_regs[DIMC_NB_CONTEXT][DIMC_TP_NB_REGS];
        bool     ctx_busy[DIMC_NB_CONTEXT];
        uint32_t ctx_job_id[DIMC_NB_CONTEXT];
        // Mode, sign and compute mask of the job a tile product belongs to: a macro takes
        // them with the operands, since it may trigger while an earlier job's last tile runs.
        uint8_t  ctx_ci[DIMC_NB_CONTEXT];
        uint8_t  ctx_sign_8b[DIMC_NB_CONTEXT];
        uint16_t ctx_compute_mask[DIMC_NB_CONTEXT];
        int      running_ctx;                  // context the engine executes (-1 none)
        // Committed-but-not-yet-running contexts, in commit order.
        std::deque<int> ctx_queue;

        int      ctx_alloc();                  // reserve a free context, -1 if none
        void     start_next_job();             // launch the pending context, if any
        // The engine and the job queue as a reset leaves them.
        void     clear_engine();

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
            uint32_t row_count, row_base, compute_cyc;
            uint32_t tile_last;   // write the accumulators out after this tile product
            uint32_t beats_per_macro, kb_beats_per_row;
            uint32_t fb_beats_per_macro, out_beats;
            // Batched descriptor: vectors per job and their strides (1 / 0 = one vector).
            uint32_t nb_vec, out_row_stride, kb_row_stride, nb_blocks;
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
        void     configure_kernel_stream(uint32_t blk_id, uint32_t m, int ctx);
        void     configure_out_stream(uint32_t blk_id, uint32_t row, uint32_t ctx);
        JobGeom job_geom[DIMC_NB_CONTEXT];
        // The running job's context.
        uint32_t exec_slot = 0;
        // Some block's fill program owes sections or its FIFOs hold any; it may be a later job's.
        bool     fill_active = false;
        bool     holds_unissued(const Dimc_Macro &m) const;
        void     write_feed(Dimc_InnerBlock &blk, uint32_t blk_id, std::deque<Dimc_InnerBlock::FeedEntry> &fifo);
        bool     beat_writable(const Dimc_Macro &mc, uint32_t job, const BeatPos &pos) const;
        void     advance_fill();
        void     retire_block(Dimc_InnerBlock &blk);
        uint64_t out_dropped = 0;   // results the out_fifo lost (as the RTL would)
        void     fill_feed(Dimc_InnerBlock &blk, uint32_t blk_id, bool kernel_feed);
        // Macro m's fill program still has kernel / feature sections to fetch.
        bool     kernel_owed(const Dimc_InnerBlock &blk, uint32_t m) const;
        bool     feature_owed(const Dimc_InnerBlock &blk, uint32_t m) const;
        uint32_t pick_kernel(const Dimc_InnerBlock &blk, uint8_t &why) const;
        uint32_t pick_input(const Dimc_InnerBlock &blk, uint8_t &why) const;
        uint8_t  refusal(const std::queue<uint64_t> &pending, const Dimc_Port &port) const;
        void     fetch_kernel_section(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m, int ctx,
                                      uint32_t job, uint32_t sub);
        void     push_section(Dimc_InnerBlock &blk, uint32_t blk_id, Dimc_HWPE_Streamer &st,
                              Dimc_InnerBlock::FeedEntry &e,
                              std::deque<Dimc_InnerBlock::FeedEntry> &fifo,
                              std::queue<uint64_t> &pending, uint8_t kind);
        uint32_t kernel_in_progress(const Dimc_InnerBlock &blk) const;
        void     feed_input(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t macro);

        uint32_t rows_before_turn(const Dimc_InnerBlock &blk, uint32_t m, uint32_t job) const;
        // The macro of a dual that runs `job`: the two take turns.
        uint32_t job_macro(uint32_t job) const { return job % this->num_macros; }
        bool     tile_computed(const Dimc_InnerBlock &blk) const;
        // Latch one context's job shape into its geometry slot.
        void latch_geom(int ctx);
        void ensure_geom(int ctx);
        uint32_t job_reg_ctx(int ctx, uint32_t idx) const;
        // Accumulate, move retired rows into the out_fifo, trigger the macro whose turn it is.
        void compute_indep(Dimc_InnerBlock &blk, uint32_t blk_id);
        void store_block(Dimc_InnerBlock &blk, uint32_t blk_id);
        int  store_word(Dimc_InnerBlock &blk, uint32_t blk_id);

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

        void fsm_loop();

        vp::ClockEvent *fsm_start_event;
        vp::ClockEvent *fsm_event;
        vp::ClockEvent *fsm_end_event;
};

#endif
