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
// still unwritten when the fill ends, STORING. Value 2 was a COMPUTING state
// that did no work; the other values keep their encoding.
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


// ---- Outer port: the tile's shared port towards memory ----
// A bandwidth limiter in the same idiom as interco/router's: the port remembers
// when it is free again, so a client arriving before that waits. Accounting is
// byte-granular, so two 32-byte beats occupy one cycle of a 64 B/cycle port
// rather than one cycle each.
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
        // The streamer's input, kernel and output TCDM ports are separate initiators, so
        // fill and write-back proceed in the same cycle. One feed cannot give two macros
        // different words in one cycle.
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
        // Two levels of position: beat_index/beat_total bound the block's
        // whole phase, and macro_beat_index/macro_beat_total say where each
        // macro is in its own fill program. fill_feed picks, per feed, the macro a
        // section is fetched for; the per-macro pair gives that section's position.
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

        // Every outstanding L1 request of the block, fill and store. Budgets are
        // per port (kb_pending, in_pending, store_pending); this union is read
        // only by the tracer.
        std::queue<uint64_t> port_pending;
        // The store beats alone. The job closes when these are acknowledged;
        // waiting on port_pending would also wait for the next job's fill beats.
        std::queue<uint64_t> store_pending;
        // Per-feed budgets, one per TCDM port: kernel rows, and the input feed
        // (feature and partial sums).
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
        // Per macro, half the dual's storage; with DIMC_INP_FIFO_SHARED only inp_fifo[0] is
        // used, as the dual's one input FIFO (inp_queue).
        std::vector<std::deque<FeedEntry>> inp_fifo;
        std::deque<FeedEntry> &inp_queue(uint32_t macro)
        {
            return this->inp_fifo[DIMC_INP_FIFO_SHARED ? 0 : macro];
        }
        // Per-macro out_fifo: results in push order. A port word leaves from the head
        // entries; out_results counts the running job's results that have left.
        struct OutEntry { int32_t psout; uint16_t row; uint16_t run; uint16_t macro; };
        std::vector<std::deque<OutEntry>> out_fifo;
        // DIMC_OUT_FIFO_SHARED: the dual's one out_fifo, both macros' results in push order.
        std::deque<OutEntry> out_shared;
        std::vector<uint32_t> out_results;
        // DIMC_NEXT_JOB_SINK: the next job's results that already left, per macro, its
        // beats, and their acknowledgements; taken over when that job starts.
        std::vector<uint32_t> out_results_next;
        uint32_t store_next_beats = 0;
        std::queue<uint64_t> store_next_pending;
        // Write-back position per macro: a macro's runs leave in vector order, and a run
        // counts once its last beat is acknowledged.
        struct RunDone { uint64_t due; uint32_t job, macro, run; };
        std::deque<RunDone> run_pending;
        struct Retired { int64_t job = -1; uint32_t runs = 0; };
        std::vector<Retired> retired;    // per macro
        // DIMC_JOB_LOOKAHEAD > 1. Per macro, in the order it triggers its jobs: the jobs
        // with rows still in its pipe, and the jobs with results still to write back
        // (out_results counts for the front one). pipe_done / stored: the last job whose
        // rows have all left the pipe / whose results have all left the out_fifo.
        struct JobLeft { uint32_t job, slot, left; };
        std::vector<std::deque<JobLeft>> in_pipe;
        std::vector<std::deque<JobLeft>> to_store;
        std::vector<uint32_t> pipe_done, stored;
        // Write-back beats not yet acknowledged, with their job.
        struct StoreAck { uint64_t due; uint32_t job; };
        std::deque<StoreAck> store_acks;
        void reset_progress()
        {
            this->in_pipe.assign(this->macros.size(), {});
            this->to_store.assign(this->macros.size(), {});
            this->pipe_done.assign(this->macros.size(), 0xFFFFFFFFu);
            this->stored.assign(this->macros.size(), 0xFFFFFFFFu);
            this->store_acks.clear();
        }

        uint32_t rows_issued;                    // compute: rows pushed into pipes

        // Per-phase completion, so the phase ends only when EVERY block is done.
        bool phase_done;

        // Accumulates across jobs, so reset_job_state() must not touch it.
        Dimc_OutAccum out_accum;

        // Clear everything the engine tracks for one job. Called from the
        // constructor, from reset(), and at every job start, so the three sites
        // cannot drift apart.
        // keep_fill leaves the fill cursor and the in-flight beats alone: the
        // fill runs ahead into the next job, so a job boundary must not reset it.
        // keep_wgt leaves the weight FIFO alone: it may hold a held job's kernel
        // (DIMC_HELD_KB_TO_FIFO) that the macro takes once the job runs.
        void reset_job_state(bool keep_fill = false, bool keep_store = false, bool keep_wgt = false)
        {
            if (!keep_fill) {
                this->fill.reset(this->macros.size());
                if (!keep_wgt) this->wgt_fifo.clear();
                for (auto &q : this->inp_fifo) q.clear();
                while (!this->port_pending.empty()) this->port_pending.pop();
                while (!this->kb_pending.empty()) this->kb_pending.pop();
                while (!this->in_pending.empty()) this->in_pending.pop();
            }
            // The write-back follows each macro across job boundaries (DIMC_JOB_LOOKAHEAD > 1).
            if (!keep_store) {
                while (!this->store_pending.empty()) this->store_pending.pop();
                this->store.reset(this->macros.size());
                for (uint32_t &n : this->out_results) n = 0;
            }
            this->rows_issued = 0;
            this->phase_done = false;
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
        // Each owns its streamers and macros, and reaches L1 through its own
        // inner port.
        std::vector<Dimc_InnerBlock> inner_blocks;

        uint8_t sel_dimc;

        // Configuration
        uint32_t num_macros;
        // Streamer bandwidth: fixed hardware properties, set once from the
        // systree / gvrun --param (no per-trigger MMIO override).
        uint32_t inner_port_bytes;    // one inner block's port, bytes/cycle
        uint32_t outer_port_bytes;    // the tile's shared outer port, bytes/cycle
        // One outer-port budget per direction, each shared by all inner blocks: stream-in
        // (the three source streams) and stream-out (the sink) never block one another.
        Dimc_OuterPort outer_port_in;   // fills: feature, partial sums, kernel rows
        Dimc_OuterPort outer_port_out;  // write-back
        // ---- Outer block ----
        // An inner block is num_macros macros on one inner port; an outer block
        // is nb_inner_blocks of them, and all of their beats pass through one
        // shared outer port. An access costs whatever the L1 bank and the
        // crossbar return; the only delay the accelerator adds of its own is
        // the wait when more beats want the shared port in a cycle than its
        // bandwidth covers.
        uint32_t nb_inner_blocks;     // inner blocks in the outer block (D-tile default 2)

        // Text trace, for diagnostics and the tracer's reports.
        vp::Trace trace;

        // Every VCD signal, per-cycle observation and accounting. Read-only
        // towards the engine and never schedules an event; see dimc_tracer.hpp.
        Dimc_Tracer tracer;

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
        // hwpe_ctrl_target.sv: there is ONE live bundle of
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
        // Committed-but-not-yet-running contexts, in commit order. A FIFO and
        // not a pair of scalars: with DIMC_NB_CONTEXT above 2 the software can
        // queue more than one job ahead of the running one.
        std::deque<int> ctx_queue;

        int      ctx_alloc();                  // reserve a free context, -1 if none
        uint32_t job_reg(uint32_t addr) const; // read a job-dep reg of the RUNNING ctx
        void     start_next_job();             // launch the pending context, if any

        // ---- Engine phase iterators ----
        // Called once per cycle. Each sets *latency=1 and returns true when its
        // phase is done. The makespan is not returned: it accrues in
        // fsm_timestamp.
        //   preload_iter : per-beat fill of every macro, with compute and
        //                  write-back of whatever has retired
        //   store_iter   : the write-back still owed when the fill ended
        bool preload_iter(int *latency);
        bool store_iter(int *latency);
        // Every result beat issued and acknowledged, on every block.
        bool store_done() const;
        // Close the running job: clear the phase state and report it.
        void close_job();

        // Scratch shared across the phases of one job (same for all blocks:
        // one control plane issues one job shape to every inner block).
        // Everything derived once per job. One per context, so a queued job's
        // shape survives until the engine reaches it.
        struct JobGeom {
            uint32_t num_active, row_count, row_base, compute_cyc;
            bool     skip_kb;
            uint32_t psin_dep;    // PSIN_DEP: jobs back to the partial sums' producer
            uint32_t psin_chain;  // PSIN_CHAIN: producer is the same macro (0) or the run before (1)
            // Slot addressing: ADDR_MODE and the per-macro / per-block strides of each operand.
            uint32_t addr_mode;
            uint32_t kb_ms, kb_bs, fb_ms, fb_bs, ps_ms, ps_bs, out_ms, out_bs;
            uint32_t psin_rows;
            uint32_t beats_per_macro, kb_beats_per_row;
            uint32_t fb_beats_per_macro, psin_beats_per_macro, out_beats;
            // Batched descriptor: vectors per job and their strides (1 / 0 = one vector).
            uint32_t nb_vec, fb_vec_stride, ps_vec_stride, out_vec_stride;
        };
        // The job id whose geometry job_geom[ctx] holds, so a job is latched once
        // even when its macros start their fill programs at different times.
        uint32_t geom_job[DIMC_NB_CONTEXT];
        // Where beat `within` of a macro's fill program falls: its kind, the run
        // (vector) it belongs to, and its index inside that segment.
        struct BeatPos { uint8_t kind; uint32_t run; uint32_t sub; };
        BeatPos  beat_pos(const JobGeom &g, uint32_t within) const;
        uint32_t stamp_beats(const JobGeom &g) const;
        void     size_fill(int ctx);
        void     plan_macro_fill(uint32_t blk_id, uint32_t m, int ctx);
        void     configure_macro_streams(uint32_t blk_id, uint32_t m, int ctx, uint32_t run,
                                         bool kernel, bool feature, bool psum);
        void     configure_out_stream(uint32_t blk_id, uint32_t m, uint32_t run);
        void     configure_out_stream_ctx(uint32_t blk_id, uint32_t m, uint32_t run, uint32_t ctx);
        bool     psum_ready(const Dimc_InnerBlock &blk, uint32_t m, const JobGeom &g,
                            uint32_t job, uint32_t run) const;
        // Byte offset of macro slot (blk_id, m) inside one operand's block.
        uint32_t slot_offset(const JobGeom &g, uint32_t blk_id, uint32_t m, uint8_t kind) const;
        uint64_t psum_waits = 0;             // cycles a feed idled on the partial-sum producer
        uint64_t psum_waits_reported = 0;
        uint32_t psum_reports = 0;
        JobGeom job_geom[DIMC_NB_CONTEXT];
        // exec_slot is the running job's context. fill_slot is the context of the
        // latest fill program planned, which may be a later job's.
        uint32_t fill_slot = 0;
        uint32_t exec_slot = 0;
        // A fill is planned and still owes beats. It may belong to a job that has
        // not started computing yet.
        bool     fill_active = false;
        // Job id of the current fill. fill_slot alone is ambiguous: slot indices
        // repeat across jobs.
        uint32_t fill_job = 0xFFFFFFFFu;
        void     configure_fill_streams(int ctx);
        // Fill order within one macro's beats, and where its operands are complete
        // enough to compute (the partial sums may still be streaming after that).
        uint8_t  beat_kind(const JobGeom &g, uint32_t within) const;
        uint32_t core_beats(const JobGeom &g) const;
        bool     holds_unissued(const Dimc_Macro &m) const;
        void     write_feed(Dimc_InnerBlock &blk, uint32_t blk_id, std::deque<Dimc_InnerBlock::FeedEntry> &fifo);
        bool     beat_writable(const Dimc_Macro &mc, uint32_t job, const BeatPos &pos) const;
        void     advance_fill();
        void     handover_step();
        uint32_t block_order(uint32_t i) const;
        void     retire_block(Dimc_InnerBlock &blk);
        uint64_t out_dropped = 0;   // results the out_fifo lost (as the RTL would)
        void     fill_feed(Dimc_InnerBlock &blk, uint32_t blk_id,
                           Dimc_InnerBlock::Cursor &cursor, bool kernel_feed);
        bool     kernel_prefetch(Dimc_InnerBlock &blk, uint32_t blk_id);
        uint32_t kernel_in_progress(const Dimc_InnerBlock &blk) const;
        bool     kernels_fetched_before(Dimc_InnerBlock &blk, uint32_t job);
        uint32_t kb_end(const JobGeom &g) const;
        void     fill_beat(Dimc_InnerBlock &blk, uint32_t blk_id,
                           Dimc_InnerBlock::Cursor &cursor, uint32_t macro,
                           std::queue<uint64_t> &feed_pending);
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
        // preload_block's two halves: the fetches into the dual's FIFOs, and the pops from
        // them into the macros. Split so the pops can follow the cycle's row triggers.
        void fetch_feeds(Dimc_InnerBlock &blk, uint32_t blk_id, Dimc_InnerBlock::Cursor &cursor);
        void fetch_kernels_first();
        uint32_t rows_before_feature(const Dimc_Macro &mc, uint32_t job, uint32_t run) const;
        bool input_urgent() const;
        void write_feeds(Dimc_InnerBlock &blk, uint32_t blk_id);
        // One block's cycle of fetch, trigger and write, in the order the switch selects.
        void block_cycle(Dimc_InnerBlock &blk, uint32_t blk_id);
        void plan_fill(int ctx);
        // Latch one context's job shape into its geometry slot.
        void latch_geom(int ctx);
        uint32_t job_reg_ctx(int ctx, uint32_t addr) const;
        // Advance every macro of a block that has finished its own fill.
        void compute_indep(Dimc_InnerBlock &blk, uint32_t blk_id);
        void store_block(Dimc_InnerBlock &blk, uint32_t blk_id);
        void store_next_job(Dimc_InnerBlock &blk, uint32_t blk_id);
        // DIMC_JOB_LOOKAHEAD > 1
        int      slot_of_job(uint32_t job) const;
        void     advance_fill_ahead();
        void     store_block_ahead(Dimc_InnerBlock &blk, uint32_t blk_id);
        void     retire_store_acks(Dimc_InnerBlock &blk);
        bool     rows_done_ahead(const Dimc_InnerBlock &blk) const;
        bool     store_done_ahead() const;
        // Move every row a macro has finished into that macro's out_fifo.
        // compute_indep needs this both at the top of a cycle and once more
        // when the last row retires, so it lives in one place.
        void drain_ready_rows(Dimc_InnerBlock &blk);

        // ---- Cycle-accurate engine ----
        // One cycle per fsm_event. TCDM accesses are async: on issue we record
        // when the response is due, and each cycle we retire the beats that
        // came back. outstanding_depth caps in-flight requests, so slow memory
        // back-pressures the engine.
        //
        // The load walks (macro, row, offset), decoded from the beat cursors:
        // beat_index bounds the block's phase and macro_beat_index[] locates a
        // beat within its own macro's stream.
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
        static void held_handler(vp::Block *__this, vp::ClockEvent *event);
        bool held_kernel_step(Dimc_InnerBlock &blk, uint32_t blk_id, int ctx);

        void fsm_loop();
        int  fsm();

        vp::ClockEvent *fsm_start_event;
        vp::ClockEvent *fsm_event;
        vp::ClockEvent *fsm_end_event;
        vp::ClockEvent *held_event;   // DIMC_HELD_KB_PRELOAD: one cycle of the idle preload
};

#endif
