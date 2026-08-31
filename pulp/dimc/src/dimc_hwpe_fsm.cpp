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

#include <dimc.hpp>
#include <cstring>
#include <algorithm>

void Dimc_HWPE::job_shape(uint32_t *num_active, uint32_t *row_count) const
{
    uint32_t n = this->register_file[DIMC_HWPE_NUM_MACROS >> 2];
    if (n == 0 || n > this->num_macros) n = this->num_macros;
    uint32_t r = this->job_reg(DIMC_HWPE_ROW_COUNT);
    if (r == 0) r = 1;
    if (r > DIMC_MACRO_KB_LEN) r = DIMC_MACRO_KB_LEN;
    *num_active = n;
    *row_count  = r;
}

void Dimc_HWPE::fsm_start_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;

    _this->trace.msg(vp::TraceLevel::WARNING, "DIMC job start\n");

    // Clear STATUS at job start so a back-to-back trigger (e.g. reuse without a
    // soft_clear) does not see the previous job's STATUS=1 and exit polling early.
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x0;

    // Every inner block gets its own streamers. Block b works on its own slice
    // of the job: the descriptor gives the base of block 0, and each further
    // block is offset by one block's worth of kernels, features and outputs.
    // That is what makes the blocks compute DIFFERENT data in parallel instead
    // of repeating each other. Software must therefore lay out
    // nb_inner_blocks x num_active kernels back to back.
    uint32_t blk_num_active, blk_row_count;
    _this->job_shape(&blk_num_active, &blk_row_count);

    // Per-macro spans now: macro m of block b owns slot (b * num_active + m).
    const uint32_t kb_one   = blk_row_count * DIMC_MACRO_KB_EW;
    const uint32_t fb_one   = DIMC_MACRO_FB_EW;
    const uint32_t out_one  = blk_row_count * 4;

    uint32_t blk_id = 0;
    for (Dimc_InnerBlock &blk : _this->inner_blocks) {
    for (uint32_t m = 0; m < blk_num_active; m++) {
    const uint32_t slot = blk_id * blk_num_active + m;
    blk.weight_stream[m].configure(
        _this->job_reg(DIMC_HWPE_JOB_KB_SRC_ADDR) + slot * kb_one,     // base_addr
        kb_one,                                      // tot_len
        _this->job_reg(DIMC_HWPE_KB_D0_LENGTH),   // d0_len
        _this->job_reg(DIMC_HWPE_KB_D0_STRIDE),   // d0_stride
        _this->job_reg(DIMC_HWPE_KB_D1_LENGTH),   // d1_len
        _this->job_reg(DIMC_HWPE_KB_D1_STRIDE),   // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the feature (FB) input streamer
    blk.input_stream[m].configure(
        _this->job_reg(DIMC_HWPE_JOB_FB_SRC_ADDR) + slot * fb_one,     // base_addr
        fb_one,                                      // tot_len
        _this->job_reg(DIMC_HWPE_FB_D0_LENGTH),   // d0_len
        _this->job_reg(DIMC_HWPE_FB_D0_STRIDE),   // d0_stride
        0,                                                      // d1_len
        0,                                                      // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the output streamer
    // Per-row partial sums in. Same per-block slicing as the outputs, since a
    // psum belongs to the row that produced it.
    blk.psin_stream[m].configure(
        _this->job_reg(DIMC_HWPE_JOB_PSIN_SRC_ADDR) + slot * out_one,
        out_one, 0, 0, 0, 0, 0, 0, 0
    );

    blk.out_stream[m].configure(
        _this->job_reg(DIMC_HWPE_JOB_DST_ADDR) + slot * out_one,      // base_addr
        out_one,                                     // tot_len
        _this->job_reg(DIMC_HWPE_OUT_D0_LENGTH),  // d0_len
        _this->job_reg(DIMC_HWPE_OUT_D0_STRIDE),  // d0_stride
        0,                                                      // d1_len
        0,                                                      // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );
    }
    blk_id++;
    }

    // Latch the per-job compute configuration once at commit, then broadcast it
    // to every macro (ci, sign_8b, compute_mask, sel_dimc).
    uint8_t  compe     = (uint8_t) (_this->register_file[DIMC_HWPE_COMPE        >> 2] & 0x1);
    uint8_t  ci        = (uint8_t) (_this->register_file[DIMC_HWPE_CFG_CI       >> 2] & 0x3);
    uint8_t  sign_8b   = (uint8_t) (_this->register_file[DIMC_HWPE_SIGN_8B      >> 2] & 0x3);
    uint16_t cmask     = (uint16_t)(_this->register_file[DIMC_HWPE_COMPUTE_MASK >> 2] & 0x3FF);
    uint8_t  psin_rows = (uint8_t) (_this->job_reg(DIMC_HWPE_PSIN_EN) & 0x1);
    _this->job_geom[_this->fill_slot].psin_rows = psin_rows;
    _this->sel_dimc   = (uint8_t)(_this->register_file[DIMC_HWPE_SEL_DIMC  >> 2] & 0xFF);
    for (Dimc_InnerBlock &blk : _this->inner_blocks)
    for (auto &m : blk.macros) {
        // `compe` (memory-vs-compute mode) is latched but never read: compute_PP
        // always performs the dot product, and COMPE=0 memory mode is not
        // implemented.
        m.compe = compe; m.ci = ci; m.sign_8b = sign_8b; m.compute_mask = cmask;
        m.psin_rows  = psin_rows;
    }
    // `sel_dimc` is stored but never consulted: macro selection goes through
    // NUM_MACROS (num_active). It exists for register-map fidelity.

    // job_running / running_job were latched by start_next_job() at commit.
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x0;   // busy

    // fsm_timestamp runs free across jobs, and the outer port keeps its
    // busy-until stamp, so a transfer still in flight when the next job starts
    // pushes that job out. Zeroing both per job was safe only because the phase
    // barriers drained everything first; it also made cross-job overlap
    // impossible to express. job_start_cycle is what the makespan trace
    // subtracts to stay per-job.
    _this->job_start_cycle = _this->fsm_timestamp;
    _this->phase_planned = false;
    _this->exec_slot = (uint32_t)(_this->running_ctx >= 0 ? _this->running_ctx : 0);

    // If the queued context that just became the running one was prefetched,
    // its operands are already in the spare banks: flip them in and skip the
    // load entirely. This is the whole point of the second context.
    _this->job_prefetched = (_this->prefetch_ctx == _this->running_ctx
                             && _this->prefetch_ready);
    if (_this->job_prefetched) {
        JobGeom &pg = _this->job_geom[_this->exec_slot];
        for (Dimc_InnerBlock &blk : _this->inner_blocks)
            for (uint32_t m = 0; m < pg.num_active; m++) {
                Dimc_Macro &mc = blk.macros[m];
                mc.exec_ready  = true;
                mc.last_kb_src = pg.kb_src;   // the prefetched fill landed
                mc.kb_ready = true;
                mc.fb_ready = true;
                mc.pipe.clear();
                mc.psin_scalar = (int32_t)_this->job_reg_ctx((int)_this->exec_slot,
                                                             DIMC_HWPE_PSIN);
                mc.fb_cur  = mc.fb_fill;
                mc.fb_fill = (uint8_t)(mc.fb_fill ^ 1u);
                if (!pg.skip_kb) {
                    mc.kb_cur  = mc.kb_fill;
                    mc.kb_fill = (uint8_t)(mc.kb_fill ^ 1u);
                }
            }
    }
    _this->prefetch_ctx   = -1;
    _this->prefetch_ready = false;

    if (!_this->job_prefetched)
        for (Dimc_InnerBlock &blk : _this->inner_blocks)
            for (uint32_t m = 0; m < _this->num_macros; m++)
                blk.macros[m].exec_ready = false;

    for (Dimc_InnerBlock &blk : _this->inner_blocks) blk.reset_job_state();
    if (_this->job_prefetched)
        for (Dimc_InnerBlock &blk : _this->inner_blocks)
            for (uint32_t m = 0; m < _this->num_macros; m++)
                blk.macros[m].beat_total = 0;   // nothing left to load

    _this->state.set(DIMC_STARTING);

    uint8_t one = 1, st = DIMC_STARTING;
    _this->busy_event.event(&one);
    _this->job_event.event((uint8_t *)&_this->running_job);
    _this->state_event.event(&st);

    _this->fsm_loop();
}

void Dimc_HWPE::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    _this->fsm_loop();
}

void Dimc_HWPE::fsm_end_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    _this->state.set(DIMC_IDLE);

    uint8_t zero = 0, st = DIMC_IDLE;
    _this->busy_event.event(&zero);
    _this->state_event.event(&st);

    // Retire the context this job used, then launch whatever is queued behind it.
    if (_this->running_ctx >= 0) _this->ctx_busy[_this->running_ctx] = false;
    _this->running_ctx = -1;
    _this->job_running = false;
    _this->finished_jobs++;
    _this->register_file[DIMC_HWPE_STATUS    >> 2] = 0x1;                 // done
    _this->register_file[DIMC_HWPE_FIN_JOBS  >> 2] = _this->finished_jobs;
    _this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC job done, STATUS=1, finished_jobs=%u qlen=%u busy=%d%d%d%d acq=%d\n",
        _this->finished_jobs, (unsigned)_this->ctx_queue.size(),
        (int)_this->ctx_busy[0], (int)_this->ctx_busy[1],
        (int)_this->ctx_busy[2 % DIMC_NB_CONTEXT], (int)_this->ctx_busy[3 % DIMC_NB_CONTEXT],
        _this->acquired_ctx);
    // Standard HWPE completion interrupt (pulse), if the line is wired.
    if (_this->irq.is_bound()) {
        _this->irq.sync(true);
        _this->irq.sync(false);
    }
    // autotrigger_n (hwpe-ctrl): 0 = chain into the next queued job automatically,
    // 1 = hold the queue until SW issues an explicit trigger (commit_trigger 0/2).
    if ((_this->register_file[DIMC_HWPE_AUTOTRIGGER_N >> 2] & 0x1) == 0)
        _this->start_next_job();
}

void Dimc_HWPE::fsm_loop()
{
    // One fsm() per call. Every phase iterator sets latency to 1
    // unconditionally, so a loop here would never take a second turn; the job
    // advances one cycle per fsm_event and the makespan accrues in
    // fsm_timestamp, not here. A phase that wanted to advance without spending
    // a cycle would return 0 and need the loop back.
    uint32_t latency = (uint32_t)this->fsm();

    // On completion fsm() returns 1, not the makespan: those cycles were already
    // spent stepping. Returning the makespan here would count them twice.
    if (state.get() == DIMC_FINISHED && !this->fsm_end_event->is_enqueued()) {
        this->event_enqueue(this->fsm_end_event, latency);
    } else if (!this->fsm_event->is_enqueued()) {
        this->event_enqueue(this->fsm_event, latency);
    }
}

// Size one job's fill: geometry into its slot, per-macro beat budgets, and the
// outer-port reservation. Shared by the running job's own load and by the
// prefetch of the queued one.
void Dimc_HWPE::latch_geom(int ctx)
{
    JobGeom &g = this->job_geom[ctx];
    uint32_t n = this->register_file[DIMC_HWPE_NUM_MACROS >> 2];
    if (n == 0 || n > this->num_macros) n = this->num_macros;
    uint32_t r = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_COUNT);
    if (r == 0) {
        this->trace.force_warning("latch_geom: ctx %d has ROW_COUNT=0 "
            "(never configured); running_ctx=%d prefetch_ctx=%d qlen=%u\n",
            ctx, this->running_ctx, this->prefetch_ctx,
            (unsigned)this->ctx_queue.size());
        r = 1;
    }
    if (r > DIMC_MACRO_KB_LEN) r = DIMC_MACRO_KB_LEN;

    g.num_active  = n;
    g.row_count   = r;
    g.row_base    = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_SEL_BASE);
    g.bias        = (int32_t)this->job_reg_ctx(ctx, DIMC_HWPE_CFG_BIAS);
    g.compute_cyc = r + DIMC_MACRO_LATENCY;
    g.psin_rows   = this->job_reg_ctx(ctx, DIMC_HWPE_PSIN_EN) & 0x1;

    uint32_t kb_src = this->job_reg_ctx(ctx, DIMC_HWPE_JOB_KB_SRC_ADDR);
    bool all_skip = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        for (uint32_t m = 0; m < n; m++) {
            Dimc_Macro &mc = blk.macros[m];
            mc.skip_kb = (kb_src == mc.last_kb_src);
            if (!mc.skip_kb) all_skip = false;
        }
    g.skip_kb = all_skip;
    g.kb_src  = kb_src;
    // last_kb_src is NOT stamped here. A prefetch that is planned and then
    // abandoned -- the running job ends before its beats are done -- would
    // otherwise leave the stamp behind, and the real fill that replaces it
    // would see skip_kb and skip a weight load that never happened.
}

void Dimc_HWPE::plan_fill(int ctx)
{
    const bool prefetch = (this->prefetch_ctx == ctx);
    const uint32_t port_bytes = this->inner_port_bytes;
    JobGeom &g = this->job_geom[ctx];
    uint32_t num_active = g.num_active, row_count = g.row_count;

    g.kb_beats_per_row   = (DIMC_MACRO_KB_EW + port_bytes - 1) / port_bytes;
    g.fb_beats_per_macro = (DIMC_MACRO_FB_EW + port_bytes - 1) / port_bytes;
    g.psin_beats_per_macro = g.psin_rows
        ? (row_count * 4 + port_bytes - 1) / port_bytes : 0;
    uint32_t kb_rows = g.skip_kb ? 0 : row_count;
    g.beats_per_macro = kb_rows * g.kb_beats_per_row
                      + g.fb_beats_per_macro + g.psin_beats_per_macro;

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        // A prefetch plans into its own cursor; the block-level one belongs to
        // the phase the running job is in and must not be touched.
        if (prefetch) {
            blk.prefetch_beat_total = num_active * g.beats_per_macro;
            blk.prefetch_beat_index = 0;
            while (!blk.prefetch_pending.empty()) blk.prefetch_pending.pop();
        } else {
            blk.beat_total = num_active * g.beats_per_macro;
            blk.beat_index = 0;
        }
        blk.fill_requested = false;
        for (uint32_t m = 0; m < num_active; m++) {
            blk.macros[m].beat_index = 0;
            blk.macros[m].beat_total = g.beats_per_macro;
            blk.macros[m].fill_done_cycle = 0;
        }
    }
}

// One beat per block per cycle for the queued context, run in every state
// except the one where the current job still owns the streamers.
static inline size_t retire_due(std::queue<uint64_t> &q, uint64_t now)
{
    while (!q.empty() && q.front() <= now) q.pop();
    return q.size();
}

// Point the three input streamers at a context's own operands. The job-start
// path configures every streamer from the RUNNING context; a prefetch fills a
// different context, so without this it would stream the running job's features
// and partial sums into the next job's banks -- the prefetched job then
// accumulated the previous job's psums and came out wrong.
// out_stream is deliberately left alone: the running job still needs it to
// store, and the job-start path sets it for the new job anyway.
void Dimc_HWPE::configure_fill_streams(int ctx)
{
    JobGeom &g = this->job_geom[ctx];
    const uint32_t kb_one  = g.row_count * DIMC_MACRO_KB_EW;
    const uint32_t fb_one  = DIMC_MACRO_FB_EW;
    const uint32_t out_one = g.row_count * 4;

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        for (uint32_t m = 0; m < g.num_active; m++) {
            const uint32_t slot = b * g.num_active + m;
            blk.weight_stream[m].configure(
                this->job_reg_ctx(ctx, DIMC_HWPE_JOB_KB_SRC_ADDR) + slot * kb_one,
                kb_one,
                this->job_reg_ctx(ctx, DIMC_HWPE_KB_D0_LENGTH),
                this->job_reg_ctx(ctx, DIMC_HWPE_KB_D0_STRIDE),
                this->job_reg_ctx(ctx, DIMC_HWPE_KB_D1_LENGTH),
                this->job_reg_ctx(ctx, DIMC_HWPE_KB_D1_STRIDE), 0, 0, 0);
            blk.input_stream[m].configure(
                this->job_reg_ctx(ctx, DIMC_HWPE_JOB_FB_SRC_ADDR) + slot * fb_one,
                fb_one,
                this->job_reg_ctx(ctx, DIMC_HWPE_FB_D0_LENGTH),
                this->job_reg_ctx(ctx, DIMC_HWPE_FB_D0_STRIDE), 0, 0, 0, 0, 0);
            blk.psin_stream[m].configure(
                this->job_reg_ctx(ctx, DIMC_HWPE_JOB_PSIN_SRC_ADDR) + slot * out_one,
                out_one, 0, 0, 0, 0, 0, 0, 0);
        }
    }
}

void Dimc_HWPE::background_fill()
{
    if (!this->cross_job_prefetch) return;
    if (this->state.get() == DIMC_IDLE) return;
    if (this->prefetch_ready) return;
    // The streamers are free once every active macro of the running job has
    // its operands; from then on they can pull the queued job's.
    if (this->prefetch_ctx < 0) {
        JobGeom &eg = this->job_geom[this->exec_slot];
        for (Dimc_InnerBlock &blk : this->inner_blocks)
            for (uint32_t m = 0; m < eg.num_active; m++)
                if (!blk.macros[m].exec_ready) return;
    }

    if (this->prefetch_ctx < 0) {
        const int head = this->queue_head();
        if (head < 0 || head == (int)this->exec_slot) return;
        this->prefetch_ctx = head;
        this->fill_slot    = (uint32_t)this->prefetch_ctx;
        this->latch_geom(this->prefetch_ctx);
        this->plan_fill(this->prefetch_ctx);
        this->configure_fill_streams(this->prefetch_ctx);
    }

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        // The prefetch owns this queue, so it has to retire it as well. Without
        // this the queue fills to outstanding_depth and the prefetch stalls
        // there for good -- it stopped at 4 of 16 beats on every job.
        retire_due(blk.prefetch_pending, this->fsm_timestamp);
        if (!blk.fill_requested) {
            Dimc_OuterPort *port = this->block_port(b);
            if (port != NULL)
                blk.prefetch_data_ready_cycle = this->outer_cut_through
                    ? (int64_t)this->fsm_timestamp + (int64_t)this->l2_burst_latency
                    : port->request((int64_t)this->fsm_timestamp, this->block_working_set());
            blk.fill_requested = true;
        }
        this->preload_block(blk, b, true);
    }

    bool done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        for (uint32_t m = 0; m < this->job_geom[this->fill_slot].num_active; m++)
            if (blk.macros[m].beat_index < blk.macros[m].beat_total) done = false;
    if (done) this->prefetch_ready = true;
}

int Dimc_HWPE::fsm()
{
    this->background_fill();

    auto next_state = this->state.get();
    int  latency    = 0;

    switch (this->state.get()) {
    case DIMC_STARTING:
        if (this->preload_iter(&latency)) next_state = DIMC_COMPUTING;
        break;

    case DIMC_COMPUTING:
        if (this->compute_iter(&latency)) next_state = DIMC_STORING;
        break;

    case DIMC_STORING:
        if (this->store_iter(&latency)) next_state = DIMC_FINISHED;
        break;

    case DIMC_FINISHED:
        break;

    default:
        this->trace.fatal("DIMC HWPE FSM: UNKNOWN STATE (%d)!\n", this->state.get());
    }

    if (next_state != this->state.get()) {
        uint8_t st = (uint8_t)next_state;
        this->state_event.event(&st);
    }
    this->state.set(next_state);
    return latency;
}

// ---- Engine phase iterators ----
// One control plane drives N inner blocks. Each phase advances every block by
// one cycle of work, then ticks the job timestamp once; the phase ends when
// all blocks are done. The per-block steps never touch fsm_timestamp: bumping
// it per block would make N blocks look N times slower.
//
// The outer stage is not a closed-form correction at the end of the job.
// Blocks reserve a Dimc_OuterPort carrying next_free_cycle, like
// interco/router's BandwidthLimiter, so a block hitting a busy port lands
// later. Sharing and serialization are emergent.

void Dimc_HWPE::phase_end_reset()
{
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.beat_total  = 0;
        blk.beat_index  = 0;
        blk.rows_issued = 0;
        blk.phase_done  = false;
    }
}

void Dimc_HWPE::beat_issued(Dimc_InnerBlock &blk, int lat, bool prefetch)
{
    if (lat < 1) lat = 1;
    if (prefetch) {
        blk.prefetch_pending.push(this->fsm_timestamp + (uint64_t)lat);
        blk.prefetch_beat_index++;
        return;
    }
    blk.pending_req_queue.push(this->fsm_timestamp + (uint64_t)lat);
    blk.beat_index++;
    blk.beat_event.event((uint8_t *)&blk.beat_index);
}


// ================= Outer port =================
void Dimc_OuterPort::configure(uint32_t bandwidth, uint32_t latency)
{
    this->bandwidth_bytes = bandwidth ? bandwidth : 1;
    this->burst_latency   = latency;
    this->next_free_cycle = 0;
    this->cursor_bytes    = 0;
}

void Dimc_OuterPort::reset()
{
    this->next_free_cycle = 0;
    this->cursor_bytes    = 0;
}

int64_t Dimc_OuterPort::busy_until() const
{
    return this->cursor_bytes / (int64_t)this->bandwidth_bytes;
}

// Reserve the port for `bytes` no earlier than `now`; return the cycle the data
// has landed. A client arriving while the port is still busy is pushed out by
// however long the previous burst still runs. That is the whole contention
// mechanism, the same one interco/router uses.
int64_t Dimc_OuterPort::request(int64_t now, uint64_t bytes)
{
    // Reserve in bytes, report in cycles. For a whole-working-set request whose
    // size is a multiple of the port width this is identical to the previous
    // cycle-granular reservation; it differs only for beats narrower than the
    // port, which the cut-through path issues.
    int64_t start = std::max(now * (int64_t)this->bandwidth_bytes, this->cursor_bytes);
    this->cursor_bytes = start + (int64_t)bytes;
    this->next_free_cycle =
        (this->cursor_bytes + this->bandwidth_bytes - 1) / this->bandwidth_bytes;
    uint32_t nf = (uint32_t)this->next_free_cycle;
    this->free_event.event((uint8_t *)&nf);
    return this->next_free_cycle + (int64_t)this->burst_latency;
}

// Shared topology -> every block indexes the one port. Independent -> one each.
Dimc_OuterPort *Dimc_HWPE::block_port(uint32_t blk)
{
    if (this->outer_ports.empty()) return NULL;
    if (this->outer_ports.size() == 1) return &this->outer_ports[0];
    return &this->outer_ports[blk % this->outer_ports.size()];
}

// Bytes one inner block must pull through the outer port for this job. On reuse
// the weights are already resident in the IMC array, so only features travel.
uint64_t Dimc_HWPE::block_working_set() const
{
    uint64_t psin_bytes = this->job_geom[this->fill_slot].psin_rows ? (uint64_t)this->job_geom[this->fill_slot].row_count * 4 : 0;
    uint64_t per_macro = this->job_geom[this->fill_slot].skip_kb
        ? ((uint64_t)DIMC_MACRO_FB_EW + psin_bytes)
        : ((uint64_t)this->job_geom[this->fill_slot].row_count * DIMC_MACRO_KB_EW + DIMC_MACRO_FB_EW + psin_bytes);
    return (uint64_t)this->job_geom[this->fill_slot].num_active * per_macro;
}

// ================= STARTING =================
bool Dimc_HWPE::preload_iter(int *latency)
{
    *latency = 1;
    const uint32_t port_bytes = this->inner_port_bytes;

    // ---- first cycle of the phase: latch the job shape, plan every block ----
    if (!this->phase_planned) {
        const uint32_t slot = this->exec_slot;
        if (!this->job_prefetched) {
            this->fill_slot = slot;
            this->latch_geom((int)slot);
            this->plan_fill((int)slot);
        }
        JobGeom &g = this->job_geom[slot];
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            for (uint32_t m = 0; m < g.num_active; m++) {
                blk.macros[m].rows_issued  = 0;
                blk.macros[m].rows_retired = 0;
                blk.macros[m].job_slot = slot;
            }
            // The store is planned here, not at the start of DIMC_STORING: its
            // beats now issue from inside this phase, as soon as the rows they
            // carry retire.
            g.out_beats = (g.row_count * 4 + port_bytes - 1) / port_bytes;
            blk.store_beat_total = g.num_active * g.out_beats;
            blk.store_beat_index = 0;
            blk.out_buf.assign(g.num_active,
                               std::vector<uint8_t>(g.row_count * 4, 0));
            blk.load_done.assign(g.num_active, 0);
        }
        // A prefetched job pays no burst head: its data is already resident.
        if (!this->job_prefetched)
            this->fsm_timestamp += this->tcdm_burst_latency;
        this->phase_planned = true;
    }

    // ---- outer-port fills ----
    // Every block asks for its working set when the job starts. Whether the
    // fills overlap or serialize is decided by the port, not here: a shared
    // port hands out later completion times, independent ports do not.
    // Per-block buffer depth (single vs ping-pong L1) is not modelled: there is
    // one fill per block per job, so depth could only matter across triggers.
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        if (blk.fill_requested) continue;
        Dimc_OuterPort *port = this->block_port(b);
        if (port != NULL) {
            blk.data_ready_cycle = this->outer_cut_through
                ? (int64_t)this->fsm_timestamp + (int64_t)this->l2_burst_latency
                : port->request((int64_t)this->fsm_timestamp, this->block_working_set());
            uint32_t dr = (uint32_t)blk.data_ready_cycle;
            blk.ready_event.event((uint8_t *)&dr);
        }
        blk.fill_requested = true;
    }

    // ---- one beat per block per cycle ----
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        this->preload_block(blk, b, false);  // one beat, to the first macro still owing
        this->compute_indep(blk);     // every macro whose own fill has landed
        this->store_block(blk, b);    // and ship whatever has already retired
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        retire_due(blk.pending_req_queue, this->fsm_timestamp);
        bool done = true;
        for (uint32_t m = 0; m < this->job_geom[this->fill_slot].num_active; m++) {
            Dimc_Macro &mac = blk.macros[m];
            if (mac.rows_issued < this->job_geom[this->fill_slot].row_count || !mac.pipe.empty()) done = false;
        }
        blk.phase_done = done;
        if (!done) all_done = false;
    }

    if (!all_done) return false;

    this->phase_end_reset();
    this->phase_planned = false;
    return true;
}

// Issue at most one beat of one inner block. The linear beat_index is the only
// position state; (macro, row, offset) are derived, so they cannot drift apart.
// Phase completion is decided by the caller from beat_index and the pending
// queue, not here, so there is nothing to report back.
void Dimc_HWPE::preload_block(Dimc_InnerBlock &blk, uint32_t blk_id, bool prefetch)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    uint32_t &blk_beat_index = prefetch ? blk.prefetch_beat_index : blk.beat_index;
    uint32_t &blk_beat_total = prefetch ? blk.prefetch_beat_total : blk.beat_total;
    std::queue<uint64_t> &blk_pending = prefetch ? blk.prefetch_pending : blk.pending_req_queue;
    const int64_t ready = prefetch ? blk.prefetch_data_ready_cycle : blk.data_ready_cycle;

    // The block's data has not landed from the outer port yet. Under
    // store-and-forward data_ready_cycle is the whole working set; under
    // cut-through it is only the fixed round trip, and each beat is admitted
    // individually below.
    if ((int64_t)this->fsm_timestamp < ready) return;

    Dimc_OuterPort *oport = this->outer_cut_through ? this->block_port(blk_id) : NULL;
    if (oport != NULL && oport->busy_until() > (int64_t)this->fsm_timestamp) return;

    if (blk_beat_index >= blk_beat_total ||
        blk_pending.size() >= this->outstanding_depth) return;

    // Lowest-index macro that still owes beats. Same order the block-wide
    // cursor produced, so this substitution changes nothing by itself.
    uint32_t macro = 0;
    while (macro < this->job_geom[this->fill_slot].num_active &&
           blk.macros[macro].beat_index >= blk.macros[macro].beat_total) macro++;
    if (macro >= this->job_geom[this->fill_slot].num_active) return;
    uint32_t within = blk.macros[macro].beat_index;
    if (within == 0)
        blk.macros[macro].trace_fill_start =
            (uint32_t)(this->fsm_timestamp - this->job_start_cycle);
    uint32_t kb_span = this->job_geom[this->fill_slot].skip_kb ? 0
                     : this->job_geom[this->fill_slot].row_count * this->job_geom[this->fill_slot].kb_beats_per_row;
    int lat;

    if (within < kb_span) {                       // ---- kernel beat ----
        uint32_t row  = within / this->job_geom[this->fill_slot].kb_beats_per_row;
        uint32_t sub  = within % this->job_geom[this->fill_slot].kb_beats_per_row;
        uint32_t off  = sub * port_bytes;
        uint32_t w    = DIMC_MACRO_KB_EW - off;
        if (w > port_bytes) w = port_bytes;
        lat = blk.weight_stream[macro].issue_beat((int)w, blk.macros[macro].row_buffer + off);
        if (sub == this->job_geom[this->fill_slot].kb_beats_per_row - 1) {  // row complete -> commit
            uint32_t row_idx = (this->job_geom[this->fill_slot].row_base + row) % DIMC_MACRO_KB_LEN;
            blk.macros[macro].write_row((int)row_idx, blk.macros[macro].row_buffer);
        }
    } else if (within < kb_span + this->job_geom[this->fill_slot].fb_beats_per_macro) {  // ---- feature beat ----
        uint32_t sub = within - kb_span;
        uint32_t off = sub * port_bytes;
        uint32_t w   = DIMC_MACRO_FB_EW - off;
        if (w > port_bytes) w = port_bytes;
        lat = blk.input_stream[macro].issue_beat((int)w, blk.macros[macro].row_buffer + off);
        if (sub == this->job_geom[this->fill_slot].fb_beats_per_macro - 1) { // feature complete
            blk.macros[macro].write_fb(blk.macros[macro].row_buffer);
            // pipe/kb_ready/fb_ready/psin_scalar belong to the job that is
            // EXECUTING, not to the one being filled. A prefetch runs while the
            // previous job is still computing, so clearing its pipe here threw
            // away rows already in flight -- whole output beats came back zero.
            // A prefetched job gets them set at job start instead.
            if (!prefetch) {
                blk.macros[macro].kb_ready = true;
                blk.macros[macro].fb_ready = true;
                blk.macros[macro].pipe.clear();
                blk.macros[macro].psin_scalar = (int32_t)this->job_reg(DIMC_HWPE_PSIN);
                // Job-relative, like the makespan it is traced beside.
                blk.load_done[macro] =
                    (uint32_t)(this->fsm_timestamp + 1 - this->job_start_cycle);
            }
        }
    } else {                                      // ---- partial-sum beat ----
        uint32_t sub  = within - kb_span - this->job_geom[this->fill_slot].fb_beats_per_macro;
        uint32_t off  = sub * port_bytes;
        uint32_t left = this->job_geom[this->fill_slot].row_count * 4 - off;
        uint32_t w    = left > port_bytes ? port_bytes : left;
        lat = blk.psin_stream[macro].issue_beat((int)w, blk.macros[macro].row_buffer + off);
        // A beat carries several rows; commit them once the last one lands.
        if (sub == this->job_geom[this->fill_slot].psin_beats_per_macro - 1) {
            for (uint32_t r = 0; r < this->job_geom[this->fill_slot].row_count; r++) {
                uint32_t row_idx = (this->job_geom[this->fill_slot].row_base + r) % DIMC_MACRO_KB_LEN;
                blk.macros[macro].write_psin_row((int)row_idx, blk.macros[macro].row_buffer + r * 4);
            }
        }
    }

    // Charge the beat to the outer port. Two 32-byte beats fit in one cycle of
    // a 64 B/cycle port, so both blocks can issue every cycle and the port runs
    // full instead of alternating whole working sets.
    if (oport != NULL) oport->request((int64_t)this->fsm_timestamp, port_bytes);

    blk.macros[macro].beat_index++;
    if (blk.macros[macro].beat_index >= blk.macros[macro].beat_total) {
        blk.macros[macro].fill_done_cycle = this->fsm_timestamp + (uint64_t)lat;
        blk.macros[macro].trace_fill_done =
            (uint32_t)(this->fsm_timestamp + (uint64_t)lat - this->job_start_cycle);
        // Hand the freshly filled banks to the compute side. The kernel bank
        // only flips when a kernel actually moved: a reuse job leaves the
        // weights where they are, so flipping it would compute against a bank
        // that was never written.
        // Only the running job's own load may swap banks here. A prefetch
        // flips at the moment its job starts, not when its data lands, or it
        // would pull the current job's operands out from under it.
        if (this->prefetch_ctx >= 0) return;
        Dimc_Macro &mc = blk.macros[macro];
        mc.exec_ready = true;
        mc.last_kb_src = this->job_geom[this->fill_slot].kb_src;   // the fill landed
        mc.fb_cur  = mc.fb_fill;
        mc.fb_fill = (uint8_t)(mc.fb_fill ^ 1u);
        // The aggregate decision, not mc.skip_kb: kb_span is one value for the
        // whole job, so either every macro pulled kernel beats or none did. A
        // macro that flipped on its own opinion would compute from the bank the
        // load did not go to.
        if (!this->job_geom[this->fill_slot].skip_kb) {
            mc.kb_cur  = mc.kb_fill;
            mc.kb_fill = (uint8_t)(mc.kb_fill ^ 1u);
        }
    }
    this->beat_issued(blk, lat, prefetch);
}

// ================= COMPUTING =================
// The rows are issued and drained inside the fill phase now, one macro at a
// time as each finishes its own beats -- that overlap is the point. Nothing is
// left to do here, so the state exists only to keep the reported sequence
// IDLE -> STARTING -> COMPUTING -> STORING -> FINISHED intact for the traces.
bool Dimc_HWPE::compute_iter(int *latency)
{
    *latency = 1;
    return true;
}

// Each macro issues its own rows as soon as its own fill has landed, so a
// macro still pulling beats through the shared inner port does not hold back a
// sibling that is ready to compute. That overlap is the inner double buffer.
void Dimc_HWPE::compute_indep(Dimc_InnerBlock &blk)
{
    const uint32_t num_active = this->job_geom[this->exec_slot].num_active;
    const uint32_t row_count  = this->job_geom[this->exec_slot].row_count;

    this->drain_ready_rows(blk);

    for (uint32_t m = 0; m < num_active; m++) {
        Dimc_Macro &mac = blk.macros[m];
        bool filled = mac.exec_ready
                   && this->fsm_timestamp >= mac.fill_done_cycle;
        if (filled && mac.rows_issued < row_count && mac.can_accept()) {
            uint32_t row_idx = (this->job_geom[this->exec_slot].row_base + mac.rows_issued) % DIMC_MACRO_KB_LEN;
            if (mac.rows_issued == 0)
                mac.trace_compute_start = (uint32_t)(this->fsm_timestamp - this->job_start_cycle);
            mac.issue((int)row_idx, (int)mac.rows_issued, this->job_geom[this->exec_slot].bias);
            mac.rows_issued++;
            if (mac.rows_issued == row_count)
                mac.trace_compute_end = (uint32_t)(this->fsm_timestamp - this->job_start_cycle);
            if (m == 0) blk.rows_event.event((uint8_t *)&mac.rows_issued);
        }
        mac.tick();
    }
}

void Dimc_HWPE::drain_ready_rows(Dimc_InnerBlock &blk)
{
    const uint32_t out_w = 4;
    for (uint32_t m = 0; m < this->job_geom[this->exec_slot].num_active; m++) {
        uint8_t *out_buf = blk.out_buf[m].data();
        while (blk.macros[m].has_ready()) {
            DimcPipeEntry e = blk.macros[m].drain();
            uint32_t psout_u = (uint32_t)e.psout;
            std::memcpy(out_buf + e.job_row * out_w, &psout_u, out_w);
            blk.macros[m].rows_retired++;
            // The same pop clocks the output accumulator (cleopatra.sv).
            blk.out_accum.push(e.psout);
        }
    }
}

// Advance one block's macro pipelines by a cycle. The macros inside a block run
// in lockstep, so a row enters all of them or none of them, and the row counter
// advances once, never per macro.
bool Dimc_HWPE::compute_block(Dimc_InnerBlock &blk)
{
    const uint32_t num_active = this->job_geom[this->exec_slot].num_active;
    const uint32_t row_count  = this->job_geom[this->exec_slot].row_count;

    this->drain_ready_rows(blk);

    if (blk.rows_issued < row_count) {
        bool all_ready = true;
        for (uint32_t m = 0; m < num_active; m++)
            if (!blk.macros[m].can_accept()) { all_ready = false; break; }
        if (all_ready) {
            uint32_t row_idx = (this->job_geom[this->exec_slot].row_base + blk.rows_issued) % DIMC_MACRO_KB_LEN;
            for (uint32_t m = 0; m < num_active; m++)
                blk.macros[m].issue((int)row_idx, (int)blk.rows_issued, this->job_geom[this->exec_slot].bias);
            blk.rows_issued++;
            blk.rows_event.event((uint8_t *)&blk.rows_issued);
        }
    }
    for (uint32_t m = 0; m < num_active; m++) blk.macros[m].tick();

    bool pipes_empty = true;
    for (uint32_t m = 0; m < num_active; m++)
        if (!blk.macros[m].pipe.empty()) { pipes_empty = false; break; }

    // No second drain here: pipes_empty means every has_ready() is false, and
    // tick() only decrements, so a ready entry would still be in its pipe.
    return blk.rows_issued >= row_count && pipes_empty;
}

// ================= STORING =================
bool Dimc_HWPE::store_iter(int *latency)
{
    *latency = 1;
    const uint32_t num_active = this->job_geom[this->exec_slot].num_active;
    const uint32_t row_count  = this->job_geom[this->exec_slot].row_count;
    const uint32_t out_bytes  = row_count * 4;

    if (!this->phase_planned) {
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            // Results leave through the same outer port, so a shared port
            // serialises the drains too. The drain is requested at the start of
            // the store phase and runs in parallel with the inner store beats:
            // per the D-tile spec the hardware overlaps drain with loading, it
            // does not wait for the whole result to reach L1. Unlike the input
            // fill, store is not store-and-forward.
            Dimc_OuterPort *port = this->block_port(b);
            if (port != NULL && !this->outer_cut_through) {
                uint64_t out_ws = (uint64_t)num_active * (uint64_t)out_bytes;
                blk.data_ready_cycle =
                    port->request((int64_t)this->fsm_timestamp, out_ws);
            } else {
                // Cut-through charges the port beat by beat as the beats issue.
                blk.data_ready_cycle = 0;
            }
            uint32_t dr = (uint32_t)blk.data_ready_cycle;
            blk.drain_event.event((uint8_t *)&dr);
        }
        // Only pay the drain head if nothing has gone out yet; a store that
        // already started inside the compute phase paid it there.
        if (this->inner_blocks[0].store_beat_index == 0)
            this->fsm_timestamp += this->tcdm_burst_latency;
        this->phase_planned = true;
    }

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        if (!blk.phase_done) this->store_block(blk, b);
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        size_t after = retire_due(blk.pending_req_queue, this->fsm_timestamp);
        bool beats_done = (blk.store_beat_index >= blk.store_beat_total) && (after == 0);
        bool drain_landed = (int64_t)this->fsm_timestamp >= blk.data_ready_cycle;
        if (beats_done && drain_landed) blk.phase_done = true;
        else                            all_done = false;
    }

    if (!all_done) return false;

    // ---- job closed: the elapsed cycles ARE the makespan, for the whole
    // outer block. No formula is applied on top: the outer port already
    // charged its bandwidth and its serialization while the job ran.
    this->phase_end_reset();
    this->phase_planned = false;

    uint64_t finish      = this->fsm_timestamp - this->job_start_cycle;
    uint32_t compute_cyc = this->job_geom[this->exec_slot].compute_cyc;
    bool     skip_kb     = this->job_geom[this->exec_slot].skip_kb;

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        for (uint32_t m = 0; m < num_active; m++) {
            this->trace.msg(vp::TraceLevel::WARNING,
                "macro[%u]: fill=[%u..%u] comp=[%u..%u] beats=%u skip_kb=%d "
                "load_done=%u compute=%u OUT=%u finish=%lu\n",
                b * num_active + m,
                blk.macros[m].trace_fill_start, blk.macros[m].trace_fill_done,
                blk.macros[m].trace_compute_start, blk.macros[m].trace_compute_end,
                blk.macros[m].beat_total, (int)skip_kb,
                blk.load_done[m], compute_cyc, blk.out_beat_lat_est, finish);
        }
        if (blk.out_accum.enable) {
            this->trace.msg(vp::TraceLevel::WARNING,
                "block[%u] out_accum: acc=%d n=%u\n",
                b, blk.out_accum.acc, blk.out_accum.count);
        }
    }

    // Each phase advanced the clock one cycle per beat, so job start to here
    // already took `finish` cycles. The completion event therefore fires after
    // 1 cycle, not after `finish` more, which would count them twice. `finish`
    // only goes to the trace.
    *latency = 1;
    this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC double-buffer: num_active=%u row_count=%u l1bw=%u "
        "reuse=%u pf=%d pfctx=%d nextctx=%d pfbeat=%u/%u rdy=%d total_latency=%lu\n",
        num_active, row_count, this->inner_port_bytes,
        (unsigned)skip_kb, (int)this->job_prefetched, this->prefetch_ctx,
        this->queue_head(), this->inner_blocks[0].prefetch_beat_index,
        this->inner_blocks[0].prefetch_beat_total, (int)this->prefetch_ready,
        (unsigned long)finish);
    return true;
}

void Dimc_HWPE::store_block(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t row_count  = this->job_geom[this->exec_slot].row_count;
    const uint32_t out_bytes  = row_count * 4;
    const uint32_t out_beats  = this->job_geom[this->exec_slot].out_beats;

    if (blk.store_beat_index >= blk.store_beat_total ||
        blk.pending_req_queue.size() >= this->outstanding_depth) return;

    uint32_t macro = blk.store_beat_index / out_beats;
    uint32_t sub   = blk.store_beat_index % out_beats;
    uint32_t off   = sub * port_bytes;
    uint32_t w     = out_bytes - off;
    if (w > port_bytes) w = port_bytes;

    // A beat carries port_bytes/4 rows. It may only go once this macro has
    // retired every row it covers -- drain_ready_rows writes them into out_buf
    // as they pop, so sending early would ship whatever was in the buffer
    // before. This is what lets the store run inside the compute phase instead
    // of waiting for the last macro to finish.
    const uint32_t rows_per_beat = port_bytes / 4;
    uint32_t need = (sub + 1) * rows_per_beat;
    if (need > row_count) need = row_count;
    if (blk.macros[macro].rows_retired < need) return;

    // Same admission as the fill: the beat is charged to the outer port as it
    // issues, so store traffic and a concurrent prefetch share the port.
    Dimc_OuterPort *oport = this->outer_cut_through ? this->block_port(blk_id) : NULL;
    if (oport != NULL && oport->busy_until() > (int64_t)this->fsm_timestamp) return;

    int lat = blk.out_stream[macro].issue_beat((int)w, blk.out_buf[macro].data() + off);
    blk.out_beat_lat_est = (uint32_t)((lat < 1 ? 1 : lat) * (int)out_beats);
    if (oport != NULL) oport->request((int64_t)this->fsm_timestamp, w);

    if (lat < 1) lat = 1;
    blk.pending_req_queue.push(this->fsm_timestamp + (uint64_t)lat);
    blk.store_beat_index++;
}
