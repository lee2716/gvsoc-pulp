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

    // Clear STATUS at job start so a back-to-back trigger (e.g. reuse without a
    // soft_clear) does not see the previous job's STATUS=1 and exit polling early.
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x0;

    // Streamers: the input ones follow the fill (plan_macro_fill), the output ones the
    // sink (configure_out_stream at each run's first result).

    // Broadcast the job-independent compute configuration to every macro (compe,
    // sel_dimc). ci, sign_8b and compute_mask follow the job (write_feed).
    uint8_t  compe     = (uint8_t) (_this->register_file[DIMC_HWPE_COMPE        >> 2] & 0x1);
    _this->sel_dimc   = (uint8_t)(_this->register_file[DIMC_HWPE_SEL_DIMC  >> 2] & 0xFF);
    for (Dimc_InnerBlock &blk : _this->inner_blocks)
    for (auto &m : blk.macros) {
        // `compe` (memory-vs-compute mode) is latched but never read: compute_PP
        // always performs the dot product, and COMPE=0 memory mode is not
        // implemented.
        // psin_rows, ci, sign_8b and compute_mask are per job and latched with the macro's
        // fill (write_feed), since a macro may compute the next job before that job starts here.
        m.compe = compe;
    }
    // `sel_dimc` is stored but never consulted: macro selection goes through
    // NUM_MACROS (num_active). It exists for register-map fidelity.

    // job_running / running_job were latched by start_next_job() at commit.
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x0;   // busy

    // fsm_timestamp runs free across jobs: port_pending holds absolute
    // due-stamps, so zeroing it while a beat is in flight would strand that
    // beat. job_start_cycle is what the makespan trace subtracts to stay
    // per-job.
    _this->job_start_cycle = _this->fsm_timestamp;
    _this->phase_planned = false;
    _this->exec_slot = (uint32_t)(_this->running_ctx >= 0 ? _this->running_ctx : 0);

    // A fill already running for this job raises kb_ready/fb_ready before its last beat
    // stamps filled_job, so clearing here would strand that macro. Compute is gated on
    // filled_job == running_job either way.
    for (Dimc_InnerBlock &blk : _this->inner_blocks)
        for (uint32_t m = 0; m < _this->num_macros; m++) {
            Dimc_Macro &mc = blk.macros[m];
            // A macro already filled, filling or being written for this job keeps its buffers.
            if (mc.filled_job == _this->running_job || mc.fill_job == _this->running_job
                || mc.write_job == _this->running_job) continue;
            mc.exec_ready  = false;
            // Cleared here, not left over from the previous job: they gate
            // compute, and this job's feature vector has not landed yet.
            mc.kb_ready    = false;
            mc.fb_ready    = false;
            mc.filled_job  = Dimc_Macro::JOB_NONE;
        }

    for (Dimc_InnerBlock &blk : _this->inner_blocks)
        blk.reset_job_state(_this->fill_active);

    _this->state.set(DIMC_STARTING);
    _this->tracer.job_start((uint64_t)_this->clock.get_cycles());

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

    // Retire the context this job used, then launch whatever is queued behind it.
    if (_this->running_ctx >= 0) _this->ctx_busy[_this->running_ctx] = false;
    _this->running_ctx = -1;
    _this->job_running = false;
    _this->finished_jobs++;
    _this->register_file[DIMC_HWPE_STATUS    >> 2] = 0x1;                 // done
    _this->register_file[DIMC_HWPE_FIN_JOBS  >> 2] = _this->finished_jobs;
    // Before start_next_job, which reports the next job as queued.
    _this->tracer.job_end();
    // Standard HWPE completion interrupt (pulse), if the line is wired.
    if (_this->irq.is_bound()) {
        _this->irq.sync(true);
        _this->irq.sync(false);
    }
    // autotrigger_n: 0 = chain into the next queued job automatically,
    // 1 = hold the queue until SW issues an explicit trigger (commit_trigger 0/2).
    if ((_this->register_file[DIMC_HWPE_AUTOTRIGGER_N >> 2] & 0x1) == 0)
        _this->start_next_job();
    // The macros already on the next job keep filling and computing through the
    // cycle this bookkeeping takes; only the controller is between jobs.
    _this->handover_step();
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

// Size one job's fill: geometry into its slot and the per-macro beat budgets.
void Dimc_HWPE::latch_geom(int ctx)
{
    JobGeom &g = this->job_geom[ctx];
    uint32_t n = this->register_file[DIMC_HWPE_NUM_MACROS >> 2];
    if (n == 0 || n > this->num_macros) n = this->num_macros;
    uint32_t r = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_COUNT);
    if (r == 0) {
        this->trace.force_warning("latch_geom: ctx %d has ROW_COUNT=0 "
            "(never configured); running_ctx=%d qlen=%u\n",
            ctx, this->running_ctx,
            (unsigned)this->ctx_queue.size());
        r = 1;
    }
    if (r > DIMC_MACRO_KB_LEN) r = DIMC_MACRO_KB_LEN;

    g.num_active  = n;
    g.row_count   = r;
    g.row_base    = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_SEL_BASE);
    // CFG_BIAS is not read: the RTL's only addend is ADDIN, which PSIN models.
    g.compute_cyc = r + DIMC_MACRO_LATENCY;
    g.psin_rows   = this->job_reg_ctx(ctx, DIMC_HWPE_PSIN_EN) & 0x1;
    g.nb_vec         = this->job_reg_ctx(ctx, DIMC_HWPE_NB_VEC);
    if (g.nb_vec == 0) g.nb_vec = 1;
    g.fb_vec_stride  = this->job_reg_ctx(ctx, DIMC_HWPE_FB_VEC_STRIDE);
    g.ps_vec_stride  = this->job_reg_ctx(ctx, DIMC_HWPE_PSIN_VEC_STRIDE);
    g.out_vec_stride = this->job_reg_ctx(ctx, DIMC_HWPE_OUT_VEC_STRIDE);
    this->geom_job[ctx] = this->ctx_job_id[ctx];

    g.psin_dep   = this->job_reg_ctx(ctx, DIMC_HWPE_PSIN_DEP);
}

uint32_t Dimc_HWPE::slot_offset(const JobGeom &g, uint32_t blk_id, uint32_t m, uint8_t kind) const
{
    // Slot slicing: one kernel block and one output block per slot, one feature block for all.
    const uint32_t slot = blk_id * g.num_active + m;
    switch (kind) {
    case DIMC_LOAD_KB: return slot * g.row_count * DIMC_MACRO_KB_EW;
    case DIMC_LOAD_FB: return 0;
    default:           return slot * g.row_count * DIMC_OUT_SLOT_BYTES;
    }
}

// The input streamers read the operands of the job and vector (run) being FILLED, not
// the running one: the fill runs ahead. Per macro, so macros on different jobs or runs
// do not share a base.
void Dimc_HWPE::configure_macro_streams(uint32_t blk_id, uint32_t m, int ctx, uint32_t run,
                                        bool kernel, bool feature, bool psum)
{
    const JobGeom &g = this->job_geom[ctx];
    Dimc_InnerBlock &blk = this->inner_blocks[blk_id];
    const uint32_t kb_one  = g.row_count * DIMC_MACRO_KB_EW;
    const uint32_t fb_one  = DIMC_MACRO_FB_EW;
    const uint32_t out_one = g.row_count * DIMC_OUT_SLOT_BYTES;   // psin reads output slots
    if (kernel)
        blk.weight_stream[m].configure(
            this->job_reg_ctx(ctx, DIMC_HWPE_JOB_KB_SRC_ADDR)
                + this->slot_offset(g, blk_id, m, DIMC_LOAD_KB), kb_one,
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D0_LENGTH),
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D0_STRIDE),
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D1_LENGTH),
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D1_STRIDE), 0, 0, 0);
    if (feature)
        blk.input_stream[m].configure(
            this->job_reg_ctx(ctx, DIMC_HWPE_JOB_FB_SRC_ADDR) + run * g.fb_vec_stride
                + this->slot_offset(g, blk_id, m, DIMC_LOAD_FB), fb_one,
            this->job_reg_ctx(ctx, DIMC_HWPE_FB_D0_LENGTH),
            this->job_reg_ctx(ctx, DIMC_HWPE_FB_D0_STRIDE), 0, 0, 0, 0, 0);
    // Per-row partial sums, laid out like the outputs they came from.
    if (psum)
        blk.psin_stream[m].configure(
            this->job_reg_ctx(ctx, DIMC_HWPE_JOB_PSIN_SRC_ADDR) + run * g.ps_vec_stride
                + this->slot_offset(g, blk_id, m, DIMC_LOAD_PSIN), out_one, 0, 0, 0, 0, 0, 0, 0);
}

void Dimc_HWPE::configure_fill_streams(int ctx)
{
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++)
        for (uint32_t m = 0; m < this->job_geom[ctx].num_active; m++)
            this->configure_macro_streams(b, m, ctx, 0, true, true, true);
}

// The output streamer of run (macro, vector) of the job in context ctx, which need not be
// the running one.
void Dimc_HWPE::configure_out_stream_ctx(uint32_t blk_id, uint32_t m, uint32_t run, uint32_t ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    const uint32_t out_one = g.row_count * DIMC_OUT_SLOT_BYTES;
    this->inner_blocks[blk_id].out_stream[m].configure(
        this->job_reg_ctx((int)ctx, DIMC_HWPE_JOB_DST_ADDR) + run * g.out_vec_stride
            + this->slot_offset(g, blk_id, m, DIMC_LOAD_NONE),
        out_one, this->job_reg_ctx((int)ctx, DIMC_HWPE_OUT_D0_LENGTH),
        this->job_reg_ctx((int)ctx, DIMC_HWPE_OUT_D0_STRIDE), 0, 0, 0, 0, 0);
}

// The output streamer of run (macro, vector) of the running job.
void Dimc_HWPE::configure_out_stream(uint32_t blk_id, uint32_t m, uint32_t run)
{
    const JobGeom &g = this->job_geom[this->exec_slot];
    const uint32_t out_one = g.row_count * DIMC_OUT_SLOT_BYTES;
    this->inner_blocks[blk_id].out_stream[m].configure(
        this->job_reg(DIMC_HWPE_JOB_DST_ADDR) + run * g.out_vec_stride
            + this->slot_offset(g, blk_id, m, DIMC_LOAD_NONE),
        out_one, this->job_reg(DIMC_HWPE_OUT_D0_LENGTH),
        this->job_reg(DIMC_HWPE_OUT_D0_STRIDE), 0, 0, 0, 0, 0);
}

// Beats per macro for one job: [kernel rows if fresh] + NB_VEC x (partial sums + feature).
void Dimc_HWPE::size_fill(int ctx)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    JobGeom &g = this->job_geom[ctx];
    g.kb_beats_per_row   = (DIMC_MACRO_KB_EW + port_bytes - 1) / port_bytes;
    g.fb_beats_per_macro = (DIMC_MACRO_FB_EW + port_bytes - 1) / port_bytes;
    // Partial sums are read from the slots a previous job wrote, port_bytes / SLOT rows
    // per port word. Per vector.
    g.psin_beats_per_macro = g.psin_rows
        ? (g.row_count * DIMC_OUT_SLOT_BYTES + port_bytes - 1) / port_bytes : 0;
    g.beats_per_macro = g.row_count * g.kb_beats_per_row
                      + g.nb_vec * (g.fb_beats_per_macro + g.psin_beats_per_macro);
}

// Start macro m's fill program for the job in context ctx (already latched and sized).
void Dimc_HWPE::plan_macro_fill(uint32_t blk_id, uint32_t m, int ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    Dimc_InnerBlock &blk = this->inner_blocks[blk_id];
    Dimc_InnerBlock::Cursor &cur = blk.fill;
    if (m >= g.num_active) return;
    if (cur.macro_beat_index.size() < blk.macros.size())
        cur.reset((uint32_t)blk.macros.size());
    Dimc_Macro &mc = blk.macros[m];
    mc.fill_job  = this->ctx_job_id[ctx];
    mc.fill_slot = (uint32_t)ctx;
    mc.owed++;
    cur.macro_beat_index[m] = 0;
    cur.macro_beat_total[m] = g.beats_per_macro;
    cur.beat_total += g.beats_per_macro;
    // A macro still holding the running job's operands keeps its stamp: it has rows
    // left to issue against them.
    if (mc.filled_job != this->running_job) mc.fill_done_cycle = 0;
    // A kernel already being streamed ahead for this job keeps its stream position. A job
    // that keeps the kernel in the macro leaves the kernel streamer alone: it may be
    // streaming the kernel of a later job.
    const bool kernel_ahead = mc.kpf_job == mc.fill_job && mc.kfetched > 0;
    this->configure_macro_streams(blk_id, m, ctx, 0, !kernel_ahead, true, true);
}

// Every macro at once, from an empty cursor.
void Dimc_HWPE::plan_fill(int ctx)
{
    this->size_fill(ctx);
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        blk.fill.reset((uint32_t)blk.macros.size());
        for (uint32_t m = 0; m < this->job_geom[ctx].num_active; m++)
            this->plan_macro_fill(b, m, ctx);
    }
}

void Dimc_OuterPort::configure(uint32_t bandwidth)
{
    this->bandwidth_bytes = bandwidth ? bandwidth : 1;
    this->cursor_bytes    = 0;
}

void Dimc_OuterPort::reset()
{
    this->cursor_bytes = 0;
}

int64_t Dimc_OuterPort::busy_until() const
{
    return this->cursor_bytes / (int64_t)this->bandwidth_bytes;
}

void Dimc_OuterPort::request(int64_t now, uint64_t bytes)
{
    // Reserve in bytes, report in cycles: a beat narrower than the port takes a
    // fraction of a cycle, so several of them can share one.
    int64_t start = now * (int64_t)this->bandwidth_bytes;
    if (this->cursor_bytes > start) start = this->cursor_bytes;
    this->cursor_bytes = start + (int64_t)bytes;
}

// Drop every pending beat whose response is due, and report how many are
// still in flight.
static inline size_t retire_due(std::queue<uint64_t> &q, uint64_t now)
{
    while (!q.empty() && q.front() <= now) q.pop();
    return q.size();
}

// Blocks book the shared outer port in turn: the first one served rotates every cycle.
uint32_t Dimc_HWPE::block_order(uint32_t i) const
{
    return (uint32_t)((i + this->fsm_timestamp) % this->inner_blocks.size());
}

// Drop the responses that are due, and advance the sink's write-back position.
void Dimc_HWPE::retire_block(Dimc_InnerBlock &blk)
{
    retire_due(blk.port_pending, this->fsm_timestamp);
    retire_due(blk.kb_pending, this->fsm_timestamp);
    retire_due(blk.in_pending, this->fsm_timestamp);
    while (!blk.run_pending.empty() && blk.run_pending.front().due <= this->fsm_timestamp) {
        const Dimc_InnerBlock::RunDone &d = blk.run_pending.front();
        blk.retired[d.macro].job  = d.job;
        blk.retired[d.macro].runs = d.run + 1;
        blk.run_pending.pop_front();
    }
}

// Run (m, run) of `job` reads the partial sums its own run wrote PSIN_DEP jobs back. Fetch
// them only once that run is written back.
bool Dimc_HWPE::psum_ready(const Dimc_InnerBlock &blk, uint32_t m, const JobGeom &g,
                           uint32_t job, uint32_t run) const
{
    if (!g.psin_rows) return true;
    if (g.psin_dep == 0 || job < g.psin_dep) return true;
    const int64_t pj = (int64_t)(job - g.psin_dep);
    const Dimc_InnerBlock::Retired &p = blk.retired[m];
    return p.job > pj || (p.job == pj && p.runs > run);
}

void Dimc_HWPE::handover_step()
{
    this->fetch_kernels_first();
    for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
        const uint32_t b = this->block_order(i);
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        this->block_cycle(blk, b);
    }
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) this->tracer.end_cycle(b);
    this->tracer.port_cycle();
    this->fsm_timestamp++;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        this->retire_block(blk);
        retire_due(blk.store_pending, this->fsm_timestamp);
    }
}

int Dimc_HWPE::fsm()
{
    auto next_state = this->state.get();
    int  latency    = 0;

    switch (this->state.get()) {
    case DIMC_STARTING:
        // The fill phase also computes and writes back. When it ends with every
        // result written and acknowledged, the job closes in this cycle rather
        // than spending another one in STORING with nothing to do.
        if (this->preload_iter(&latency)) {
            if (this->store_done()) {
                this->close_job();
                next_state = DIMC_FINISHED;
            } else {
                next_state = DIMC_STORING;
            }
        }
        break;

    case DIMC_STORING:
        if (this->store_iter(&latency)) {
            this->close_job();
            next_state = DIMC_FINISHED;
        }
        break;

    case DIMC_FINISHED:
        break;

    default:
        this->trace.fatal("DIMC HWPE FSM: UNKNOWN STATE (%d)!\n", this->state.get());
    }

    if (next_state != this->state.get()) this->tracer.state(next_state);
    this->state.set(next_state);
    return latency;
}

// ---- Engine phase iterators ----
// One control plane drives N inner blocks. Each phase advances every block by
// one cycle of work, then ticks the job timestamp once; the phase ends when
// all blocks are done. The per-block steps never touch fsm_timestamp: bumping
// it per block would make N blocks look N times slower.

void Dimc_HWPE::phase_end_reset()
{
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        if (!this->fill_active) {
            blk.fill.beat_total = 0;
            blk.fill.beat_index = 0;
        }
        blk.rows_issued = 0;
        blk.phase_done  = false;
    }
}


void Dimc_HWPE::advance_fill()
{
    bool any = false;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        if (blk.fill.beat_index < blk.fill.beat_total
            || !blk.wgt_fifo.empty() || !blk.inp_fifo.empty()) any = true;
    this->fill_active = any;
    if (this->ctx_queue.empty()) return;
    const int nxt = this->ctx_queue.front();
    if ((uint32_t)nxt == this->exec_slot) return;
    const uint32_t nxt_job = this->ctx_job_id[nxt];

    // The next job's programs chain from the running job's own.
    if (this->job_geom[this->exec_slot].nb_vec <= 1) {
        // One vector per job: the next job's fill starts once every macro has fetched all of
        // this job's program. Not fill_active: a prefetched kernel in the weight FIFO keeps it set.
        bool spent = true;
        for (const Dimc_InnerBlock &blk : this->inner_blocks)
            if (blk.fill.beat_index < blk.fill.beat_total) spent = false;
        if (spent && this->fill_job == this->running_job) {
            this->latch_geom(nxt);
            this->fill_slot   = (uint32_t)nxt;
            this->fill_job    = nxt_job;
            this->plan_fill(nxt);
            this->fill_active = true;
        }
        return;
    }
    // Batched job: each macro chains on its own as soon as its program for the running job
    // is spent, so it may load the next kernel while its sibling still runs its last vectors.
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        for (uint32_t m = 0; m < blk.macros.size(); m++) {
            Dimc_Macro &mc = blk.macros[m];
            if (mc.fill_job != this->running_job) continue;
            if (blk.fill.macro_beat_index[m] < blk.fill.macro_beat_total[m]) continue;
            if (this->geom_job[nxt] != nxt_job) {
                this->latch_geom(nxt);
                this->size_fill(nxt);
            }
            if (m >= this->job_geom[nxt].num_active) continue;
            this->plan_macro_fill(b, m, nxt);
            this->fill_slot   = (uint32_t)nxt;
            this->fill_job    = nxt_job;
            this->fill_active = true;
        }
    }
}

// Fill program of one macro for one job: vector 0's partial sums and feature, then the
// kernel rows, then vectors 1..NB_VEC-1, each partial sums then feature.
Dimc_HWPE::BeatPos Dimc_HWPE::beat_pos(const JobGeom &g, uint32_t within) const
{
    const uint32_t fb  = g.fb_beats_per_macro;
    const uint32_t ps  = g.psin_beats_per_macro;
    const uint32_t kb  = g.row_count * g.kb_beats_per_row;
    const uint32_t per = ps + fb;
    if (within < ps)       return {DIMC_LOAD_PSIN, 0, within};
    if (within < per)      return {DIMC_LOAD_FB,   0, within - ps};
    if (within < per + kb) return {DIMC_LOAD_KB, 0, within - per};
    within -= kb;
    const uint32_t w = within - per;
    const uint32_t run = 1 + w / per, k = w % per;
    if (k < ps) return {DIMC_LOAD_PSIN, run, k};
    return {DIMC_LOAD_FB, run, k - ps};
}

uint8_t Dimc_HWPE::beat_kind(const JobGeom &g, uint32_t within) const
{
    return this->beat_pos(g, within).kind;
}

// Beats after which the macro may start issuing vector 0: its feature and the kernel
// rows. Later vectors each wait for their own feature (fb_run / fb_ready_cycle).
uint32_t Dimc_HWPE::stamp_beats(const JobGeom &g) const
{
    const uint32_t kb = g.row_count * g.kb_beats_per_row;
    return g.psin_beats_per_macro + g.fb_beats_per_macro + kb;
}

uint32_t Dimc_HWPE::core_beats(const JobGeom &g) const
{
    return this->stamp_beats(g);
}

// The macro holds operands of a job it has not finished triggering. Writing a
// different job's feature or kernel rows into it would change those rows.
bool Dimc_HWPE::holds_unissued(const Dimc_Macro &m) const
{
    return m.exec_ready && m.filled_job != Dimc_Macro::JOB_NONE
        && !(m.issue_job == m.filled_job && m.runs_issued >= m.job_nb_vec);
}

// ================= STARTING =================
bool Dimc_HWPE::preload_iter(int *latency)
{
    *latency = 1;
    const uint32_t port_bytes = this->inner_port_bytes;

    // ---- first cycle of the phase: latch the job shape, plan every block ----
    if (!this->phase_planned) {
        const uint32_t slot = this->exec_slot;
        // Latched once per job: a macro that chained ahead already latched it, and
        // re-latching would rewrite the geometry its cursor was built from.
        if (this->geom_job[slot] != this->running_job) this->latch_geom((int)slot);
        // Plan the macros whose fill program is not yet for this job. None planned: all
        // from an empty cursor.
        auto planned = [&](const Dimc_Macro &mc) -> bool {
            return mc.fill_job == this->running_job;
        };
        bool any_planned = false;
        for (Dimc_InnerBlock &blk : this->inner_blocks)
            for (uint32_t m = 0; m < this->job_geom[slot].num_active; m++)
                if (planned(blk.macros[m])) any_planned = true;
        if (!any_planned) {
            this->fill_slot   = slot;
            this->fill_job    = this->running_job;
            this->plan_fill((int)slot);
            this->fill_active = true;
        } else {
            for (uint32_t b = 0; b < this->inner_blocks.size(); b++)
                for (uint32_t m = 0; m < this->job_geom[slot].num_active; m++)
                    if (!planned(this->inner_blocks[b].macros[m]))
                        this->plan_macro_fill(b, m, (int)slot);
        }
        JobGeom &g = this->job_geom[slot];
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            // The store is planned here, not at the start of DIMC_STORING: its
            // beats now issue from inside this phase, as soon as the rows they
            // carry retire. rows_issued and the result set are reset per macro at its
            // first row of this job (compute_indep): it may already be computing it.
            // Port words per macro: SLOT-byte results, packed port_bytes / SLOT per word.
            g.out_beats = g.nb_vec
                * ((g.row_count * DIMC_OUT_SLOT_BYTES + port_bytes - 1) / port_bytes);
            blk.store.reset((uint32_t)blk.macros.size());
            blk.store.beat_total = g.num_active * g.out_beats;
            // Results of this job that left while the previous one ran.
            blk.store.beat_index = blk.store_next_beats;
            for (uint32_t m = 0; m < blk.macros.size(); m++) {
                blk.out_results[m] = blk.out_results_next[m];
                blk.out_results_next[m] = 0;
            }
            blk.store_next_beats = 0;
            while (!blk.store_next_pending.empty()) {
                blk.store_pending.push(blk.store_next_pending.front());
                blk.store_next_pending.pop();
            }
        }
        this->phase_planned = true;
    }

    // ---- per inner block and cycle: fill, compute, write back ----
    // Each macro follows its own fill program, which may be for a later job than the
    // running one. Whether the blocks' fetches overlap or serialize is decided by the
    // outer port: it books bytes, so blocks whose combined demand fits one cycle all go.
    this->fetch_kernels_first();
    for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
        const uint32_t b = this->block_order(i);
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        this->block_cycle(blk, b);    // one fetch per feed, the macros' rows, one write per FIFO
        this->store_block(blk, b);    // and ship whatever has already retired
    }
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) this->tracer.end_cycle(b);

    // Lowest priority: the next jobs' kernels, only on port bytes the programs left over.
    for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
        const uint32_t b = this->block_order(i);
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        if (blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) continue;
        this->kernel_prefetch(blk, b);
    }
    this->tracer.port_cycle();
    this->fsm_timestamp++;

    this->advance_fill();

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        this->retire_block(blk);
        retire_due(blk.store_pending, this->fsm_timestamp);
        retire_due(blk.store_next_pending, this->fsm_timestamp);
        // Done when every row of the running job has retired. The pipe may already
        // hold the next job's rows, so its emptiness says nothing about this one.
        bool done = true;
        const uint32_t s = this->running_job & 1;
        for (uint32_t m = 0; m < this->job_geom[this->exec_slot].num_active; m++) {
            Dimc_Macro &mac = blk.macros[m];
            if (mac.set_job[s] != this->running_job ||
                mac.rows_retired_set[s] < this->job_geom[this->exec_slot].nb_vec
                                          * this->job_geom[this->exec_slot].row_count) done = false;
        }
        blk.phase_done = done;
        if (!done) all_done = false;
    }

    if (!all_done) return false;

    this->phase_end_reset();
    this->phase_planned = false;
    return true;
}

// One fill beat per feed per cycle. The kernel and input feeds each have their own TCDM
// port and dual FIFO, so both fetch in the same cycle; a macro takes only one of their
// sections per cycle. Partial sums ride the input feed.
void Dimc_HWPE::preload_block(Dimc_InnerBlock &blk, uint32_t blk_id,
                              Dimc_InnerBlock::Cursor &cursor)
{
    // Per feed and cycle: one section fetched from L1 into the dual's FIFO, and one
    // popped from it into a macro. The fetch sees the FIFO before this cycle's pop.
    this->fetch_feeds(blk, blk_id, cursor);
    this->write_feeds(blk, blk_id);
}

void Dimc_HWPE::fetch_feeds(Dimc_InnerBlock &blk, uint32_t blk_id,
                            Dimc_InnerBlock::Cursor &cursor)
{
    if (cursor.beat_index < cursor.beat_total)
        this->fill_feed(blk, blk_id, cursor, false);   // input: feature, partial sums
    if (!blk.kb_fed_first)
        this->fill_feed(blk, blk_id, cursor, true);    // kernel rows, or the next job's ahead
    blk.kb_fed_first = false;
}

// Rows the macro still triggers before vector `run` of `job` may enter its feature buffer.
uint32_t Dimc_HWPE::rows_before_feature(const Dimc_Macro &mc, uint32_t job, uint32_t run) const
{
    // Nothing triggered yet: the first job's vector 0 is next; anything later waits for rows
    // that have not started (the macro's kernel may still be going in).
    if (mc.issue_job == Dimc_Macro::JOB_NONE)
        return !run && (mc.write_job == Dimc_Macro::JOB_NONE || mc.write_job == job) ? 0 : 0xFFFFFFFFu;
    uint32_t target;
    if (mc.issue_job == job)                    target = run;
    else if (mc.issue_job + 1u == job && !run) target = mc.job_nb_vec;
    else                                        return 0xFFFFFFFFu;
    if (mc.runs_issued >= target) return 0;
    const uint32_t cur = mc.rows_issued < mc.job_rows ? mc.rows_issued : 0;
    return (target - mc.runs_issued) * mc.job_rows - cur;
}

// DIMC_KB_FEED_YIELD: a macro whose next input section (partial sums or feature) is for a
// vector it may start within that many rows.
bool Dimc_HWPE::input_urgent() const
{
    for (const Dimc_InnerBlock &blk : this->inner_blocks) {
        const Dimc_InnerBlock::Cursor &c = blk.fill;
        for (uint32_t m = 0; m < blk.macros.size() && m < c.macro_beat_index.size(); m++) {
            if (c.macro_beat_index[m] >= c.macro_beat_total[m]) continue;
            const Dimc_Macro &t = blk.macros[m];
            const BeatPos pos = this->beat_pos(this->job_geom[t.fill_slot], c.macro_beat_index[m]);
            if (pos.kind == DIMC_LOAD_KB) continue;
            if (this->rows_before_feature(t, t.fill_job, pos.run) <= DIMC_KB_FEED_YIELD) return true;
        }
    }
    return false;
}

// DIMC_KB_FEED_FIRST: the kernel feed of each dual about to run out of the kernel being
// written, before the block loop books any input; with DIMC_KB_FEED_YIELD, not while a
// macro is about to wait for its inputs.
void Dimc_HWPE::fetch_kernels_first()
{
    if (!DIMC_KB_FEED_FIRST) return;
    if (DIMC_KB_FEED_YIELD && this->input_urgent()) return;
    for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
        const uint32_t b = this->block_order(i);
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        for (uint32_t m = 0; m < blk.macros.size(); m++) {
            const Dimc_Macro &t = blk.macros[m];
            if (t.write_job == Dimc_Macro::JOB_NONE || t.stamped) continue;
            const JobGeom &g = this->job_geom[t.write_slot];
            const uint32_t kb_all = g.row_count * g.kb_beats_per_row;
            if (t.kw == 0 || t.kw >= kb_all) continue;
            uint32_t n = 0;
            for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo) if (q.macro == m) n++;
            if (n > DIMC_KB_FEED_FIRST) continue;
            this->fill_feed(blk, b, blk.fill, true);
            blk.kb_fed_first = true;
            break;
        }
    }
}

void Dimc_HWPE::write_feeds(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    this->write_feed(blk, blk_id, blk.inp_fifo[0]);   // one inp_pop
    this->write_feed(blk, blk_id, blk.wgt_fifo);
}

// The pops go before the triggers and see last cycle's state: no section enters a macro in
// a cycle in which it computes.
void Dimc_HWPE::block_cycle(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    this->preload_block(blk, blk_id, blk.fill);
    this->compute_indep(blk, blk_id);
}

// May this section be written into the macro now? Kernel and feature sections wait for
// the write port: triggering shuts the kernel write port, and a new feature would change
// the rows still to come. A feature of vector b >= 1 waits until vector b - 1 is fully
// triggered (one feature buffer). Partial sums go to the ADDIN queue outside the macro.
bool Dimc_HWPE::beat_writable(const Dimc_Macro &mc, uint32_t job, const BeatPos &pos) const
{
    if (pos.kind == DIMC_LOAD_PSIN) {
        // A later job's section would open the macro's write program for that job and drop
        // the one not yet handed to compute (its kernel may still be streaming in).
        if (mc.write_job != Dimc_Macro::JOB_NONE && mc.write_job != job && !mc.stamped) return false;
        // The two ADDIN sets alternate between runs. A run's first section opens its set,
        // which the run two before it (or the previous job's run of that parity) read last:
        // every row of that run has to be triggered first.
        if (pos.sub != 0) return true;
        // Nothing triggered yet: runs 0 and 1 open unused sets; run r >= 2 reuses run r - 2's.
        if (mc.issue_job == Dimc_Macro::JOB_NONE) return pos.run < 2u;
        if (mc.issue_job == job) return pos.run < 2u || mc.runs_issued + 1u >= pos.run;
        if (mc.issue_job + 1u == job)
            return pos.run < 2u && mc.runs_issued + 1u >= mc.job_nb_vec + pos.run;
        return false;
    }
    if (mc.filled_job != job && this->holds_unissued(mc)) return false;
    // The kernel write port is shut in a trigger cycle (COMPE). Only reachable with the
    // pops after the triggers: holds_unissued turns false in the last row's own cycle.
    if (pos.kind == DIMC_LOAD_KB && mc.last_trigger_cycle == (int64_t)this->fsm_timestamp)
        return false;
    // A kernel replaces the one every earlier job of the macro computes with, so the macro
    // must have finished triggering every job before it. Its programs are for consecutive
    // jobs up to fill_job; `owed` of them are not finished. A kernel fetched ahead of its
    // program is for the job after fill_job.
    if (pos.kind == DIMC_LOAD_KB && mc.fill_job != Dimc_Macro::JOB_NONE) {
        if (job - mc.fill_job == 1u) {
            if (mc.owed != 0u) return false;
        } else if (mc.fill_job - job < 0x80000000u) {
            if (mc.owed > mc.fill_job - job + 1u) return false;
        } else {
            return false;
        }
    }
    // The job being written is not handed to the compute side yet (its kernel may be
    // complete while vector 0's feature still waits for its partial sums): another job's
    // sections would overwrite it.
    if (mc.write_job != Dimc_Macro::JOB_NONE && mc.write_job != job && !mc.stamped) return false;
    if (pos.kind == DIMC_LOAD_FB && pos.run >= 1
        && !(mc.issue_job == job && mc.runs_issued >= pos.run)) return false;
    return true;
}

void Dimc_HWPE::fill_feed(Dimc_InnerBlock &blk, uint32_t blk_id,
                          Dimc_InnerBlock::Cursor &cursor, bool kernel_feed)
{
    std::queue<uint64_t> &feed_pending = kernel_feed ? blk.kb_pending : blk.in_pending;
    if (kernel_feed) {
        // Program kernel sections already fetched ahead are consumed without a fetch.
        for (uint32_t m = 0; m < blk.macros.size() && m < cursor.macro_beat_index.size(); m++) {
            Dimc_Macro &t = blk.macros[m];
            if (t.kpf_job != t.fill_job) continue;
            const JobGeom &fsel = this->job_geom[t.fill_slot];
            while (cursor.macro_beat_index[m] < cursor.macro_beat_total[m]) {
                const BeatPos p = this->beat_pos(fsel, cursor.macro_beat_index[m]);
                if (p.kind != DIMC_LOAD_KB || p.sub >= t.kfetched) break;
                cursor.macro_beat_index[m]++;
                cursor.beat_index++;
            }
        }
    }
    if (cursor.beat_index >= cursor.beat_total && !kernel_feed) return;
    if (kernel_feed && blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) {
        this->tracer.kernel_skip(blk_id, DIMC_WHY_WAIT_DEPTH);
        this->tracer.fill_skip(blk_id, DIMC_WHY_WAIT_DEPTH);
        return;
    }
    // Pick the macro this section is fetched for, among those whose next section is on this
    // feed. Kernel: a macro that can take it now, else the one with the fewest runs left.
    // Input: into the macro's own half of the input FIFO when it has room; partial sums only
    // when no macro wants a feature, and once their producer is written back. The half is in
    // order, so run r + 2's partial sums (run r's set) land only after run r is triggered.
    const uint32_t nb_fill = (uint32_t)blk.macros.size();
    const size_t   half    = DIMC_INP_FIFO_DEPTH;
    // Shared FIFO: among the macros that may fetch, the one whose section is needed first
    // (fewest rows before its vector may enter, plus kernel sections its job still writes).
    auto urgency = [&](const Dimc_Macro &t, const BeatPos &pos) -> uint64_t {
        uint64_t u = this->rows_before_feature(t, t.fill_job, pos.run);
        const JobGeom &fg = this->job_geom[t.fill_slot];
        const uint32_t kb_all = fg.row_count * fg.kb_beats_per_row;
        const uint32_t kw = t.write_job == t.fill_job ? t.kw : 0;
        if (kw < kb_all) u += kb_all - kw;
        return u;
    };
    uint64_t best_u = ~(uint64_t)0;
    uint32_t macro = nb_fill;
    bool held_by_compute = false, held_by_psum = false;
    for (int pass = 0; pass < 2 && macro == nb_fill; pass++) {
        for (uint32_t m = 0; m < nb_fill; m++) {
            if (m >= cursor.macro_beat_index.size()) break;
            const uint32_t at = cursor.macro_beat_index[m];
            if (at >= cursor.macro_beat_total[m]) continue;
            const Dimc_Macro &t = blk.macros[m];
            const JobGeom &fsel = this->job_geom[t.fill_slot];
            const BeatPos pos = this->beat_pos(fsel, at);
            const uint8_t k = pos.kind;
            if ((k == DIMC_LOAD_KB) != kernel_feed) continue;
            if (kernel_feed) {
                const uint32_t ip = this->kernel_in_progress(blk);
                if (ip != nb_fill && ip != m) continue;
                if (!this->beat_writable(t, t.fill_job, pos)) { held_by_compute = true; continue; }
                macro = m;
                break;
            }
            if (blk.inp_queue(m).size() >= half) continue;
            const bool psum = k == DIMC_LOAD_PSIN;
            if (pass == 0 && psum) continue;
            if (psum && !this->psum_ready(blk, m, fsel, t.fill_job, pos.run)) { held_by_psum = true; continue; }
            // A run's first partial-sum section clears its set's row flags (fill_beat). With the
            // shared FIFO's full depth it can be fetched while an earlier run still triggers rows
            // of that set: it waits until the set is free.
            if (psum && pos.sub == 0 && !this->beat_writable(t, t.fill_job, pos)) {
                held_by_compute = true;
                continue;
            }
            // The shared FIFO pops in order: a feature section is fetched once the rows it waits
            // for (the previous run, or the macro's current job) are at most
            // DIMC_INP_FETCH_LEAD from triggered, so a head its macro cannot take yet holds the
            // other macro's sections for at most that long.
            if (k == DIMC_LOAD_FB && !this->beat_writable(t, t.fill_job, pos)
                && this->rows_before_feature(t, t.fill_job, pos.run) > DIMC_INP_FETCH_LEAD) {
                held_by_compute = true;
                continue;
            }
            const uint64_t u = urgency(t, pos);
            if (u < best_u) { best_u = u; macro = m; }
        }
    }
    if (macro == nb_fill && kernel_feed) {
        uint32_t best_left = 0xFFFFFFFFu;
        for (uint32_t m = 0; m < nb_fill; m++) {
            if (m >= cursor.macro_beat_index.size()) break;
            const uint32_t at = cursor.macro_beat_index[m];
            if (at >= cursor.macro_beat_total[m]) continue;
            const Dimc_Macro &t = blk.macros[m];
            if (this->beat_pos(this->job_geom[t.fill_slot], at).kind != DIMC_LOAD_KB) continue;
            const uint32_t ip = this->kernel_in_progress(blk);
            if (ip != nb_fill && ip != m) continue;
            const uint32_t left = t.job_nb_vec > t.runs_issued ? t.job_nb_vec - t.runs_issued : 0;
            if (left < best_left) { best_left = left; macro = m; }
        }
    }
    if (macro == nb_fill) {
        if (held_by_psum) {
            this->psum_waits++;
            this->tracer.fill_skip(blk_id, DIMC_WHY_WAIT_PSUM);
        } else if (held_by_compute) {
            this->tracer.fill_skip(blk_id, DIMC_WHY_WAIT_COMPUTE);
        }
        return;
    }
    // A request: within the outstanding depth, and the port free this cycle. The kept
    // second section of a port word needs neither.
    const Dimc_Macro &sel = blk.macros[macro];
    const uint8_t sel_kind = this->beat_pos(this->job_geom[sel.fill_slot], cursor.macro_beat_index[macro]).kind;
    const Dimc_HWPE_Streamer &st = sel_kind == DIMC_LOAD_KB ? blk.weight_stream[macro]
                                 : sel_kind == DIMC_LOAD_FB ? blk.input_stream[macro] : blk.psin_stream[macro];
    if (!st.pair_ready()) {
        if (feed_pending.size() >= this->outstanding_depth) {
            if (kernel_feed) this->tracer.kernel_skip(blk_id, DIMC_WHY_WAIT_DEPTH);
            else             this->tracer.input_skip(blk_id, DIMC_WHY_WAIT_DEPTH);
            this->tracer.fill_skip(blk_id, DIMC_WHY_WAIT_DEPTH);
            return;
        }
        if (this->outer_port_in.busy_until() > (int64_t)this->fsm_timestamp) {
            if (kernel_feed) this->tracer.kernel_skip(blk_id, DIMC_WHY_WAIT_OUTER_PORT);
            else             this->tracer.input_skip(blk_id, DIMC_WHY_WAIT_OUTER_PORT);
            this->tracer.fill_skip(blk_id, DIMC_WHY_WAIT_OUTER_PORT);
            return;
        }
    }
    this->fill_beat(blk, blk_id, cursor, macro, feed_pending);
}

// The macro whose kernel is partly in the weight FIFO (started, not finished), or nb.
// Its sections must all go in before any other macro's, or the FIFO holds two kernels
// interleaved and the first macro waits behind the second's compute.
uint32_t Dimc_HWPE::kernel_in_progress(const Dimc_InnerBlock &blk) const
{
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        const Dimc_Macro &t = blk.macros[m];
        if (t.kpf_job == Dimc_Macro::JOB_NONE || t.kfetched == 0) continue;
        const JobGeom &g = this->job_geom[t.kpf_slot];
        if (this->geom_job[t.kpf_slot] != t.kpf_job) continue;
        if (t.kfetched < g.row_count * g.kb_beats_per_row) return m;
    }
    return (uint32_t)blk.macros.size();
}

// True when every macro of the dual has fetched the whole kernel of every job from the
// running one up to (not including) `job` that needs a kernel on it.
bool Dimc_HWPE::kernels_fetched_before(Dimc_InnerBlock &blk, uint32_t job)
{
    auto check = [&](uint32_t j, int slot) -> bool {
        if (j >= job) return true;
        if (this->geom_job[slot] != j) { this->latch_geom(slot); this->size_fill(slot); }
        const JobGeom &g = this->job_geom[slot];
        const uint32_t kb_all = g.row_count * g.kb_beats_per_row;
        for (uint32_t m = 0; m < blk.macros.size() && m < g.num_active; m++) {
            const Dimc_Macro &t = blk.macros[m];
            if (t.kpf_job != Dimc_Macro::JOB_NONE && t.kpf_job > j) continue;
            if (t.kpf_job == j && t.kfetched >= kb_all) continue;
            // The running job's kernel may already sit in the macro from before this bookkeeping.
            if (j == this->running_job && (t.filled_job == j || t.write_job == j) && t.kw >= kb_all) continue;
            return false;
        }
        return true;
    };
    if (this->running_ctx >= 0 && !check(this->running_job, (int)this->exec_slot)) return false;
    for (int c : this->ctx_queue) {
        const uint32_t j = this->ctx_job_id[c];
        if (j >= job) break;
        if (!check(j, c)) return false;
    }
    return true;
}

// First program index after a macro's kernel sections for a job of geometry g.
uint32_t Dimc_HWPE::kb_end(const JobGeom &g) const
{
    const uint32_t kb = g.row_count * g.kb_beats_per_row;
    return g.psin_beats_per_macro + g.fb_beats_per_macro + kb;
}

// Fetch one kernel section of a macro's next job (or of its current job whose program has
// not reached the kernel) into the weight FIFO, ahead of the program. Only while
// no other macro of the dual owes kernel sections its program has reached; one macro at a
// time: the one in progress, else the one with the fewest runs left.
bool Dimc_HWPE::kernel_prefetch(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    Dimc_InnerBlock::Cursor &cur = blk.fill;
    const uint32_t nb = (uint32_t)blk.macros.size();
    const uint32_t ip = this->kernel_in_progress(blk);
    uint32_t own = 0;   // macros whose kernel of their current program is fetched ahead
    for (uint32_t m = 0; m < nb && m < cur.macro_beat_index.size(); m++) {
        if (m == ip) continue;
        const Dimc_Macro &t = blk.macros[m];
        if (t.fill_job == Dimc_Macro::JOB_NONE) continue;
        if (cur.macro_beat_index[m] >= cur.macro_beat_total[m]) continue;
        const JobGeom &g = this->job_geom[t.fill_slot];
        const uint32_t kb_all = g.row_count * g.kb_beats_per_row;
        if (cur.macro_beat_index[m] < this->kb_end(g)
            && !(t.kpf_job == t.fill_job && t.kfetched >= kb_all)) {
            // Program still before its kernel sections: fetched ahead below.
            if (cur.macro_beat_index[m] < this->kb_end(g) - kb_all) {
                own |= 1u << m;
                continue;
            }
            return false;
        }
    }
    // The job after each macro's current fill job.
    auto next_of = [&](uint32_t job, int *slot) -> bool {
        if (job == this->running_job) {
            if (this->ctx_queue.empty() || (uint32_t)this->ctx_queue.front() == this->exec_slot) return false;
            *slot = this->ctx_queue.front(); return true;
        }
        for (size_t i = 0; i + 1 < this->ctx_queue.size(); i++)
            if (this->ctx_job_id[this->ctx_queue[i]] == job) { *slot = this->ctx_queue[i + 1]; return true; }
        return false;
    };
    uint32_t pick = nb, best_left = 0xFFFFFFFFu;
    int pick_slot = -1;
    if (ip != nb) {
        // Finish the kernel already under way, whether or not its program is planned.
        pick = ip; pick_slot = (int)blk.macros[ip].kpf_slot;
    }
    for (uint32_t m = 0; m < nb && pick == nb; m++) {
        const Dimc_Macro &t = blk.macros[m];
        if (t.fill_job == Dimc_Macro::JOB_NONE) continue;
        int slot;
        if (own >> m & 1u)                    slot = (int)t.fill_slot;
        else if (!next_of(t.fill_job, &slot)) continue;
        uint32_t job = this->ctx_job_id[slot];
        if (this->geom_job[slot] != job) { this->latch_geom(slot); this->size_fill(slot); }
        const JobGeom &gn = this->job_geom[slot];
        if (m >= gn.num_active) continue;
        const uint32_t kb_all = gn.row_count * gn.kb_beats_per_row;
        const uint32_t done = t.kpf_job == job ? t.kfetched : 0;
        if (done >= kb_all) continue;
        // Every macro of the dual, this one included, must have fetched its kernel of every
        // earlier job still to run, or this kernel would enter the FIFO ahead of one needed first.
        if (done == 0 && !this->kernels_fetched_before(blk, job)) continue;
        if (done > 0) { pick = m; pick_slot = slot; break; }       // the one in progress
        const uint32_t left = t.job_nb_vec > t.runs_issued ? t.job_nb_vec - t.runs_issued : 0;
        if (left < best_left) { best_left = left; pick = m; pick_slot = slot; }
    }
    if (pick == nb) return false;
    // Only one macro's kernel ahead in the FIFO at a time.
    for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
        if (q.macro != pick) return false;
    Dimc_Macro &fm = blk.macros[pick];
    const uint32_t job = this->ctx_job_id[pick_slot];
    const JobGeom &fg = this->job_geom[pick_slot];
    if (fm.kpf_job != job) {
        fm.kpf_job = job; fm.kpf_slot = (uint32_t)pick_slot; fm.kfetched = 0;
        this->configure_macro_streams(blk_id, pick, pick_slot, 0, true, false, false);
    }
    if (!blk.weight_stream[pick].pair_ready()) {
        if (blk.kb_pending.size() >= this->outstanding_depth) return false;
        if (this->outer_port_in.busy_until() > (int64_t)this->fsm_timestamp) {
            this->tracer.kernel_skip(blk_id, DIMC_WHY_WAIT_OUTER_PORT);
            return false;
        }
    }
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t sub = fm.kfetched;
    Dimc_InnerBlock::FeedEntry e;
    e.macro = pick; e.slot = (uint32_t)pick_slot; e.job = job;
    e.within = fg.psin_beats_per_macro + fg.fb_beats_per_macro + sub;
    const uint32_t off = (sub % fg.kb_beats_per_row) * port_bytes;
    e.bytes = DIMC_MACRO_KB_EW - off < port_bytes ? DIMC_MACRO_KB_EW - off : port_bytes;
    int lat = blk.weight_stream[pick].issue_beat((int)e.bytes, e.data);
    const bool requested = lat != Dimc_HWPE_Streamer::NO_REQUEST;
    if (lat < 1) lat = requested ? 1 : 0;
    e.ready = this->fsm_timestamp + (uint64_t)lat + 1;
    blk.wgt_fifo.push_back(e);
    fm.kfetched++;
    if (requested) {
        this->outer_port_in.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
        this->tracer.outer_port_booked(blk_id, DIMC_LOAD_KB);
        blk.kb_pending.push(this->fsm_timestamp + (uint64_t)lat);
        blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
    }
    return true;
}

// One cycle while the engine is idle and the queue is held. Each dual fetches the first held
// job's kernel of its first macro into its weight FIFO; the job's fill program then skips what
// is fetched (kfetched).
void Dimc_HWPE::held_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    if (_this->job_running || _this->ctx_queue.empty()) return;
    const int ctx = _this->ctx_queue.front();
    const uint32_t job = _this->ctx_job_id[ctx];
    if (_this->geom_job[ctx] != job) { _this->latch_geom(ctx); _this->size_fill(ctx); }
    bool more = false, did = false;
    for (uint32_t b = 0; b < _this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = _this->inner_blocks[b];
        const size_t q0 = blk.wgt_fifo.size();
        uint32_t f0 = 0;
        for (const Dimc_Macro &m : blk.macros) f0 += m.kfetched;
        if (_this->held_kernel_step(blk, b, ctx)) more = true;
        const size_t q1 = blk.wgt_fifo.size();
        uint32_t f1 = 0;
        for (const Dimc_Macro &m : blk.macros) f1 += m.kfetched;
        if (f1 != f0 || blk.wgt_fifo.size() != q1 || q1 != q0) did = true;
        // The FIFO keeps the kernel: run until its last section has landed.
        for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
            if (q.ready > _this->fsm_timestamp) more = true;
    }
    for (uint32_t b = 0; b < _this->inner_blocks.size(); b++) _this->tracer.end_cycle(b);
    _this->tracer.port_cycle();
    // A call with nothing to do (the one after the last write) only records the idle
    // cycle, so the trace does not keep the last load until the job starts.
    if (!more && !did) return;
    _this->fsm_timestamp++;
    for (Dimc_InnerBlock &blk : _this->inner_blocks) _this->retire_block(blk);
    _this->event_enqueue(_this->held_event, 1);
}

// Fetch one kernel section of the held job for the lowest macro whose kernel is not all
// fetched (the first macro only), while the FIFO holds no other macro's sections. False when there is nothing left to fetch for this dual.
bool Dimc_HWPE::held_kernel_step(Dimc_InnerBlock &blk, uint32_t blk_id, int ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    const uint32_t job = this->ctx_job_id[ctx];
    const uint32_t kb_all = g.row_count * g.kb_beats_per_row;
    uint32_t pick = (uint32_t)blk.macros.size();
    for (uint32_t m = 0; m < blk.macros.size() && m < 1u; m++) {
        const Dimc_Macro &t = blk.macros[m];
        if (t.kpf_job == job && t.kfetched >= kb_all) continue;
        pick = m;
        break;
    }
    if (pick == blk.macros.size()) return false;
    for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
        if (q.macro != pick) return true;          // the previous macro's kernel drains first
    if (blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) return true;
    Dimc_Macro &fm = blk.macros[pick];
    if (fm.kpf_job != job) {
        fm.kpf_job = job; fm.kpf_slot = (uint32_t)ctx; fm.kfetched = 0;
        this->configure_macro_streams(blk_id, pick, ctx, 0, true, false, false);
    }
    if (!blk.weight_stream[pick].pair_ready()) {
        if (blk.kb_pending.size() >= this->outstanding_depth) return true;
        if (this->outer_port_in.busy_until() > (int64_t)this->fsm_timestamp) return true;
    }
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t sub = fm.kfetched;
    Dimc_InnerBlock::FeedEntry e;
    e.macro = pick; e.slot = (uint32_t)ctx; e.job = job;
    e.within = g.psin_beats_per_macro + g.fb_beats_per_macro + sub;
    const uint32_t off = (sub % g.kb_beats_per_row) * port_bytes;
    e.bytes = DIMC_MACRO_KB_EW - off < port_bytes ? DIMC_MACRO_KB_EW - off : port_bytes;
    int lat = blk.weight_stream[pick].issue_beat((int)e.bytes, e.data);
    const bool requested = lat != Dimc_HWPE_Streamer::NO_REQUEST;
    if (lat < 1) lat = requested ? 1 : 0;
    e.ready = this->fsm_timestamp + (uint64_t)lat + 1;
    blk.wgt_fifo.push_back(e);
    fm.kfetched++;
    if (requested) {
        this->outer_port_in.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
        this->tracer.outer_port_booked(blk_id, DIMC_LOAD_KB);
        blk.kb_pending.push(this->fsm_timestamp + (uint64_t)lat);
        blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
    }
    return true;
}

// Fetch one section of macro `macro`'s program from L1 into the dual's FIFO.
void Dimc_HWPE::fill_beat(Dimc_InnerBlock &blk, uint32_t blk_id,
                          Dimc_InnerBlock::Cursor &cursor, uint32_t macro,
                          std::queue<uint64_t> &feed_pending)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t within = cursor.macro_beat_index[macro];
    Dimc_Macro &fm = blk.macros[macro];
    const JobGeom &fg = this->job_geom[fm.fill_slot];
    const BeatPos  pos = this->beat_pos(fg, within);
    const uint32_t ps_span = fg.psin_beats_per_macro;
    const uint32_t pset    = (fm.fill_job * fg.nb_vec + pos.run) & 1;
    const uint8_t  kind    = pos.kind;

    // First section of a run: mark its partial sums outstanding, in the run's set, which
    // no row still to trigger reads.
    const bool run_start = pos.sub == 0
        && kind == (ps_span ? DIMC_LOAD_PSIN : DIMC_LOAD_FB);
    if (run_start)
        for (uint64_t &r : fm.psin_row_ready[pset])
            r = fg.psin_rows ? ~(uint64_t)0 : 0;
    // Vectors after the first read their own feature and partial-sum blocks.
    if (pos.run >= 1 && pos.sub == 0 && kind != DIMC_LOAD_KB)
        this->configure_macro_streams(blk_id, macro, (int)fm.fill_slot, pos.run,
                                      false, kind == DIMC_LOAD_FB, kind == DIMC_LOAD_PSIN);

    Dimc_InnerBlock::FeedEntry e;
    e.macro = macro; e.slot = fm.fill_slot; e.job = fm.fill_job; e.within = within;
    int lat;
    if (kind == DIMC_LOAD_KB) {
        const uint32_t off = (pos.sub % fg.kb_beats_per_row) * port_bytes;
        e.bytes = DIMC_MACRO_KB_EW - off < port_bytes ? DIMC_MACRO_KB_EW - off : port_bytes;
        lat = blk.weight_stream[macro].issue_beat((int)e.bytes, e.data);
        if (fm.kpf_job != fm.fill_job) { fm.kpf_job = fm.fill_job; fm.kpf_slot = fm.fill_slot; fm.kfetched = 0; }
        fm.kfetched++;
    } else if (kind == DIMC_LOAD_FB) {
        const uint32_t off = pos.sub * port_bytes;
        e.bytes = DIMC_MACRO_FB_EW - off < port_bytes ? DIMC_MACRO_FB_EW - off : port_bytes;
        lat = blk.input_stream[macro].issue_beat((int)e.bytes, e.data);
    } else {
        const uint32_t per   = port_bytes / DIMC_OUT_SLOT_BYTES;
        const uint32_t first = pos.sub * per;
        const uint32_t n     = fg.row_count - first < per ? fg.row_count - first : per;
        e.bytes = n * DIMC_OUT_SLOT_BYTES;
        lat = blk.psin_stream[macro].issue_beat((int)e.bytes, e.data);
    }
    const bool requested = lat != Dimc_HWPE_Streamer::NO_REQUEST;
    if (lat < 1) lat = requested ? 1 : 0;
    e.ready = this->fsm_timestamp + (uint64_t)lat + 1;   // fifo_v3, not fall-through
    (kind == DIMC_LOAD_KB ? blk.wgt_fifo : blk.inp_queue(macro)).push_back(e);

    cursor.macro_beat_index[macro]++;
    cursor.beat_index++;
    if (!requested) return;
    this->outer_port_in.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
    this->tracer.outer_port_booked(blk_id, kind);
    feed_pending.push(this->fsm_timestamp + (uint64_t)lat);
    blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
}

// Pop the head of one feed's FIFO into its macro, once landed and once the macro's
// write port takes it (head-of-line: a section the macro cannot take yet holds the
// sections behind it).
void Dimc_HWPE::write_feed(Dimc_InnerBlock &blk, uint32_t blk_id,
                           std::deque<Dimc_InnerBlock::FeedEntry> &fifo)
{
    if (fifo.empty()) return;
    const Dimc_InnerBlock::FeedEntry &e = fifo.front();
    if (e.ready > this->fsm_timestamp) return;
    Dimc_Macro &mc = blk.macros[e.macro];
    const JobGeom &g = this->job_geom[e.slot];
    const BeatPos pos = this->beat_pos(g, e.within);
    if (!this->beat_writable(mc, e.job, pos)) {
        this->tracer.fill_skip(blk_id, DIMC_WHY_WAIT_COMPUTE);
        return;
    }
    // Kernel and feature sections: one per macro per cycle.
    if (pos.kind != DIMC_LOAD_PSIN) {
        if (mc.last_write_cycle == (int64_t)this->fsm_timestamp) return;
        mc.last_write_cycle = (int64_t)this->fsm_timestamp;
    }
    // The macro's write program follows its sections: a new job's first one opens it.
    if (mc.write_job != e.job) {
        mc.write_job  = e.job;
        mc.write_slot = e.slot;
        mc.written    = 0;
        mc.kw = 0; mc.f0 = false; mc.stamped = false;
    }
    if (pos.kind == DIMC_LOAD_KB) mc.kw++;
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t pset       = (e.job * g.nb_vec + pos.run) & 1;
    bool feature_done = false;

    if (pos.kind == DIMC_LOAD_KB) {
        const uint32_t row = pos.sub / g.kb_beats_per_row;
        const uint32_t sub = pos.sub % g.kb_beats_per_row;
        std::memcpy(mc.kb_row_buffer + sub * port_bytes, e.data, e.bytes);
        if (sub == g.kb_beats_per_row - 1)   // row complete -> commit
            mc.write_row((int)((g.row_base + row) % DIMC_MACRO_KB_LEN), mc.kb_row_buffer);
    } else if (pos.kind == DIMC_LOAD_FB) {
        std::memcpy(mc.row_buffer + pos.sub * port_bytes, e.data, e.bytes);
        if (pos.sub == g.fb_beats_per_macro - 1) {   // feature complete
            mc.write_fb(mc.row_buffer);
            mc.kb_ready = true;
            mc.fb_ready = true;
            // The pipe is NOT cleared here: when the fill runs ahead, it still holds
            // the previous job's last rows, triggered before this feature landed.
            mc.psin_scalar = (int32_t)this->job_reg_ctx((int)e.slot, DIMC_HWPE_PSIN);
            mc.fb_run = pos.run;
            mc.fb_ready_cycle = this->fsm_timestamp + 1;
            feature_done = true;
            if (pos.run == 0) mc.f0 = true;
        }
    } else {
        // Rows [first, first + n) in one port word; each row's value is the low 4 bytes
        // of its SLOT-byte slot.
        const uint32_t per   = port_bytes / DIMC_OUT_SLOT_BYTES;
        const uint32_t first = pos.sub * per;
        const uint32_t n     = e.bytes / DIMC_OUT_SLOT_BYTES;
        // The set these partial sums go into was last read by the run two before this one,
        // or by the previous job's run of that parity; its rows must all be triggered.
        // beat_writable already holds such a section: this is a consistency check.
        if (pos.sub == 0) {
            bool free_set;
            if (mc.issue_job == Dimc_Macro::JOB_NONE) free_set = pos.run < 2u;  /* never triggered */
            else if (mc.issue_job == e.job)           free_set = pos.run < 2u || mc.runs_issued + 1u >= pos.run;
            else if (mc.issue_job + 1u == e.job)      free_set = pos.run < 2u
                                                          && (mc.runs_issued + 1u >= mc.job_nb_vec + pos.run);
            else                                       free_set = false;   /* two or more jobs ahead */
            if (!free_set)
                this->trace.force_warning("DIMC psin set overwrite: macro %u job %u run %u set %u while the "
                    "set's previous run still has rows to trigger\n", (unsigned)e.macro, e.job, pos.run, pset);
        }
        for (uint32_t j = 0; j < n; j++) {
            const uint32_t row_idx = (g.row_base + first + j) % DIMC_MACRO_KB_LEN;
            mc.write_psin_row((int)row_idx, e.data + j * DIMC_OUT_SLOT_BYTES, (int)pset);
            mc.psin_row_ready[pset][row_idx] = this->fsm_timestamp + 1;
        }
    }
    const uint32_t within = e.within, macro = e.macro;
    fifo.pop_front();
    mc.written++;
    const bool macro_filled = mc.written >= g.beats_per_macro;
    // Vector 0's feature and the whole kernel (unless reused) are in. The partial sums
    // are gated per row (psin_row_ready), so they need not be counted here.
    const uint32_t kb_all = g.row_count * g.kb_beats_per_row;
    const bool ready = !mc.stamped && mc.f0 && mc.kw >= kb_all;
    if (ready) {
        mc.stamped = true;
        // Vector 0's feature and the kernel rows are in: hand the macro to the compute
        // side, with the shape of the job it now holds.
        mc.fill_done_cycle = this->fsm_timestamp + 1;
        mc.exec_ready   = true;
        mc.filled_job   = mc.write_job;
        mc.filled_slot  = mc.write_slot;
        mc.job_nb_vec   = g.nb_vec;
        mc.job_rows     = g.row_count;
        mc.job_row_base = g.row_base;
        mc.psin_rows    = (uint8_t)g.psin_rows;
        mc.ci           = this->ctx_ci[mc.write_slot];
        mc.sign_8b      = this->ctx_sign_8b[mc.write_slot];
        mc.compute_mask = this->ctx_compute_mask[mc.write_slot];
    }
    this->tracer.fill_beat(blk_id, macro, within, macro_filled, feature_done, 1);
}

// Rows are issued and drained inside the fill phase, one macro at a time as
// each finishes its own beats -- that overlap is the point -- so there is no
// compute phase of its own.

// Each macro issues its own rows as soon as its own fill has landed, so a
// macro still pulling beats through the shared inner port does not hold back a
// sibling that is ready to compute. That overlap is the inner double buffer.
void Dimc_HWPE::compute_indep(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    this->drain_ready_rows(blk);

    // Each macro triggers whenever its own vector is ready; results go to its own out_fifo,
    // so two macros may trigger in one cycle. A macro holding the next job's operands issues
    // its rows too, only into free out_fifo entries.
    const uint32_t J = this->running_job;
    const size_t fifo_cap = DIMC_OUT_FIFO_DEPTH / blk.macros.size();
    for (uint32_t m = 0; m < (uint32_t)blk.macros.size(); m++) {
        Dimc_Macro &mac = blk.macros[m];
        const bool ahead = mac.filled_job == J + 1;
        if (mac.exec_ready && (mac.filled_job == J || ahead)) {
            const uint32_t jj = mac.filled_job;
            if (mac.issue_job != jj) {
                const uint32_t s = jj & 1;
                mac.issue_job = jj;
                mac.issue_slot = mac.filled_slot;
                mac.rows_issued = 0;
                mac.runs_issued = 0;
                mac.set_job[s] = jj;
                mac.rows_retired_set[s] = 0;
            }
            const uint32_t b = mac.runs_issued;   // the vector it issues next
            const bool room = !ahead || blk.out_fifo[m].size() + mac.pipe.size() < fifo_cap;
            if (b < mac.job_nb_vec && mac.fb_run == b && room
                && this->fsm_timestamp >= mac.fb_ready_cycle
                && this->fsm_timestamp >= mac.fill_done_cycle && mac.can_accept()) {
                if (mac.rows_issued >= mac.job_rows) mac.rows_issued = 0;   // next run
                const uint32_t row_idx = (mac.job_row_base + mac.rows_issued) % DIMC_MACRO_KB_LEN;
                const uint32_t pset = (jj * mac.job_nb_vec + b) & 1;
                if (!mac.psin_rows || this->fsm_timestamp >= mac.psin_row_ready[pset][row_idx]) {
                    mac.psin_sel = (uint8_t)pset;
                    mac.issue((int)row_idx, (int)mac.rows_issued, (int)(jj & 1), (int)b);
                    mac.last_trigger_cycle = (int64_t)this->fsm_timestamp;
                    mac.rows_issued++;
                    this->tracer.row_issued(blk_id, m, mac.rows_issued, mac.job_rows);
                    if (mac.rows_issued >= mac.job_rows) {
                        mac.runs_issued++;
                        if (mac.runs_issued >= mac.job_nb_vec && mac.owed) mac.owed--;
                    }
                }
            }
        }
        mac.tick();
    }
}

void Dimc_HWPE::drain_ready_rows(Dimc_InnerBlock &blk)
{
    // A macro is never stalled for its out_fifo: a result the FIFO cannot take is lost, as
    // in the RTL. The model also warns, and a force_warning fails the run.
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        if (!blk.macros[m].has_ready()) continue;
        DimcPipeEntry e = blk.macros[m].drain();
        blk.macros[m].rows_retired_set[e.set]++;
        if (blk.out_fifo[m].size() >= DIMC_OUT_FIFO_DEPTH / blk.macros.size()) {
            this->out_dropped++;
            if (this->out_dropped <= 8)
                this->trace.force_warning("DIMC result lost: macro %u row %d (out_fifo full)\n",
                                          m, e.job_row);
            continue;
        }
        blk.out_fifo[m].push_back({e.psout, (uint16_t)e.job_row, (uint16_t)e.run});
        // The same pop clocks the output accumulator.
        blk.out_accum.push(e.psout);
    }
}

// ================= STORING =================
bool Dimc_HWPE::store_iter(int *latency)
{
    *latency = 1;

    if (!this->phase_planned) {
        this->phase_planned = true;
    }

    for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
        const uint32_t b = this->block_order(i);
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        if (i == 0) this->fetch_kernels_first();
        // The macro pipeline does not stop because the job moved on to
        // storing: rows issued near the end of the fill are still in flight and
        // have to retire before their beats can go out. Every row of the running
        // job is already issued; compute_indep ticks and drains, and may issue a
        // later job's rows.
        this->block_cycle(blk, b);                // the next job keeps filling
        if (!blk.phase_done) this->store_block(blk, b);
    }
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) this->tracer.end_cycle(b);
    this->tracer.port_cycle();

    this->fsm_timestamp++;
    this->advance_fill();

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        this->retire_block(blk);
        size_t after = retire_due(blk.store_pending, this->fsm_timestamp);
        bool beats_done = (blk.store.beat_index >= blk.store.beat_total) && (after == 0);
        if (beats_done) blk.phase_done = true;
        else            all_done = false;
    }

    if (!all_done) return false;

    // Every result is out. The elapsed cycles ARE the makespan, for the whole
    // outer block: every beat was charged as it issued, against the latency the
    // L1 bank and the crossbar returned. fsm() closes the job.
    *latency = 1;
    return true;
}

bool Dimc_HWPE::store_done() const
{
    for (const Dimc_InnerBlock &blk : this->inner_blocks)
        if (blk.store.beat_index < blk.store.beat_total || !blk.store_pending.empty())
            return false;
    return true;
}

// Each phase advanced the clock one cycle per beat, so job start to here already
// took the job's whole duration. The completion event fires one cycle later, not
// after that duration again, which would count it twice.
void Dimc_HWPE::close_job()
{
    this->phase_end_reset();
    this->phase_planned = false;
    this->tracer.job_closed((uint64_t)this->clock.get_cycles());
}

void Dimc_HWPE::store_block(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const JobGeom &g          = this->job_geom[this->exec_slot];
    const uint32_t rows       = g.row_count;
    const uint32_t per_word   = port_bytes / DIMC_OUT_SLOT_BYTES;

    // One port word per dual per cycle, straight from a macro's out_fifo: its head entries are
    // the running job's next results in order (row k % rows of vector k / rows). The lowest-index
    // macro holding a whole word goes; an unexpected head fails the run (the RTL would misplace it).
    for (uint32_t m = 0; m < g.num_active; m++) {
        std::deque<Dimc_InnerBlock::OutEntry> &fifo = blk.out_fifo[m];
        const uint32_t k = blk.out_results[m];
        // Every result of this job from this macro has left; what is behind it in the
        // out_fifo belongs to the next job and waits for its sink.
        if (k >= rows * g.nb_vec) continue;
        const uint32_t r = k % rows, vec = k / rows;
        const uint32_t n = rows - r < per_word ? rows - r : per_word;
        if (fifo.size() < n) continue;
        bool in_order = true;
        for (uint32_t j = 0; j < n; j++)
            if (fifo[j].row != r + j || fifo[j].run != vec) in_order = false;
        if (!in_order) {
            this->trace.force_warning("DIMC out_fifo order: macro %u head is vector %u row %u, "
                "the sink expects vector %u row %u; the RTL would misplace it\n",
                m, fifo[0].run, fifo[0].row, vec, r);
            continue;
        }
        if (blk.store_pending.size() >= this->outstanding_depth) {
            this->tracer.store_skip(blk_id, DIMC_WHY_WAIT_DEPTH);
            return;
        }
        if (this->outer_port_out.busy_until() > (int64_t)this->fsm_timestamp) {
            this->tracer.store_skip(blk_id, DIMC_WHY_WAIT_OUTER_PORT);
            return;
        }
        // First word of a run: point the macro's output streamer at the run's block.
        if (r == 0) this->configure_out_stream(blk_id, m, vec);
        uint8_t word[32];
        for (uint32_t j = 0; j < n; j++) {
            uint8_t *slot = word + j * DIMC_OUT_SLOT_BYTES;
            std::memset(slot, fifo[j].psout < 0 ? 0xFF : 0x00, DIMC_OUT_SLOT_BYTES);
            std::memcpy(slot, &fifo[j].psout, 4);
        }
        const uint32_t w = n * DIMC_OUT_SLOT_BYTES;
        int lat = blk.out_stream[m].issue_beat((int)w, word);
        this->tracer.store_beat(blk_id, m, lat, g.out_beats);
        if (lat < 1) lat = 1;
        blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
        blk.store_pending.push(this->fsm_timestamp + (uint64_t)lat);
        this->outer_port_out.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
        this->tracer.outer_port_booked(blk_id, DIMC_PORT_WB);
        if (r + n == rows)
            blk.run_pending.push_back({this->fsm_timestamp + (uint64_t)lat, this->running_job, m, vec});
        fifo.erase(fifo.begin(), fifo.begin() + n);
        blk.out_results[m] += n;
        blk.store.beat_index++;
        return;
    }
    // Nothing of the running job went out this cycle.
    this->store_next_job(blk, blk_id);
}

// One port word of a macro's next-job results, once every result of
// the running job from that macro has left. The word goes to the next job's destination,
// from its own context; the order check and the port rules are store_block's.
void Dimc_HWPE::store_next_job(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t per_word   = port_bytes / DIMC_OUT_SLOT_BYTES;
    const JobGeom &gr         = this->job_geom[this->exec_slot];
    const uint32_t next       = this->running_job + 1;
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        const Dimc_Macro &mac = blk.macros[m];
        if (mac.issue_job != next) continue;
        if (m < gr.num_active && blk.out_results[m] < gr.row_count * gr.nb_vec) continue;
        std::deque<Dimc_InnerBlock::OutEntry> &fifo = blk.out_fifo[m];
        const JobGeom &g   = this->job_geom[mac.issue_slot];
        const uint32_t rows = g.row_count;
        const uint32_t k = blk.out_results_next[m];
        const uint32_t r = k % rows, vec = k / rows;
        const uint32_t n = rows - r < per_word ? rows - r : per_word;
        if (fifo.size() < n) continue;
        bool in_order = true;
        for (uint32_t j = 0; j < n; j++)
            if (fifo[j].row != r + j || fifo[j].run != vec) in_order = false;
        if (!in_order) {
            this->trace.force_warning("DIMC out_fifo order (next job): macro %u head is vector %u "
                "row %u, the sink expects vector %u row %u\n", m, fifo[0].run, fifo[0].row, vec, r);
            continue;
        }
        if (blk.store_pending.size() + blk.store_next_pending.size() >= this->outstanding_depth) return;
        if (this->outer_port_out.busy_until() > (int64_t)this->fsm_timestamp) return;
        if (r == 0) this->configure_out_stream_ctx(blk_id, m, vec, mac.issue_slot);
        uint8_t word[32];
        for (uint32_t j = 0; j < n; j++) {
            uint8_t *slot = word + j * DIMC_OUT_SLOT_BYTES;
            std::memset(slot, fifo[j].psout < 0 ? 0xFF : 0x00, DIMC_OUT_SLOT_BYTES);
            std::memcpy(slot, &fifo[j].psout, 4);
        }
        const uint32_t w = n * DIMC_OUT_SLOT_BYTES;
        int lat = blk.out_stream[m].issue_beat((int)w, word);
        this->tracer.store_beat(blk_id, m, lat,
                                g.nb_vec * ((rows * DIMC_OUT_SLOT_BYTES + port_bytes - 1) / port_bytes));
        if (lat < 1) lat = 1;
        blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
        blk.store_next_pending.push(this->fsm_timestamp + (uint64_t)lat);
        this->outer_port_out.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
        this->tracer.outer_port_booked(blk_id, DIMC_PORT_WB);
        if (r + n == rows)
            blk.run_pending.push_back({this->fsm_timestamp + (uint64_t)lat, next, m, vec});
        fifo.erase(fifo.begin(), fifo.begin() + n);
        blk.out_results_next[m] += n;
        blk.store_next_beats++;
        return;
    }
}
