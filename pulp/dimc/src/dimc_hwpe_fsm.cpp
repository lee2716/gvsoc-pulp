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

void Dimc_HWPE::fsm_start_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;

    // STATUS reads 0 (busy) from job start until the job ends.
    // ci, sign_8b, compute_mask and psin_rows travel with each job's operands (write_feed).
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x0;

    // fsm_timestamp runs free across jobs: the pending queues hold absolute due cycles.
    _this->job_start_cycle = _this->fsm_timestamp;
    _this->exec_slot = (uint32_t)(_this->running_ctx >= 0 ? _this->running_ctx : 0);

    // A macro already filled, filling or being written for this job keeps its buffers:
    // clearing kb_ready/fb_ready under a fill still running would strand it.
    for (Dimc_InnerBlock &blk : _this->inner_blocks)
        for (uint32_t m = 0; m < _this->num_macros; m++) {
            Dimc_Macro &mc = blk.macros[m];
            if (mc.filled_job == _this->running_job || mc.fill_job == _this->running_job
                || mc.write_job == _this->running_job) continue;
            mc.kb_ready    = false;
            mc.fb_ready    = false;
            mc.filled_job  = Dimc_Macro::JOB_NONE;
        }

    for (Dimc_InnerBlock &blk : _this->inner_blocks)
        blk.reset_job_state(_this->fill_active);

    _this->state.set(DIMC_STARTING);
    _this->tracer.job_start((uint64_t)_this->clock.get_cycles());

    _this->plan_job();
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
    _this->engine_cycle(DIMC_IDLE);
}

// One engine cycle of the running job, then schedule the next one or, once the job has
// closed, the job end one cycle later. The job closes in the cycle its last row has retired
// and its last result is acknowledged; STORING is the stretch after every row has retired
// while results are still owed. The makespan accrues in fsm_timestamp.
void Dimc_HWPE::fsm_loop()
{
    const uint32_t cur_state = this->state.get();
    uint32_t next_state = cur_state;
    uint32_t latency    = 0;
    if (cur_state == DIMC_STARTING || cur_state == DIMC_STORING) {
        this->engine_cycle(cur_state);
        latency = 1;
        if (this->rows_done() && this->store_done()) {
            this->tracer.job_closed((uint64_t)this->clock.get_cycles());
            next_state = DIMC_FINISHED;
        } else if (cur_state == DIMC_STARTING && this->rows_done()) {
            next_state = DIMC_STORING;
        }
    } else if (cur_state != DIMC_FINISHED) {
        this->trace.fatal("DIMC HWPE FSM: UNKNOWN STATE (%d)!\n", cur_state);
    }
    if (next_state != cur_state) this->tracer.state(next_state);
    this->state.set(next_state);

    if (next_state == DIMC_FINISHED && !this->fsm_end_event->is_enqueued()) {
        this->event_enqueue(this->fsm_end_event, latency);
    } else if (!this->fsm_event->is_enqueued()) {
        this->event_enqueue(this->fsm_event, latency);
    }
}

void Dimc_HWPE::latch_geom(int ctx)
{
    JobGeom &g = this->job_geom[ctx];
    uint32_t n = this->register_file[DIMC_HWPE_NUM_MACROS >> 2];
    if (n == 0 || n > this->num_macros) n = this->num_macros;
    uint32_t r = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_COUNT);
    if (r == 0) {
        this->trace.force_warning("latch_geom: ctx %d has ROW_COUNT=0 (never configured); "
            "running_ctx=%d qlen=%u\n", ctx, this->running_ctx, (unsigned)this->ctx_queue.size());
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
    g.psin_dep       = this->job_reg_ctx(ctx, DIMC_HWPE_PSIN_DEP);
    this->geom_job[ctx] = this->ctx_job_id[ctx];
}

// Latch and size context ctx's job unless its geometry slot already holds it.
void Dimc_HWPE::ensure_geom(int ctx)
{
    if (this->geom_job[ctx] == this->ctx_job_id[ctx]) return;
    this->latch_geom(ctx);
    this->size_fill(ctx);
}

// Sections per macro for one job: kernel rows + NB_VEC x (partial sums + feature).
void Dimc_HWPE::size_fill(int ctx)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    JobGeom &g = this->job_geom[ctx];
    g.kb_beats_per_row   = (DIMC_MACRO_KB_EW + port_bytes - 1) / port_bytes;
    g.fb_beats_per_macro = (DIMC_MACRO_FB_EW + port_bytes - 1) / port_bytes;
    // Per vector: partial sums from the slots a previous job wrote, port_bytes / SLOT rows a word.
    g.psin_beats_per_macro = g.psin_rows
        ? (g.row_count * DIMC_OUT_SLOT_BYTES + port_bytes - 1) / port_bytes : 0;
    g.beats_per_macro = g.kb_sections()
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
    // A macro still holding the running job's operands has rows left to issue against them.
    if (mc.filled_job != this->running_job) mc.fill_done_cycle = 0;
    // A kernel already being fetched ahead for this job keeps its stream position.
    const bool kernel_ahead = mc.kpf_job == mc.fill_job && mc.kfetched > 0;
    this->configure_macro_streams(blk_id, m, ctx, 0, !kernel_ahead, true, true);
}

// Why a new request is refused now: the outstanding depth, else the outer port; 0 if neither.
uint8_t Dimc_HWPE::refusal(const std::queue<uint64_t> &pending, const Dimc_OuterPort &port) const
{
    if (pending.size() >= this->outstanding_depth) return DIMC_WHY_WAIT_DEPTH;
    if (port.busy_until() > (int64_t)this->fsm_timestamp) return DIMC_WHY_WAIT_OUTER_PORT;
    return 0;
}

// Drop every pending beat whose response is due.
static inline void retire_due(std::queue<uint64_t> &q, uint64_t now)
{
    while (!q.empty() && q.front() <= now) q.pop();
}

// Every result beat of the running job issued and acknowledged on this block.
static inline bool block_stored(const Dimc_InnerBlock &blk)
{
    return blk.store.beat_index >= blk.store.beat_total && blk.store_pending.empty();
}

// Blocks book the shared outer port in turn: the first one served rotates every cycle.
uint32_t Dimc_HWPE::block_order(uint32_t i) const
{
    return (uint32_t)((i + this->fsm_timestamp) % this->inner_blocks.size());
}

// Drop the responses that are due, fill and store, and advance the sink's write-back position.
void Dimc_HWPE::retire_block(Dimc_InnerBlock &blk)
{
    retire_due(blk.port_pending, this->fsm_timestamp);
    retire_due(blk.kb_pending, this->fsm_timestamp);
    retire_due(blk.in_pending, this->fsm_timestamp);
    retire_due(blk.store_pending, this->fsm_timestamp);
    while (!blk.run_pending.empty() && blk.run_pending.front().due <= this->fsm_timestamp) {
        const Dimc_InnerBlock::RunDone &d = blk.run_pending.front();
        blk.retired[d.macro].job  = d.job;
        blk.retired[d.macro].runs = d.run + 1;
        blk.run_pending.pop_front();
    }
}

// Run `run` of `job` on macro m reads the partial sums the same macro and run of job - PSIN_DEP
// wrote. Fetch them only once that run is written back.
bool Dimc_HWPE::psum_ready(const Dimc_InnerBlock &blk, uint32_t m, const JobGeom &g,
                           uint32_t job, uint32_t run) const
{
    if (!g.psin_rows) return true;
    if (g.psin_dep == 0 || job < g.psin_dep) return true;
    const int64_t pj = (int64_t)(job - g.psin_dep);
    const Dimc_InnerBlock::Retired &p = blk.retired[m];
    return p.job > pj || (p.job == pj && p.runs > run);
}

// Chain the next queued job's fill program onto each macro whose program for the running job
// is spent, so it may load the next kernel while its sibling still runs its last vectors; then
// note whether any fill is outstanding.
void Dimc_HWPE::advance_fill()
{
    const int nxt = this->ctx_queue.empty() ? -1 : this->ctx_queue.front();
    if (nxt >= 0 && (uint32_t)nxt != this->exec_slot)
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            for (uint32_t m = 0; m < blk.macros.size(); m++) {
                if (blk.macros[m].fill_job != this->running_job) continue;
                if (blk.fill.macro_beat_index[m] < blk.fill.macro_beat_total[m]) continue;
                this->ensure_geom(nxt);
                if (m < this->job_geom[nxt].num_active) this->plan_macro_fill(b, m, nxt);
            }
        }
    this->fill_active = false;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        if (blk.fill.owed() || !blk.wgt_fifo.empty() || !blk.inp_fifo.empty()) this->fill_active = true;
}

// Fill program of one macro for one job: vector 0's partial sums and feature, then the
// kernel rows, then vectors 1..NB_VEC-1, each partial sums then feature.
Dimc_HWPE::BeatPos Dimc_HWPE::beat_pos(const JobGeom &g, uint32_t within) const
{
    const uint32_t fb  = g.fb_beats_per_macro;
    const uint32_t ps  = g.psin_beats_per_macro;
    const uint32_t kb  = g.kb_sections();
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

// The macro holds operands of a job it has not finished triggering. Writing a
// different job's feature or kernel rows into it would change those rows.
bool Dimc_HWPE::holds_unissued(const Dimc_Macro &m) const
{
    return m.filled_job != Dimc_Macro::JOB_NONE
        && !(m.issue_job == m.filled_job && m.runs_issued >= m.job_nb_vec);
}

// First cycle of a job: latch its shape, plan the fill programs not yet chained to it, and
// its store.
void Dimc_HWPE::plan_job()
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t slot = this->exec_slot;
    // Latched once per job: re-latching would rewrite the geometry a chained cursor was built from.
    if (this->geom_job[slot] != this->running_job) this->latch_geom((int)slot);
    this->size_fill((int)slot);
    JobGeom &g = this->job_geom[slot];
    // Plan the macros whose fill program did not chain to this job.
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++)
        for (uint32_t m = 0; m < g.num_active; m++)
            if (this->inner_blocks[b].macros[m].fill_job != this->running_job)
                this->plan_macro_fill(b, m, (int)slot);
    // The store issues from inside the job, as soon as the rows a word carries retire.
    // Port words per macro: SLOT-byte results, port_bytes / SLOT a word.
    g.out_beats = g.nb_vec * ((g.row_count * DIMC_OUT_SLOT_BYTES + port_bytes - 1) / port_bytes);
    // The store cursors were cleared by reset_job_state.
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        blk.store.beat_total = g.num_active * g.out_beats;
}

// One cycle of every block: fetch, write into the macros and trigger, then, per `phase`:
//   DIMC_STARTING  write back, and fetch kernels ahead of the programs if the port is free;
//   DIMC_STORING   the same, writing back only on the blocks whose results are not all out;
//   DIMC_IDLE      the job-end cycle: no write-back, no prefetch, no new fill program.
void Dimc_HWPE::engine_cycle(uint32_t phase)
{
    const bool job_open = phase != DIMC_IDLE;
    const bool storing  = phase == DIMC_STORING;

    // Each macro follows its own fill program, which may be for a later job than the
    // running one. Every request books a whole outer-port word, so at most one fetch request
    // per cycle goes out across all blocks; block_order rotates which block asks first.
    this->fetch_kernels_first();
    for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
        const uint32_t b = this->block_order(i);
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        // One fetch per feed into the dual's FIFOs, which see the FIFOs before this cycle's
        // pops; one pop per FIFO into the macros, before the triggers, so no section enters a
        // macro in a cycle in which it computes; the macros' rows; then the write-back.
        if (blk.fill.owed())
            this->fill_feed(blk, b, false);   // input: feature, partial sums
        if (!blk.kb_fed_first)
            this->fill_feed(blk, b, true);    // kernel rows of the programs
        blk.kb_fed_first = false;
        this->write_feed(blk, b, blk.inp_fifo);
        this->write_feed(blk, b, blk.wgt_fifo);
        this->compute_indep(blk, b);
        if (job_open && !(storing && blk.phase_done)) this->store_block(blk, b);
    }
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) this->tracer.end_cycle(b);

    // Lowest priority: kernels ahead of the programs, only if the outer port is still free.
    if (job_open)
        for (uint32_t i = 0; i < this->inner_blocks.size(); i++) {
            const uint32_t b = this->block_order(i);
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            if (blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) continue;
            this->kernel_prefetch(blk, b);
        }
    this->tracer.port_cycle();
    this->fsm_timestamp++;

    if (job_open) this->advance_fill();

    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        this->retire_block(blk);
        if (storing && block_stored(blk)) blk.phase_done = true;
    }
}

// Every row of the running job has retired, on every block. The pipe may already hold the
// next job's rows, so its emptiness says nothing about this one.
bool Dimc_HWPE::rows_done() const
{
    const JobGeom &g = this->job_geom[this->exec_slot];
    const uint32_t s = this->running_job & 1;
    for (const Dimc_InnerBlock &blk : this->inner_blocks)
        for (uint32_t m = 0; m < g.num_active; m++) {
            const Dimc_Macro &mac = blk.macros[m];
            if (mac.set_job[s] != this->running_job
                || mac.rows_retired_set[s] < g.nb_vec * g.row_count) return false;
        }
    return true;
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
        for (uint32_t m = 0; m < blk.macros.size(); m++) {
            BeatPos pos;
            if (!this->program_next(blk, m, pos) || pos.kind == DIMC_LOAD_KB) continue;
            const Dimc_Macro &t = blk.macros[m];
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
            if (t.kw == 0 || t.kw >= this->job_geom[t.write_slot].kb_sections()) continue;
            uint32_t n = 0;
            for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo) if (q.macro == m) n++;
            if (n > DIMC_KB_FEED_FIRST) continue;
            this->fill_feed(blk, b, true);
            blk.kb_fed_first = true;
            break;
        }
    }
}

// The ADDIN set run `run` of `job` opens is free: the run two before it, or the previous job's
// run of that parity, which read it last, has every row triggered. Nothing triggered yet: runs
// 0 and 1 open unused sets; run r >= 2 reuses run r - 2's. Two or more jobs ahead: not free.
bool Dimc_HWPE::psin_set_free(const Dimc_Macro &mc, uint32_t job, uint32_t run) const
{
    if (mc.issue_job == Dimc_Macro::JOB_NONE) return run < 2u;
    if (mc.issue_job == job) return run < 2u || mc.runs_issued + 1u >= run;
    if (mc.issue_job + 1u == job) return run < 2u && mc.runs_issued + 1u >= mc.job_nb_vec + run;
    return false;
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
        return pos.sub != 0 || this->psin_set_free(mc, job, pos.run);
    }
    if (mc.filled_job != job && this->holds_unissued(mc)) return false;
    // The kernel write port is shut in a trigger cycle (COMPE). A guard: the pops run before
    // the triggers, so holds_unissued already keeps the last row's cycle out.
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

// The next section of macro m's fill program; false when it has none.
bool Dimc_HWPE::program_next(const Dimc_InnerBlock &blk, uint32_t m, BeatPos &pos) const
{
    const Dimc_InnerBlock::Cursor &c = blk.fill;
    if (m >= c.macro_beat_index.size() || c.macro_beat_index[m] >= c.macro_beat_total[m]) return false;
    pos = this->beat_pos(this->job_geom[blk.macros[m].fill_slot], c.macro_beat_index[m]);
    return true;
}

// Kernel feed: among the macros whose next section is a kernel one, the first that can take it
// now, else the first. Only the macro whose kernel is in progress, if any.
uint32_t Dimc_HWPE::pick_kernel(const Dimc_InnerBlock &blk, uint8_t &why) const
{
    const uint32_t nb = (uint32_t)blk.macros.size(), ip = this->kernel_in_progress(blk);
    uint32_t pick = nb;
    for (uint32_t m = 0; m < nb; m++) {
        BeatPos pos;
        if (!this->program_next(blk, m, pos) || pos.kind != DIMC_LOAD_KB) continue;
        if (ip != nb && ip != m) continue;
        const Dimc_Macro &t = blk.macros[m];
        if (this->beat_writable(t, t.fill_job, pos)) return m;
        why = DIMC_WHY_WAIT_COMPUTE;
        if (pick == nb) pick = m;
    }
    return pick;
}

// Input feed, into the dual's shared FIFO while it has room: among the macros that may fetch,
// the one whose section is needed first (fewest rows before its vector may enter, plus kernel
// sections its job still writes). Partial sums only when no macro wants a feature, and once
// their producer is written back. why: a hold on the producer outranks one on compute.
uint32_t Dimc_HWPE::pick_input(const Dimc_InnerBlock &blk, uint8_t &why) const
{
    const uint32_t nb = (uint32_t)blk.macros.size();
    if (blk.inp_fifo.size() >= DIMC_INP_FIFO_DEPTH) return nb;
    uint64_t best_u = ~(uint64_t)0;
    uint32_t pick = nb;
    for (int pass = 0; pass < 2 && pick == nb; pass++) {
        for (uint32_t m = 0; m < nb; m++) {
            BeatPos pos;
            if (!this->program_next(blk, m, pos) || pos.kind == DIMC_LOAD_KB) continue;
            const Dimc_Macro &t = blk.macros[m];
            const JobGeom &fg = this->job_geom[t.fill_slot];
            const bool psum = pos.kind == DIMC_LOAD_PSIN;
            if (pass == 0 && psum) continue;
            if (psum && !this->psum_ready(blk, m, fg, t.fill_job, pos.run)) { why = DIMC_WHY_WAIT_PSUM; continue; }
            // fill_beat marks a run's set outstanding at the run's first partial-sum section, so
            // that section is fetched only once no earlier run still triggers rows of the set.
            if (psum && pos.sub == 0 && !this->beat_writable(t, t.fill_job, pos)) {
                if (why != DIMC_WHY_WAIT_PSUM) why = DIMC_WHY_WAIT_COMPUTE;
                continue;
            }
            // The shared FIFO pops in order: a feature section is fetched once the rows it waits
            // for (the previous run, or the macro's current job) are at most
            // DIMC_INP_FETCH_LEAD from triggered, so a head its macro cannot take yet holds the
            // other macro's sections for at most that long.
            if (!psum && !this->beat_writable(t, t.fill_job, pos)
                && this->rows_before_feature(t, t.fill_job, pos.run) > DIMC_INP_FETCH_LEAD) {
                if (why != DIMC_WHY_WAIT_PSUM) why = DIMC_WHY_WAIT_COMPUTE;
                continue;
            }
            uint64_t u = this->rows_before_feature(t, t.fill_job, pos.run);
            const uint32_t kb_all = fg.kb_sections();
            const uint32_t kw = t.write_job == t.fill_job ? t.kw : 0;
            if (kw < kb_all) u += kb_all - kw;
            if (u < best_u) { best_u = u; pick = m; }
        }
    }
    return pick;
}

// One section of a feed per cycle, from L1 into the dual's FIFO, for the macro the feed's
// picker selects. A request needs the outstanding depth and the outer port; the kept second
// section of a port word needs neither.
void Dimc_HWPE::fill_feed(Dimc_InnerBlock &blk, uint32_t blk_id, bool kernel_feed)
{
    Dimc_InnerBlock::Cursor &cursor = blk.fill;
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
            }
        }
        if (blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) {
            this->tracer.feed_skip(blk_id, true, DIMC_WHY_WAIT_DEPTH);
            return;
        }
    } else if (!cursor.owed()) {
        return;
    }
    uint8_t why = 0;
    const uint32_t macro = kernel_feed ? this->pick_kernel(blk, why) : this->pick_input(blk, why);
    if (macro == blk.macros.size()) {
        if (why) this->tracer.fill_skip(blk_id, why);
        return;
    }
    BeatPos pos;
    this->program_next(blk, macro, pos);
    const Dimc_HWPE_Streamer &st = pos.kind == DIMC_LOAD_KB ? blk.weight_stream[macro]
                                 : pos.kind == DIMC_LOAD_FB ? blk.input_stream[macro] : blk.psin_stream[macro];
    why = st.pair_ready() ? 0 : this->refusal(kernel_feed ? blk.kb_pending : blk.in_pending,
                                              this->outer_port_in);
    if (why) {
        this->tracer.feed_skip(blk_id, kernel_feed, why);
        return;
    }
    this->fill_beat(blk, blk_id, macro);
}

// The macro whose kernel is partly fetched (started, not finished), or nb.
// Its sections must all go in before any other macro's, or the FIFO holds two kernels
// interleaved and the first macro waits behind the second's compute.
uint32_t Dimc_HWPE::kernel_in_progress(const Dimc_InnerBlock &blk) const
{
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        const Dimc_Macro &t = blk.macros[m];
        if (t.kpf_job == Dimc_Macro::JOB_NONE || t.kfetched == 0) continue;
        const JobGeom &g = this->job_geom[t.kpf_slot];
        if (this->geom_job[t.kpf_slot] != t.kpf_job) continue;
        if (t.kfetched < g.kb_sections()) return m;
    }
    return (uint32_t)blk.macros.size();
}

// Fetch one kernel section of a macro's next job (or of its current job whose program has
// not reached the kernel) into the weight FIFO, ahead of the program. One macro at a time:
// the one in progress, else the lowest-index one with a kernel left to fetch.
bool Dimc_HWPE::kernel_prefetch(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    Dimc_InnerBlock::Cursor &cur = blk.fill;
    const uint32_t nb = (uint32_t)blk.macros.size();
    const uint32_t ip = this->kernel_in_progress(blk);
    // Macros whose program has not reached the kernel sections of its job: that kernel, not
    // the next job's, is fetched ahead.
    uint32_t own = 0;
    for (uint32_t m = 0; m < nb && m < cur.macro_beat_index.size(); m++) {
        const Dimc_Macro &t = blk.macros[m];
        if (m == ip || t.fill_job == Dimc_Macro::JOB_NONE) continue;
        const JobGeom &g = this->job_geom[t.fill_slot];
        if (cur.macro_beat_index[m] < g.psin_beats_per_macro + g.fb_beats_per_macro
            && !(t.kpf_job == t.fill_job && t.kfetched >= g.kb_sections()))
            own |= 1u << m;
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
    uint32_t pick = nb;
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
        const uint32_t job = this->ctx_job_id[slot];
        this->ensure_geom(slot);
        const JobGeom &gn = this->job_geom[slot];
        if (m >= gn.num_active) continue;
        const uint32_t done = t.kpf_job == job ? t.kfetched : 0;
        if (done >= gn.kb_sections()) continue;
        pick = m; pick_slot = slot;
    }
    if (pick == nb) return false;
    // Fetch ahead only while the FIFO holds no other macro's sections.
    for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
        if (q.macro != pick) return false;
    return this->fetch_kernel_ahead(blk, blk_id, pick, pick_slot, true);
}

// Fetch the next kernel section of the job in context ctx for macro m, ahead of its program.
// False when the outstanding depth or the outer port refuses the request; trace_refusal
// reports the outer-port refusal.
bool Dimc_HWPE::fetch_kernel_ahead(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m, int ctx,
                                   bool trace_refusal)
{
    Dimc_Macro &fm = blk.macros[m];
    const uint32_t job = this->ctx_job_id[ctx];
    if (fm.kpf_job != job) {
        fm.kpf_job = job; fm.kpf_slot = (uint32_t)ctx; fm.kfetched = 0;
        this->configure_macro_streams(blk_id, m, ctx, 0, true, false, false);
    }
    const uint8_t why = blk.weight_stream[m].pair_ready() ? 0 : this->refusal(blk.kb_pending, this->outer_port_in);
    if (why) {
        if (trace_refusal && why == DIMC_WHY_WAIT_OUTER_PORT) this->tracer.kernel_skip(blk_id, why);
        return false;
    }
    this->fetch_kernel_section(blk, blk_id, m, ctx, job, fm.kfetched);
    return true;
}

// Kernel section `sub` of `job` (context ctx) for macro m, into the weight FIFO.
void Dimc_HWPE::fetch_kernel_section(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m, int ctx,
                                     uint32_t job, uint32_t sub)
{
    const JobGeom &g = this->job_geom[ctx];
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t off = (sub % g.kb_beats_per_row) * port_bytes;
    Dimc_InnerBlock::FeedEntry e;
    e.macro = m; e.slot = (uint32_t)ctx; e.job = job;
    e.within = g.psin_beats_per_macro + g.fb_beats_per_macro + sub;
    e.bytes = DIMC_MACRO_KB_EW - off < port_bytes ? DIMC_MACRO_KB_EW - off : port_bytes;
    this->push_section(blk, blk_id, blk.weight_stream[m], e, blk.wgt_fifo, blk.kb_pending, DIMC_LOAD_KB);
    blk.macros[m].kfetched++;
}

// Issue section e from streamer st and queue it in fifo, landed the cycle after its
// response (fifo_v3, not fall-through). A request books the outer port and `pending`.
void Dimc_HWPE::push_section(Dimc_InnerBlock &blk, uint32_t blk_id, Dimc_HWPE_Streamer &st,
                             Dimc_InnerBlock::FeedEntry &e,
                             std::deque<Dimc_InnerBlock::FeedEntry> &fifo,
                             std::queue<uint64_t> &pending, uint8_t kind)
{
    int lat = st.issue_beat((int)e.bytes, e.data);
    const bool requested = lat != Dimc_HWPE_Streamer::NO_REQUEST;
    if (lat < 1) lat = requested ? 1 : 0;
    e.ready = this->fsm_timestamp + (uint64_t)lat + 1;
    fifo.push_back(e);
    if (!requested) return;
    const uint64_t due = this->fsm_timestamp + (uint64_t)lat;
    this->outer_port_in.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
    this->tracer.outer_port_booked(blk_id, kind);
    pending.push(due);
    blk.port_pending.push(due);
}

// One cycle while the engine is idle and the queue is held. Each dual fetches the first held
// job's kernel of its first macro into its weight FIFO; the job's fill program then skips what
// is fetched (kfetched).
void Dimc_HWPE::held_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    if (_this->job_running || _this->ctx_queue.empty()) return;
    const int ctx = _this->ctx_queue.front();
    _this->ensure_geom(ctx);
    bool more = false;
    for (uint32_t b = 0; b < _this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = _this->inner_blocks[b];
        if (_this->held_kernel_step(blk, b, ctx)) more = true;
        // The FIFO keeps the kernel: run until its last section has landed.
        for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
            if (q.ready > _this->fsm_timestamp) more = true;
    }
    for (uint32_t b = 0; b < _this->inner_blocks.size(); b++) _this->tracer.end_cycle(b);
    _this->tracer.port_cycle();
    // A call with nothing to do (the one after the last section has landed) only records
    // the idle cycle, so the trace does not keep the last load until the job starts.
    if (!more) return;
    _this->fsm_timestamp++;
    for (Dimc_InnerBlock &blk : _this->inner_blocks) _this->retire_block(blk);
    _this->event_enqueue(_this->held_event, 1);
}

// Fetch one kernel section of the held job for the dual's first macro, while the FIFO holds
// no other macro's sections. False when that kernel is all fetched.
bool Dimc_HWPE::held_kernel_step(Dimc_InnerBlock &blk, uint32_t blk_id, int ctx)
{
    if (blk.macros.empty()) return false;
    const Dimc_Macro &t = blk.macros[0];
    if (t.kpf_job == this->ctx_job_id[ctx] && t.kfetched >= this->job_geom[ctx].kb_sections()) return false;
    for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
        if (q.macro != 0) return true;          // the previous macro's kernel drains first
    if (blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) return true;
    this->fetch_kernel_ahead(blk, blk_id, 0, ctx, false);
    return true;
}

// Fetch one section of macro `macro`'s program from L1 into the dual's FIFO.
void Dimc_HWPE::fill_beat(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t macro)
{
    Dimc_InnerBlock::Cursor &cursor = blk.fill;
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t within = cursor.macro_beat_index[macro];
    Dimc_Macro &fm = blk.macros[macro];
    const JobGeom &fg = this->job_geom[fm.fill_slot];
    const BeatPos  pos = this->beat_pos(fg, within);
    const uint32_t pset    = (fm.fill_job * fg.nb_vec + pos.run) & 1;
    const uint8_t  kind    = pos.kind;

    // First section of a run: mark its partial sums outstanding, in the run's set, which
    // no row still to trigger reads.
    if (pos.sub == 0 && kind == (fg.psin_beats_per_macro ? DIMC_LOAD_PSIN : DIMC_LOAD_FB))
        for (uint64_t &r : fm.psin_row_ready[pset]) r = fg.psin_rows ? ~(uint64_t)0 : 0;
    // Vectors after the first read their own feature and partial-sum blocks.
    if (pos.run >= 1 && pos.sub == 0 && kind != DIMC_LOAD_KB)
        this->configure_macro_streams(blk_id, macro, (int)fm.fill_slot, pos.run,
                                      false, kind == DIMC_LOAD_FB, kind == DIMC_LOAD_PSIN);

    if (kind == DIMC_LOAD_KB) {
        if (fm.kpf_job != fm.fill_job) { fm.kpf_job = fm.fill_job; fm.kpf_slot = fm.fill_slot; fm.kfetched = 0; }
        this->fetch_kernel_section(blk, blk_id, macro, (int)fm.fill_slot, fm.fill_job, pos.sub);
    } else {
        Dimc_InnerBlock::FeedEntry e;
        e.macro = macro; e.slot = fm.fill_slot; e.job = fm.fill_job; e.within = within;
        if (kind == DIMC_LOAD_FB) {
            const uint32_t off = pos.sub * port_bytes;
            e.bytes = DIMC_MACRO_FB_EW - off < port_bytes ? DIMC_MACRO_FB_EW - off : port_bytes;
        } else {
            const uint32_t per   = port_bytes / DIMC_OUT_SLOT_BYTES;
            const uint32_t first = pos.sub * per;
            const uint32_t n     = fg.row_count - first < per ? fg.row_count - first : per;
            e.bytes = n * DIMC_OUT_SLOT_BYTES;
        }
        this->push_section(blk, blk_id, kind == DIMC_LOAD_FB ? blk.input_stream[macro] : blk.psin_stream[macro],
                           e, blk.inp_fifo, blk.in_pending, kind);
    }
    cursor.macro_beat_index[macro]++;
}

// Pop the head of one feed's FIFO into its macro, once landed and once the macro's
// write port takes it (head-of-line: a section the macro cannot take yet holds the
// sections behind it).
void Dimc_HWPE::write_feed(Dimc_InnerBlock &blk, uint32_t blk_id,
                           std::deque<Dimc_InnerBlock::FeedEntry> &fifo)
{
    if (fifo.empty() || fifo.front().ready > this->fsm_timestamp) return;
    const Dimc_InnerBlock::FeedEntry &e = fifo.front();
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
        mc.write_job = e.job; mc.write_slot = e.slot;
        mc.written = 0; mc.kw = 0; mc.f0 = false; mc.stamped = false;
    }
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t pset       = (e.job * g.nb_vec + pos.run) & 1;
    bool feature_done = false;

    if (pos.kind == DIMC_LOAD_KB) {
        mc.kw++;
        const uint32_t row = pos.sub / g.kb_beats_per_row;
        const uint32_t sub = pos.sub % g.kb_beats_per_row;
        std::memcpy(mc.kb_row_buffer + sub * port_bytes, e.data, e.bytes);
        if (sub == g.kb_beats_per_row - 1)   // row complete -> commit
            mc.write_row((int)((g.row_base + row) % DIMC_MACRO_KB_LEN), mc.kb_row_buffer);
    } else if (pos.kind == DIMC_LOAD_FB) {
        std::memcpy(mc.row_buffer + pos.sub * port_bytes, e.data, e.bytes);
        if (pos.sub == g.fb_beats_per_macro - 1) {   // feature complete
            mc.write_fb(mc.row_buffer);
            mc.kb_ready = mc.fb_ready = true;
            // The pipe is NOT cleared here: when the fill runs ahead, it still holds
            // the previous job's last rows, triggered before this feature landed.
            mc.psin_scalar = (int32_t)this->job_reg_ctx((int)e.slot, DIMC_HWPE_PSIN);
            mc.fb_run = pos.run;
            mc.fb_ready_cycle = this->fsm_timestamp + 1;
            feature_done = true;
            mc.f0 = mc.f0 || pos.run == 0;
        }
    } else {
        // Rows [first, first + n) in one port word; each row's value is the low 4 bytes
        // of its SLOT-byte slot.
        const uint32_t per   = port_bytes / DIMC_OUT_SLOT_BYTES;
        const uint32_t first = pos.sub * per;
        const uint32_t n     = e.bytes / DIMC_OUT_SLOT_BYTES;
        // beat_writable already holds a section whose set is not free: a consistency check.
        if (pos.sub == 0 && !this->psin_set_free(mc, e.job, pos.run))
            this->trace.force_warning("DIMC psin set overwrite: macro %u job %u run %u set %u while the "
                "set's previous run still has rows to trigger\n", (unsigned)e.macro, e.job, pos.run, pset);
        for (uint32_t j = 0; j < n; j++) {
            const uint32_t row_idx = (g.row_base + first + j) % DIMC_MACRO_KB_LEN;
            mc.write_psin_row((int)row_idx, e.data + j * DIMC_OUT_SLOT_BYTES, (int)pset);
            mc.psin_row_ready[pset][row_idx] = this->fsm_timestamp + 1;
        }
    }
    const uint32_t within = e.within, macro = e.macro;
    fifo.pop_front();
    const bool macro_filled = ++mc.written >= g.beats_per_macro;
    // Vector 0's feature and the kernel rows are in: hand the macro to the compute side, with
    // the shape of the job it now holds. The partial sums are gated per row (psin_row_ready).
    if (!mc.stamped && mc.f0 && mc.kw >= g.kb_sections()) {
        mc.stamped = true;
        mc.fill_done_cycle = this->fsm_timestamp + 1;
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

// Each macro issues its own rows as soon as its own operands have landed, so a macro still
// being filled does not hold back a sibling that is ready to compute.
void Dimc_HWPE::compute_indep(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    // Every row a macro has finished goes into that macro's out_fifo. A macro is never stalled
    // for its out_fifo: a result the FIFO cannot take is lost, as in the RTL. The model also
    // warns; force_warning exits under werror.
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        if (!blk.macros[m].has_ready()) continue;
        DimcPipeEntry e = blk.macros[m].drain();
        blk.macros[m].rows_retired_set[e.set]++;
        if (blk.out_fifo[m].size() >= DIMC_OUT_FIFO_DEPTH / blk.macros.size()) {
            if (++this->out_dropped <= 8)
                this->trace.force_warning("DIMC result lost: macro %u row %d (out_fifo full)\n",
                                          m, e.job_row);
            continue;
        }
        blk.out_fifo[m].push_back({e.psout, (uint16_t)e.job_row, (uint16_t)e.run});
        blk.out_accum.push(e.psout);   // summed as it enters the out_fifo
    }

    // Each macro triggers whenever its own vector is ready; results go to its own out_fifo,
    // so two macros may trigger in one cycle. A macro holding the next job's operands issues
    // its rows too, only into free out_fifo entries.
    const uint32_t J = this->running_job;
    const size_t fifo_cap = DIMC_OUT_FIFO_DEPTH / blk.macros.size();
    for (uint32_t m = 0; m < (uint32_t)blk.macros.size(); m++) {
        Dimc_Macro &mac = blk.macros[m];
        const bool ahead = mac.filled_job == J + 1;
        if (mac.filled_job != Dimc_Macro::JOB_NONE && (mac.filled_job == J || ahead)) {
            const uint32_t jj = mac.filled_job;
            if (mac.issue_job != jj) {
                const uint32_t s = jj & 1;
                mac.issue_job = jj;
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

bool Dimc_HWPE::store_done() const
{
    for (const Dimc_InnerBlock &blk : this->inner_blocks)
        if (!block_stored(blk)) return false;
    return true;
}

// One port word per dual per cycle, straight from a macro's out_fifo: its head entries are the
// running job's next results in order (row k % rows of vector k / rows). The lowest-index macro
// holding a whole word goes; an unexpected head fails the run (the RTL would misplace it). What
// is behind a macro's results of the running job waits for the next job.
void Dimc_HWPE::store_block(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const JobGeom &g = this->job_geom[this->exec_slot];
    for (uint32_t m = 0; m < g.num_active; m++) {
        if (blk.out_results[m] >= g.row_count * g.nb_vec) continue;
        if (this->store_word(blk, blk_id, m) >= 0) return;
    }
}

// Write the next port word of macro m's results of the running job. -1: no whole word in order
// at the head; 0: the outstanding depth or the outer port refused it; 1: written.
int Dimc_HWPE::store_word(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t ctx        = this->exec_slot;
    const JobGeom &g          = this->job_geom[ctx];
    const uint32_t rows       = g.row_count;
    const uint32_t per_word   = port_bytes / DIMC_OUT_SLOT_BYTES;
    std::deque<Dimc_InnerBlock::OutEntry> &fifo = blk.out_fifo[m];
    uint32_t &k = blk.out_results[m];
    const uint32_t r = k % rows, vec = k / rows;
    const uint32_t n = rows - r < per_word ? rows - r : per_word;
    if (fifo.size() < n) return -1;
    for (uint32_t j = 0; j < n; j++) {
        if (fifo[j].row == r + j && fifo[j].run == vec) continue;
        this->trace.force_warning("DIMC out_fifo order: macro %u head is vector %u row %u, the "
            "sink expects vector %u row %u; the RTL would misplace it\n",
            m, fifo[0].run, fifo[0].row, vec, r);
        return -1;
    }
    if (const uint8_t why = this->refusal(blk.store_pending, this->outer_port_out)) {
        this->tracer.store_skip(blk_id, why);
        return 0;
    }
    // First word of a run: point the macro's output streamer at the run's block.
    if (r == 0) this->configure_out_stream(blk_id, m, vec, ctx);
    uint8_t word[32];
    for (uint32_t j = 0; j < n; j++) {
        uint8_t *slot = word + j * DIMC_OUT_SLOT_BYTES;
        std::memset(slot, fifo[j].psout < 0 ? 0xFF : 0x00, DIMC_OUT_SLOT_BYTES);
        std::memcpy(slot, &fifo[j].psout, 4);
    }
    int lat = blk.out_stream[m].issue_beat((int)(n * DIMC_OUT_SLOT_BYTES), word);
    this->tracer.store_beat(blk_id, m, lat,
                            g.nb_vec * ((rows * DIMC_OUT_SLOT_BYTES + port_bytes - 1) / port_bytes));
    if (lat < 1) lat = 1;
    const uint64_t due = this->fsm_timestamp + (uint64_t)lat;
    blk.port_pending.push(due);
    blk.store_pending.push(due);
    this->outer_port_out.request((int64_t)this->fsm_timestamp, this->outer_port_bytes);
    this->tracer.outer_port_booked(blk_id, DIMC_PORT_WB);
    if (r + n == rows)
        blk.run_pending.push_back({due, this->running_job, m, vec});
    fifo.erase(fifo.begin(), fifo.begin() + n);
    k += n;
    blk.store.beat_index++;
    return 1;
}
