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
    // ci, sign_8b and compute_mask travel with each job's operands (write_feed).
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
    uint32_t r = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_COUNT);
    if (r == 0) {
        this->trace.force_warning("latch_geom: ctx %d has ROW_COUNT=0 (never configured); "
            "running_ctx=%d qlen=%u\n", ctx, this->running_ctx, (unsigned)this->ctx_queue.size());
        r = 1;
    }
    if (r > DIMC_MACRO_KB_LEN) r = DIMC_MACRO_KB_LEN;

    g.row_count   = r;
    g.row_base    = this->job_reg_ctx(ctx, DIMC_HWPE_ROW_SEL_BASE);
    g.compute_cyc = r + DIMC_MACRO_LATENCY;
    g.tile_last   = this->job_reg_ctx(ctx, DIMC_HWPE_TILE_LAST) & 0x1;
    g.nb_vec         = this->job_reg_ctx(ctx, DIMC_HWPE_NB_VEC);
    if (g.nb_vec == 0) g.nb_vec = 1;
    if (g.nb_vec > DIMC_ACC_VECTORS)
        this->trace.fatal("NB_VEC %u: the dual has accumulators for %u vectors\n", g.nb_vec, DIMC_ACC_VECTORS);
    g.fb_vec_stride  = this->job_reg_ctx(ctx, DIMC_HWPE_FB_VEC_STRIDE);
    g.out_row_stride = this->job_reg_ctx(ctx, DIMC_HWPE_OUT_ROW_STRIDE);
    this->geom_job[ctx] = this->ctx_job_id[ctx];
}

// Latch and size context ctx's job unless its geometry slot already holds it.
void Dimc_HWPE::ensure_geom(int ctx)
{
    if (this->geom_job[ctx] == this->ctx_job_id[ctx]) return;
    this->latch_geom(ctx);
    this->size_fill(ctx);
}

// Sections per macro for one job: the kernel rows and NB_VEC feature vectors.
void Dimc_HWPE::size_fill(int ctx)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    JobGeom &g = this->job_geom[ctx];
    g.kb_beats_per_row   = (DIMC_MACRO_KB_EW + port_bytes - 1) / port_bytes;
    g.fb_beats_per_macro = (DIMC_MACRO_FB_EW + port_bytes - 1) / port_bytes;
    g.beats_per_macro    = g.kb_sections() + g.nb_vec * g.fb_beats_per_macro;
}

// Start macro m's fill program for the job in context ctx (already latched and sized).
void Dimc_HWPE::plan_macro_fill(uint32_t blk_id, uint32_t m, int ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    Dimc_InnerBlock &blk = this->inner_blocks[blk_id];
    Dimc_InnerBlock::FillCursor &cur = blk.fill;
    if (m != this->job_macro(this->ctx_job_id[ctx])) return;
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
    this->configure_macro_streams(blk_id, m, ctx, 0, !kernel_ahead, true);
}

// Why a new request is refused now: the outstanding depth, else the port; 0 if neither.
uint8_t Dimc_HWPE::refusal(const std::queue<uint64_t> &pending, const Dimc_Port &port) const
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

// Drop the responses that are due, fill and store.
void Dimc_HWPE::retire_block(Dimc_InnerBlock &blk)
{
    retire_due(blk.port_pending, this->fsm_timestamp);
    retire_due(blk.kb_pending, this->fsm_timestamp);
    retire_due(blk.in_pending, this->fsm_timestamp);
    retire_due(blk.store_pending, this->fsm_timestamp);
}

// Start the next queued job's fill program on its macro once that macro's previous program is
// spent, so it is loaded while the other macro computes; then note whether any fill is outstanding.
void Dimc_HWPE::advance_fill()
{
    const int nxt = this->ctx_queue.empty() ? -1 : this->ctx_queue.front();
    if (nxt >= 0 && (uint32_t)nxt != this->exec_slot) {
        const uint32_t job = this->ctx_job_id[nxt], m = this->job_macro(job);
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            if (blk.macros[m].fill_job == job) continue;
            if (m < blk.fill.macro_beat_index.size()
                && blk.fill.macro_beat_index[m] < blk.fill.macro_beat_total[m]) continue;
            this->ensure_geom(nxt);
            this->plan_macro_fill(b, m, nxt);
        }
    }
    this->fill_active = false;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        if (blk.fill.owed() || !blk.wgt_fifo.empty() || !blk.inp_fifo.empty()) this->fill_active = true;
}

// Fill program of one macro for one job: vector 0's feature, then the kernel rows, then the
// features of vectors 1..NB_VEC-1.
Dimc_HWPE::BeatPos Dimc_HWPE::beat_pos(const JobGeom &g, uint32_t within) const
{
    const uint32_t fb = g.fb_beats_per_macro;
    const uint32_t kb = g.kb_sections();
    if (within < fb)      return {DIMC_LOAD_FB, 0, within};
    if (within < fb + kb) return {DIMC_LOAD_KB, 0, within - fb};
    const uint32_t w = within - fb - kb;
    return {DIMC_LOAD_FB, 1 + w / fb, w % fb};
}

// The macro holds operands of a job it has not finished triggering. Writing a
// different job's feature or kernel rows into it would change those rows.
bool Dimc_HWPE::holds_unissued(const Dimc_Macro &m) const
{
    return m.filled_job != Dimc_Macro::JOB_NONE
        && !(m.issue_job == m.filled_job && m.runs_issued >= m.job_nb_vec);
}

// First cycle of a job: latch its shape, plan its macro's fill program unless it started, and
// its store.
void Dimc_HWPE::plan_job()
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t slot = this->exec_slot;
    // Latched once per job: re-latching would rewrite the geometry a fill program was built from.
    if (this->geom_job[slot] != this->running_job) this->latch_geom((int)slot);
    this->size_fill((int)slot);
    JobGeom &g = this->job_geom[slot];
    // Plan the job's macro unless its fill program already started.
    const uint32_t pm = this->job_macro(this->running_job);
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++)
        if (this->inner_blocks[b].macros[pm].fill_job != this->running_job)
            this->plan_macro_fill(b, pm, (int)slot);
    // Port words of one output tile: one per row, NB_VEC results each. Only a TILE_LAST job writes.
    g.out_beats = g.row_count;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        blk.store.beat_total = g.tile_last ? g.out_beats : 0;
}

// One cycle of every block: fetch, write into the macros, accumulate and trigger; while a job
// is open (not in its job-end cycle, DIMC_IDLE) also the write-back and the next job's fill
// program. In DIMC_STORING a block whose results are all out stops writing back.
void Dimc_HWPE::engine_cycle(uint32_t phase)
{
    const bool job_open = phase != DIMC_IDLE;
    const bool storing  = phase == DIMC_STORING;

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        // One fetch per feed, seeing the FIFOs before this cycle's pops; one pop per FIFO
        // into the macros, before the triggers, so no section enters a macro in a cycle in
        // which it computes; the macros' rows; then the write-back.
        this->fill_feed(blk, b, false);   // feature vectors
        this->fill_feed(blk, b, true);    // kernel rows
        this->write_feed(blk, b, blk.inp_fifo);
        this->write_feed(blk, b, blk.wgt_fifo);
        this->compute_indep(blk, b);
        if (job_open && !(storing && blk.phase_done)) this->store_block(blk, b);
    }
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) this->tracer.end_cycle(b);
    this->tracer.port_cycle();
    this->fsm_timestamp++;

    if (job_open) this->advance_fill();

    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        this->retire_block(blk);
        if (storing && block_stored(blk)) blk.phase_done = true;
    }
}

// Every row of the running job has retired on this block and every result is accumulated.
bool Dimc_HWPE::tile_computed(const Dimc_InnerBlock &blk) const
{
    const JobGeom &g = this->job_geom[this->exec_slot];
    const uint32_t s = this->running_job & 1;
    const Dimc_Macro &mac = blk.macros[this->job_macro(this->running_job)];
    return mac.set_job[s] == this->running_job
        && mac.rows_retired_set[s] >= g.nb_vec * g.row_count && blk.out_fifo.empty();
}

bool Dimc_HWPE::rows_done() const
{
    for (const Dimc_InnerBlock &blk : this->inner_blocks)
        if (!this->tile_computed(blk)) return false;
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
    else if (mc.issue_job + this->num_macros == job && !run) target = mc.job_nb_vec;   // its previous job
    else                                        return 0xFFFFFFFFu;
    if (mc.runs_issued >= target) return 0;
    const uint32_t cur = mc.rows_issued < mc.job_rows ? mc.rows_issued : 0;
    return (target - mc.runs_issued) * mc.job_rows - cur;
}

// May this section be written into the macro now? Triggering shuts the kernel write port, and
// a new feature would change the rows still to come: a feature of vector b >= 1 waits until
// vector b - 1 is fully triggered (one feature buffer).
bool Dimc_HWPE::beat_writable(const Dimc_Macro &mc, uint32_t job, const BeatPos &pos) const
{
    if (mc.filled_job != job && this->holds_unissued(mc)) return false;
    // The kernel write port is shut in a trigger cycle (COMPE). A guard: the pops run before
    // the triggers, so holds_unissued already keeps the last row's cycle out.
    if (pos.kind == DIMC_LOAD_KB && mc.last_trigger_cycle == (int64_t)this->fsm_timestamp)
        return false;
    // A kernel replaces the one every earlier job of the macro computes with, so the macro
    // must have finished triggering every job before it: `owed` counts its fill programs
    // whose job is not fully triggered. A held job's kernel is fetched before its program
    // starts, so it may be for a job after fill_job.
    if (pos.kind == DIMC_LOAD_KB && mc.fill_job != Dimc_Macro::JOB_NONE) {
        if (job - mc.fill_job == 1u) {
            if (mc.owed != 0u) return false;
        } else if (mc.fill_job - job < 0x80000000u) {
            if (mc.owed > mc.fill_job - job + 1u) return false;
        } else {
            return false;
        }
    }
    // The job being written is not handed to the compute side yet: another job's sections
    // would overwrite it.
    if (mc.write_job != Dimc_Macro::JOB_NONE && mc.write_job != job && !mc.stamped) return false;
    if (pos.kind == DIMC_LOAD_FB && pos.run >= 1
        && !(mc.issue_job == job && mc.runs_issued >= pos.run)) return false;
    return true;
}

// The next section of macro m's fill program; false when it has none.
bool Dimc_HWPE::program_next(const Dimc_InnerBlock &blk, uint32_t m, BeatPos &pos) const
{
    const Dimc_InnerBlock::FillCursor &c = blk.fill;
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
// sections its job still writes).
uint32_t Dimc_HWPE::pick_input(const Dimc_InnerBlock &blk, uint8_t &why) const
{
    const uint32_t nb = (uint32_t)blk.macros.size();
    if (blk.inp_fifo.size() >= DIMC_INP_FIFO_DEPTH) return nb;
    uint64_t best_u = ~(uint64_t)0;
    uint32_t pick = nb;
    for (uint32_t m = 0; m < nb; m++) {
        BeatPos pos;
        if (!this->program_next(blk, m, pos) || pos.kind == DIMC_LOAD_KB) continue;
        const Dimc_Macro &t = blk.macros[m];
        const JobGeom &fg = this->job_geom[t.fill_slot];
        // The shared FIFO pops in order: a feature section is fetched once the rows it waits
        // for (the previous vector, the macro's previous job, and for a vector after the first
        // the other macro's turn) are at most DIMC_INP_FETCH_LEAD from triggered, so a head its
        // macro cannot take yet holds the other macro's sections for at most that long.
        const uint64_t turn = pos.run ? this->rows_before_turn(blk, m, t.fill_job) : 0;
        if (!this->beat_writable(t, t.fill_job, pos)
            && this->rows_before_feature(t, t.fill_job, pos.run) + turn > DIMC_INP_FETCH_LEAD) {
            why = DIMC_WHY_WAIT_COMPUTE;
            continue;
        }
        uint64_t u = this->rows_before_feature(t, t.fill_job, pos.run);
        const uint32_t kb_all = fg.kb_sections();
        const uint32_t kw = t.write_job == t.fill_job ? t.kw : 0;
        if (kw < kb_all) u += kb_all - kw;
        if (u < best_u) { best_u = u; pick = m; }
    }
    return pick;
}

// One section of a feed, from L1 into the dual's FIFO, for the macro the feed's picker selects.
// A request needs the outstanding depth and the feed's port.
void Dimc_HWPE::fill_feed(Dimc_InnerBlock &blk, uint32_t blk_id, bool kernel_feed)
{
    Dimc_InnerBlock::FillCursor &cursor = blk.fill;
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
    why = kernel_feed ? this->refusal(blk.kb_pending, blk.kernel_port)
                      : this->refusal(blk.in_pending, blk.input_port);
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

// Fetch the next kernel section of the held job in context ctx for macro m, before its fill
// program starts. False when the outstanding depth or the kernel port refuses the request.
bool Dimc_HWPE::fetch_kernel_ahead(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t m, int ctx)
{
    Dimc_Macro &fm = blk.macros[m];
    const uint32_t job = this->ctx_job_id[ctx];
    if (fm.kpf_job != job) {
        fm.kpf_job = job; fm.kpf_slot = (uint32_t)ctx; fm.kfetched = 0;
        this->configure_macro_streams(blk_id, m, ctx, 0, true, false);
    }
    if (this->refusal(blk.kb_pending, blk.kernel_port)) return false;
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
    e.within = g.fb_beats_per_macro + sub;
    e.bytes = DIMC_MACRO_KB_EW - off < port_bytes ? DIMC_MACRO_KB_EW - off : port_bytes;
    this->push_section(blk, blk_id, blk.weight_stream[m], e, blk.wgt_fifo, blk.kb_pending, DIMC_LOAD_KB);
    blk.macros[m].kfetched++;
}

// Issue section e from streamer st on its feed's port and queue it in fifo, landed the cycle
// after its last byte's response (fifo_v3, not fall-through).
void Dimc_HWPE::push_section(Dimc_InnerBlock &blk, uint32_t blk_id, Dimc_HWPE_Streamer &st,
                             Dimc_InnerBlock::FeedEntry &e,
                             std::deque<Dimc_InnerBlock::FeedEntry> &fifo,
                             std::queue<uint64_t> &pending, uint8_t kind)
{
    Dimc_Port &port = kind == DIMC_LOAD_KB ? blk.kernel_port : blk.input_port;
    int lat = st.issue_beat((int)e.bytes, e.data);
    if (lat < 1) lat = 1;
    port.request((int64_t)this->fsm_timestamp, this->inner_port_bytes);
    // The section has crossed the port when the port is free again.
    const uint64_t due = (uint64_t)port.busy_until() - 1 + (uint64_t)lat;
    e.ready = due + 1;
    fifo.push_back(e);
    this->tracer.outer_port_booked(blk_id, kind);
    pending.push(due);
    blk.port_pending.push(due);
}

// One cycle while the engine is idle and the queue is held. Each dual fetches the first held
// job's kernel into its weight FIFO; the job's fill program then skips what is fetched (kfetched).
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

// Fetch one kernel section of the held job for its macro, while the FIFO holds no other
// macro's sections. False when that kernel is all fetched.
bool Dimc_HWPE::held_kernel_step(Dimc_InnerBlock &blk, uint32_t blk_id, int ctx)
{
    if (blk.macros.empty()) return false;
    const uint32_t m = this->job_macro(this->ctx_job_id[ctx]);
    const Dimc_Macro &t = blk.macros[m];
    if (t.kpf_job == this->ctx_job_id[ctx] && t.kfetched >= this->job_geom[ctx].kb_sections()) return false;
    for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo)
        if (q.macro != m) return true;          // the previous macro's kernel drains first
    if (blk.wgt_fifo.size() >= DIMC_WGT_FIFO_DEPTH) return true;
    this->fetch_kernel_ahead(blk, blk_id, m, ctx);
    return true;
}

// Fetch one section of macro `macro`'s program from L1 into the dual's FIFO.
void Dimc_HWPE::fill_beat(Dimc_InnerBlock &blk, uint32_t blk_id, uint32_t macro)
{
    Dimc_InnerBlock::FillCursor &cursor = blk.fill;
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t within = cursor.macro_beat_index[macro];
    Dimc_Macro &fm = blk.macros[macro];
    const BeatPos pos = this->beat_pos(this->job_geom[fm.fill_slot], within);

    if (pos.kind == DIMC_LOAD_KB) {
        if (fm.kpf_job != fm.fill_job) { fm.kpf_job = fm.fill_job; fm.kpf_slot = fm.fill_slot; fm.kfetched = 0; }
        this->fetch_kernel_section(blk, blk_id, macro, (int)fm.fill_slot, fm.fill_job, pos.sub);
    } else {
        // Vectors after the first read their own feature block.
        if (pos.run >= 1 && pos.sub == 0)
            this->configure_macro_streams(blk_id, macro, (int)fm.fill_slot, pos.run, false, true);
        Dimc_InnerBlock::FeedEntry e;
        e.macro = macro; e.slot = fm.fill_slot; e.job = fm.fill_job; e.within = within;
        const uint32_t off = pos.sub * port_bytes;
        e.bytes = DIMC_MACRO_FB_EW - off < port_bytes ? DIMC_MACRO_FB_EW - off : port_bytes;
        this->push_section(blk, blk_id, blk.input_stream[macro], e, blk.inp_fifo, blk.in_pending, DIMC_LOAD_FB);
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
    // One section per macro per cycle.
    if (mc.last_write_cycle == (int64_t)this->fsm_timestamp) return;
    mc.last_write_cycle = (int64_t)this->fsm_timestamp;
    // The macro's write program follows its sections: a new job's first one opens it.
    if (mc.write_job != e.job) {
        mc.write_job = e.job; mc.write_slot = e.slot;
        mc.written = 0; mc.kw = 0; mc.f0 = false; mc.stamped = false;
    }
    const uint32_t port_bytes = this->inner_port_bytes;
    bool feature_done = false;

    if (pos.kind == DIMC_LOAD_KB) {
        mc.kw++;
        const uint32_t row = pos.sub / g.kb_beats_per_row;
        const uint32_t sub = pos.sub % g.kb_beats_per_row;
        std::memcpy(mc.kb_row_buffer + sub * port_bytes, e.data, e.bytes);
        if (sub == g.kb_beats_per_row - 1)   // row complete -> commit
            mc.write_row((int)((g.row_base + row) % DIMC_MACRO_KB_LEN), mc.kb_row_buffer);
    } else {
        std::memcpy(mc.row_buffer + pos.sub * port_bytes, e.data, e.bytes);
        if (pos.sub == g.fb_beats_per_macro - 1) {   // feature complete
            mc.write_fb(mc.row_buffer);
            mc.kb_ready = mc.fb_ready = true;
            mc.addin = (int32_t)this->job_reg_ctx((int)e.slot, DIMC_HWPE_PSIN);
            mc.fb_run = pos.run;
            mc.fb_ready_cycle = this->fsm_timestamp + 1;
            feature_done = true;
            mc.f0 = mc.f0 || pos.run == 0;
        }
    }
    const uint32_t within = e.within, macro = e.macro;
    fifo.pop_front();
    const bool macro_filled = ++mc.written >= g.beats_per_macro;
    // Vector 0's feature and the kernel rows are in: hand the macro to the compute side, with
    // the shape of the job it now holds.
    if (!mc.stamped && mc.f0 && mc.kw >= g.kb_sections()) {
        mc.stamped = true;
        mc.fill_done_cycle = this->fsm_timestamp + 1;
        mc.filled_job   = mc.write_job;
        mc.job_nb_vec   = g.nb_vec;
        mc.job_rows     = g.row_count;
        mc.job_row_base = g.row_base;
        mc.ci           = this->ctx_ci[mc.write_slot];
        mc.sign_8b      = this->ctx_sign_8b[mc.write_slot];
        mc.compute_mask = this->ctx_compute_mask[mc.write_slot];
    }
    this->tracer.fill_beat(blk_id, macro, within, macro_filled, feature_done, 1);
}

// Rows the dual's other macro still triggers, plus the results in its pipe and in the out_fifo,
// before macro m may start `job`. 0 once m has started it; 0xFFFFFFFF while the other macro
// has not started the job before.
uint32_t Dimc_HWPE::rows_before_turn(const Dimc_InnerBlock &blk, uint32_t m, uint32_t job) const
{
    const Dimc_Macro &mc = blk.macros[m];
    if (mc.issue_job == job && (mc.rows_issued || mc.runs_issued)) return 0;
    uint32_t rows = (uint32_t)blk.out_fifo.size();
    for (uint32_t s = 0; s < blk.macros.size(); s++) {
        if (s == m) continue;
        const Dimc_Macro &sc = blk.macros[s];
        // issue_job trails filled_job only once all its rows are triggered.
        const bool issuing  = sc.issue_job != Dimc_Macro::JOB_NONE;
        const bool finished = issuing && (sc.issue_job != sc.filled_job || sc.runs_issued >= sc.job_nb_vec);
        const bool mid      = issuing && !finished && (sc.rows_issued || sc.runs_issued);
        // The last job the macro has fully triggered, and the jobs before `job` it has not.
        uint32_t done = this->running_job - 1u;
        if (issuing) {
            const uint32_t d = sc.issue_job - (finished ? 0u : 1u);
            if ((int32_t)(d - done) > 0) done = d;
        }
        const int32_t owed = (int32_t)(job - 1u - done);
        if (mid) {
            if (owed > 1) return 0xFFFFFFFFu;
            const uint32_t cur = sc.rows_issued < sc.job_rows ? sc.rows_issued : 0;
            rows += (sc.job_nb_vec - sc.runs_issued) * sc.job_rows - cur;
        } else if (owed > 0) {
            return 0xFFFFFFFFu;
        }
        rows += (uint32_t)sc.pipe.size();
    }
    return rows;
}

// One result leaves the out_fifo into its accumulator; rows the macros have finished enter the
// out_fifo; then the macro whose turn it is triggers its next row.
void Dimc_HWPE::compute_indep(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    // Popped before this cycle's pushes: the FIFO is not fall-through.
    if (!blk.out_fifo.empty()) {
        const Dimc_InnerBlock::OutEntry &e = blk.out_fifo.front();
        blk.acc[e.run * DIMC_MACRO_KB_LEN + e.row] += e.psout;
        blk.out_fifo.pop_front();
    }
    // A macro is never stalled for the out_fifo: a result it cannot take is lost, as in the
    // RTL. The model also warns; force_warning exits under werror.
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        if (!blk.macros[m].has_ready()) continue;
        DimcPipeEntry e = blk.macros[m].drain();
        blk.macros[m].rows_retired_set[e.set]++;
        if (blk.out_fifo.size() >= DIMC_OUT_FIFO_DEPTH) {
            if (++this->out_dropped <= 8)
                this->trace.force_warning("DIMC result lost: macro %u row %d (out_fifo full)\n",
                                          m, e.job_row);
            continue;
        }
        blk.out_fifo.push_back({e.psout, (uint16_t)e.job_row, (uint16_t)e.run});
        blk.out_accum.push(e.psout);   // summed as it enters the out_fifo
    }

    // A macro holding the next job's operands starts it when its turn comes, before the
    // running job closes; not after a TILE_LAST job, whose accumulators are written out first.
    const uint32_t J = this->running_job;
    const bool presenting = this->job_geom[this->exec_slot].tile_last;
    for (uint32_t m = 0; m < (uint32_t)blk.macros.size(); m++) {
        Dimc_Macro &mac = blk.macros[m];
        const bool ahead = mac.filled_job == J + 1;
        if (mac.filled_job != Dimc_Macro::JOB_NONE && (mac.filled_job == J || (ahead && !presenting))) {
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
            if (b < mac.job_nb_vec && mac.fb_run == b
                && !this->rows_before_turn(blk, m, jj)
                && this->fsm_timestamp >= mac.fb_ready_cycle
                && this->fsm_timestamp >= mac.fill_done_cycle && mac.can_accept()) {
                if (mac.rows_issued >= mac.job_rows) mac.rows_issued = 0;   // next run
                const uint32_t row_idx = (mac.job_row_base + mac.rows_issued) % DIMC_MACRO_KB_LEN;
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
        mac.tick();
    }
}

// The running job's results are accumulated on every block and, after a TILE_LAST job, written.
bool Dimc_HWPE::store_done() const
{
    for (const Dimc_InnerBlock &blk : this->inner_blocks)
        if (!block_stored(blk)) return false;
    return true;
}

// After a TILE_LAST job's last result is accumulated: one port word of the accumulators per
// dual per cycle, to the job's destination.
void Dimc_HWPE::store_block(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const JobGeom &g = this->job_geom[this->exec_slot];
    if (!g.tile_last || blk.out_written >= g.row_count) return;
    if (!this->tile_computed(blk)) return;
    this->store_word(blk, blk_id);
}

// Write the next row of the dual's accumulators, one result per vector in one port word. 0: the
// outstanding depth or the output port refused it; 1: written. The accumulators clear with
// the last row.
int Dimc_HWPE::store_word(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const uint32_t ctx = this->exec_slot;
    const JobGeom &g   = this->job_geom[ctx];
    uint32_t &row      = blk.out_written;
    if (const uint8_t why = this->refusal(blk.store_pending, blk.output_port)) {
        this->tracer.store_skip(blk_id, why);
        return 0;
    }
    this->configure_out_stream(blk_id, row, ctx);
    uint8_t word[DIMC_ACC_VECTORS * DIMC_OUT_SLOT_BYTES];
    for (uint32_t v = 0; v < g.nb_vec; v++)
        std::memcpy(word + v * DIMC_OUT_SLOT_BYTES, &blk.acc[v * DIMC_MACRO_KB_LEN + row], DIMC_OUT_SLOT_BYTES);
    int lat = blk.out_stream[0].issue_beat((int)(g.nb_vec * DIMC_OUT_SLOT_BYTES), word);
    this->tracer.store_beat(blk_id, this->job_macro(this->running_job), lat, g.out_beats);
    if (lat < 1) lat = 1;
    const uint64_t due = this->fsm_timestamp + (uint64_t)lat;
    blk.port_pending.push(due);
    blk.store_pending.push(due);
    blk.output_port.request((int64_t)this->fsm_timestamp, DIMC_OUTPUT_PORT_BYTES);
    this->tracer.outer_port_booked(blk_id, DIMC_PORT_WB);
    row++;
    blk.store.beat_index++;
    if (row >= g.row_count)
        for (int32_t &a : blk.acc) a = 0;
    return 1;
}
