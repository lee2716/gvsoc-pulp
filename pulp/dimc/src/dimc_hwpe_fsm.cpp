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

    for (Dimc_InnerBlock &blk : _this->inner_blocks)
        for (uint32_t m = 0; m < _this->num_macros; m++) {
            blk.macros[m].exec_ready     = false;
            // Cleared here, not left over from the previous job: they gate
            // compute, and this job's feature vector has not landed yet.
            blk.macros[m].kb_ready       = false;
            blk.macros[m].fb_ready       = false;
        }

    for (Dimc_InnerBlock &blk : _this->inner_blocks) blk.reset_job_state();

    _this->phase_entry_ts  = _this->fsm_timestamp;
    _this->job_entry_cycle = (uint64_t)_this->clock.get_cycles();
    if (_this->jobs_measured != 0)
        _this->acc_gap_cycles += _this->job_entry_cycle - _this->last_job_end;
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
        "DIMC job done, STATUS=1, finished_jobs=%u\n", _this->finished_jobs);
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
// outer-port reservation.
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
    // last_kb_src is NOT stamped here: it records that a weight load actually
    // landed, and stamping it at planning time would let a fill that never ran
    // be mistaken for one that did.
}

void Dimc_HWPE::plan_fill(int ctx)
{

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
        Dimc_InnerBlock::Cursor &cur = blk.fill;
        cur.reset((uint32_t)blk.macros.size());
        cur.beat_total = num_active * g.beats_per_macro;
        for (uint32_t m = 0; m < num_active; m++) {
            cur.macro_beat_total[m] = g.beats_per_macro;
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

int Dimc_HWPE::fsm()
{
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
        uint64_t spent = this->fsm_timestamp - this->phase_entry_ts;
        switch (this->state.get()) {
        case DIMC_STARTING:  this->acc_starting  += spent; break;
        case DIMC_COMPUTING: this->acc_computing += spent; break;
        case DIMC_STORING:   this->acc_storing   += spent; break;
        default: break;
        }
        this->phase_entry_ts = this->fsm_timestamp;
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

void Dimc_HWPE::phase_end_reset()
{
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.fill.beat_total = 0;
        blk.fill.beat_index = 0;
        blk.rows_issued = 0;
        blk.phase_done  = false;
    }
}

void Dimc_HWPE::beat_issued(Dimc_InnerBlock &blk, Dimc_InnerBlock::Cursor &cursor,
                            int lat)
{
    if (lat < 1) lat = 1;
    blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
    cursor.beat_index++;
    blk.beat_event.event((uint8_t *)&cursor.beat_index);
}


// ================= STARTING =================
bool Dimc_HWPE::preload_iter(int *latency)
{
    *latency = 1;
    const uint32_t port_bytes = this->inner_port_bytes;

    // ---- first cycle of the phase: latch the job shape, plan every block ----
    if (!this->phase_planned) {
        const uint32_t slot = this->exec_slot;
        this->fill_slot = slot;
        this->latch_geom((int)slot);
        this->plan_fill((int)slot);
        JobGeom &g = this->job_geom[slot];
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            for (uint32_t m = 0; m < g.num_active; m++) {
                blk.macros[m].rows_issued  = 0;
                blk.macros[m].rows_retired = 0;
            }
            // The store is planned here, not at the start of DIMC_STORING: its
            // beats now issue from inside this phase, as soon as the rows they
            // carry retire.
            g.out_beats = (g.row_count * 4 + port_bytes - 1) / port_bytes;
            blk.store.reset((uint32_t)blk.macros.size());
            blk.store.beat_total = g.num_active * g.out_beats;
            blk.out_buf.assign(g.num_active,
                               std::vector<uint8_t>(g.row_count * 4, 0));
            blk.load_done.assign(g.num_active, 0);
        }
        this->phase_planned = true;
    }

    // ---- outer-port fills ----
    // Every block asks for its working set when the job starts. Whether the
    // fills overlap or serialize is decided by the port, not here: a shared
    // port hands out later completion times, independent ports do not.
    // Per-block buffer depth (single vs ping-pong L1) is not modelled: there is
    // one fill per block per job, so depth could only matter across triggers.
    // ---- one beat per block per cycle ----
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        this->preload_block(blk, b, blk.fill);  // one beat, to the first macro still owing
        this->compute_indep(blk);     // every macro whose own fill has landed
        this->store_block(blk, b);    // and ship whatever has already retired
        this->publish_activity(blk);
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        retire_due(blk.port_pending, this->fsm_timestamp);
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
void Dimc_HWPE::preload_block(Dimc_InnerBlock &blk, uint32_t blk_id,
                              Dimc_InnerBlock::Cursor &cursor)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    std::queue<uint64_t> &blk_pending = blk.port_pending;

    if (cursor.beat_index >= cursor.beat_total ||
        blk_pending.size() >= this->outstanding_depth) {
        if (cursor.beat_index < cursor.beat_total) this->acc_stall_full++;
        return;
    }

    // Lowest-index macro that still owes beats. Same order the block-wide
    // cursor produced, so this substitution changes nothing by itself.
    uint32_t macro = 0;
    while (macro < this->job_geom[this->fill_slot].num_active &&
           cursor.macro_beat_index[macro] >= cursor.macro_beat_total[macro]) macro++;
    if (macro >= this->job_geom[this->fill_slot].num_active) return;
    uint32_t within = cursor.macro_beat_index[macro];
    if (within == 0)
        blk.macros[macro].trace_fill_start =
            (uint32_t)(this->fsm_timestamp - this->job_start_cycle);
    const JobGeom &fg = this->job_geom[this->fill_slot];
    // The feature and the partial sums go first, then the kernel rows. The
    // macro needs the feature vector before it can compute anything, so
    // sending it last would keep every row waiting for the whole fill; the RTL
    // drives FD/FA and D/WA on independent ports and imposes no order between
    // them.
    const uint32_t fb_span   = fg.fb_beats_per_macro;
    const uint32_t ps_span   = fb_span + fg.psin_beats_per_macro;
    const uint32_t kb_span   = fg.skip_kb ? 0 : fg.row_count * fg.kb_beats_per_row;
    int lat;

    if (within >= ps_span) {                      // ---- kernel beat ----
        uint32_t idx  = within - ps_span;
        uint32_t row  = idx / fg.kb_beats_per_row;
        uint32_t sub  = idx % fg.kb_beats_per_row;
        uint32_t off  = sub * port_bytes;
        uint32_t w    = DIMC_MACRO_KB_EW - off;
        if (w > port_bytes) w = port_bytes;
        lat = blk.weight_stream[macro].issue_beat((int)w, blk.macros[macro].row_buffer + off);
        if (sub == fg.kb_beats_per_row - 1) {  // row complete -> commit
            uint32_t row_idx = (fg.row_base + row) % DIMC_MACRO_KB_LEN;
            blk.macros[macro].write_row((int)row_idx, blk.macros[macro].row_buffer);
        }
    } else if (within < fb_span) {                // ---- feature beat ----
        uint32_t sub = within;
        uint32_t off = sub * port_bytes;
        uint32_t w   = DIMC_MACRO_FB_EW - off;
        if (w > port_bytes) w = port_bytes;
        lat = blk.input_stream[macro].issue_beat((int)w, blk.macros[macro].row_buffer + off);
        if (sub == fb_span - 1) {                 // feature complete
            blk.macros[macro].write_fb(blk.macros[macro].row_buffer);
            blk.macros[macro].kb_ready = true;
            blk.macros[macro].fb_ready = true;
            blk.macros[macro].pipe.clear();
            blk.macros[macro].psin_scalar = (int32_t)this->job_reg(DIMC_HWPE_PSIN);
            // Job-relative, like the makespan it is traced beside.
            blk.load_done[macro] =
                (uint32_t)(this->fsm_timestamp + 1 - this->job_start_cycle);
        }
    } else {                                      // ---- partial-sum beat ----
        uint32_t sub  = within - fb_span;
        uint32_t off  = sub * port_bytes;
        uint32_t left = fg.row_count * 4 - off;
        uint32_t w    = left > port_bytes ? port_bytes : left;
        lat = blk.psin_stream[macro].issue_beat((int)w, blk.macros[macro].row_buffer + off);
        // A beat carries several rows; commit them once the last one lands.
        if (sub == fg.psin_beats_per_macro - 1) {
            for (uint32_t r = 0; r < fg.row_count; r++) {
                uint32_t row_idx = (fg.row_base + r) % DIMC_MACRO_KB_LEN;
                blk.macros[macro].write_psin_row((int)row_idx, blk.macros[macro].row_buffer + r * 4);
            }
        }
    }

    cursor.macro_beat_index[macro]++;
    if (cursor.macro_beat_index[macro] >= cursor.macro_beat_total[macro]) {
        blk.macros[macro].fill_done_cycle = this->fsm_timestamp + (uint64_t)lat;
        blk.macros[macro].trace_fill_done =
            (uint32_t)(this->fsm_timestamp + (uint64_t)lat - this->job_start_cycle);
        // Hand the freshly filled buffers to the compute side.
        Dimc_Macro &mc = blk.macros[macro];
        mc.exec_ready = true;
        mc.last_kb_src = this->job_geom[this->fill_slot].kb_src;   // the fill landed
    }
    this->acc_beats++;
    this->acc_beat_lat += (uint64_t)(lat < 1 ? 1 : lat);
    blk.loaded_this_cycle = true;
    this->beat_issued(blk, cursor, lat);
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
            blk.computed_this_cycle = true;
        }
        mac.tick();
    }
}

void Dimc_HWPE::publish_activity(Dimc_InnerBlock &blk)
{
    uint8_t ld = blk.loaded_this_cycle ? 1 : 0;
    uint8_t cp = blk.computed_this_cycle ? 1 : 0;
    blk.load_active_event.event(&ld);
    blk.comp_active_event.event(&cp);
    blk.loaded_this_cycle = blk.computed_this_cycle = false;
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

// ================= STORING =================
bool Dimc_HWPE::store_iter(int *latency)
{
    *latency = 1;
    const uint32_t num_active = this->job_geom[this->exec_slot].num_active;
    const uint32_t row_count  = this->job_geom[this->exec_slot].row_count;
    const uint32_t out_bytes  = row_count * 4;

    if (!this->phase_planned) {
        this->phase_planned = true;
    }

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        // The macro pipeline does not stop because the job moved on to
        // storing: rows issued near the end of the fill are still in flight and
        // have to retire before their beats can go out. compute_indep issues
        // nothing here -- every row is already issued -- it ticks and drains.
        this->compute_indep(blk);
        if (!blk.phase_done) this->store_block(blk, b);
        this->publish_activity(blk);
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        size_t after = retire_due(blk.port_pending, this->fsm_timestamp);
        bool beats_done = (blk.store.beat_index >= blk.store.beat_total) && (after == 0);
        if (beats_done) blk.phase_done = true;
        else            all_done = false;
    }

    if (!all_done) return false;

    // ---- job closed: the elapsed cycles ARE the makespan, for the whole
    // outer block. No formula is applied on top: every beat was charged as it
    // issued, against the latency the L1 bank and the crossbar returned.
    this->phase_end_reset();
    this->phase_planned = false;

    uint32_t compute_cyc = this->job_geom[this->exec_slot].compute_cyc;
    bool     skip_kb     = this->job_geom[this->exec_slot].skip_kb;
    uint64_t finish      = this->fsm_timestamp - this->job_start_cycle;

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        for (uint32_t m = 0; m < num_active; m++) {
            this->trace.msg(vp::TraceLevel::WARNING,
                "macro[%u]: fill=[%u..%u] comp=[%u..%u] beats=%u skip_kb=%d "
                "load_done=%u compute=%u OUT=%u finish=%lu\n",
                b * num_active + m,
                blk.macros[m].trace_fill_start, blk.macros[m].trace_fill_done,
                blk.macros[m].trace_compute_start, blk.macros[m].trace_compute_end,
                blk.fill.macro_beat_total[m], (int)skip_kb,
                blk.load_done[m], compute_cyc, blk.out_beat_lat_est, finish);
        }
        if (blk.out_accum.enable) {
            this->trace.msg(vp::TraceLevel::WARNING,
                "block[%u] out_accum: acc=%d n=%u\n",
                b, blk.out_accum.acc, blk.out_accum.count);
        }
    }

    // Each phase advanced the clock one cycle per beat, so job start to here
    // already took the job's whole duration. The completion event therefore
    // fires after 1 cycle, not after that duration again, which would count it
    // twice.
    *latency = 1;
    const uint64_t now      = (uint64_t)this->clock.get_cycles();
    const uint64_t job_busy = now - this->job_entry_cycle;
    // Analytic minimum: every beat the fill must move through the inner port,
    // one per cycle, plus the macro pipeline's drain. The blocks run in
    // parallel, so one block's beats set the floor.
    const uint64_t ideal = (uint64_t)this->job_geom[this->exec_slot].num_active
                         * this->job_geom[this->exec_slot].beats_per_macro
                         + DIMC_MACRO_LATENCY;
    this->acc_busy_cycles  += job_busy;
    this->acc_ideal_cycles += ideal;
    this->last_job_end      = now;
    this->jobs_measured++;
    this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC double-buffer: num_active=%u row_count=%u l1bw=%u "
        "reuse=%u | %lu ---> %lu cyc | period = %lu cyc | ideal = %lu | "
        "uti = %.3f | totals: jobs=%u busy=%lu gap=%lu uti=%.3f\n",
        num_active, row_count, this->inner_port_bytes,
        (unsigned)skip_kb,
        (unsigned long)this->job_entry_cycle, (unsigned long)now,
        (unsigned long)job_busy, (unsigned long)ideal,
        job_busy ? (1.0 * ideal) / (1.0 * job_busy) : 0.0,
        this->jobs_measured,
        (unsigned long)this->acc_busy_cycles,
        (unsigned long)this->acc_gap_cycles,
        this->acc_busy_cycles
            ? (1.0 * this->acc_ideal_cycles) / (1.0 * this->acc_busy_cycles) : 0.0);
    this->trace.msg(vp::TraceLevel::WARNING,
        "  beats=%lu avg_lat=%.2f stalled_on_depth=%lu (depth=%u)\n",
        (unsigned long)this->acc_beats,
        this->acc_beats ? (1.0 * this->acc_beat_lat) / (1.0 * this->acc_beats) : 0.0,
        (unsigned long)this->acc_stall_full, this->outstanding_depth);
    return true;
}

void Dimc_HWPE::store_block(Dimc_InnerBlock &blk, uint32_t blk_id)
{
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t row_count  = this->job_geom[this->exec_slot].row_count;
    const uint32_t out_bytes  = row_count * 4;
    const uint32_t out_beats  = this->job_geom[this->exec_slot].out_beats;

    if (blk.store.beat_index >= blk.store.beat_total ||
        blk.port_pending.size() >= this->outstanding_depth) return;

    uint32_t macro = blk.store.beat_index / out_beats;
    uint32_t sub   = blk.store.beat_index % out_beats;
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

    int lat = blk.out_stream[macro].issue_beat((int)w, blk.out_buf[macro].data() + off);
    blk.out_beat_lat_est = (uint32_t)((lat < 1 ? 1 : lat) * (int)out_beats);

    if (lat < 1) lat = 1;
    blk.port_pending.push(this->fsm_timestamp + (uint64_t)lat);
    blk.store.beat_index++;
}
