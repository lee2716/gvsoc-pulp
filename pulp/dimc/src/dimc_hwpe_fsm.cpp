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
    uint32_t blk_num_active = _this->register_file[DIMC_HWPE_NUM_MACROS >> 2];
    if (blk_num_active == 0 || blk_num_active > _this->num_macros)
        blk_num_active = _this->num_macros;
    uint32_t blk_row_count = _this->job_reg(DIMC_HWPE_ROW_COUNT);
    if (blk_row_count == 0) blk_row_count = 1;
    if (blk_row_count > DIMC_MACRO_KB_LEN) blk_row_count = DIMC_MACRO_KB_LEN;

    const uint32_t kb_span  = blk_num_active * blk_row_count * DIMC_MACRO_KB_EW;
    const uint32_t fb_span  = blk_num_active * DIMC_MACRO_FB_EW;
    const uint32_t out_span = blk_num_active * blk_row_count * 4;

    uint32_t blk_id = 0;
    for (Dimc_InnerBlock &blk : _this->inner_blocks) {
    blk.weight_stream.configure(
        _this->job_reg(DIMC_HWPE_JOB_KB_SRC_ADDR) + blk_id * kb_span,  // base_addr
        _this->job_reg(DIMC_HWPE_KB_TOTAL_LENGTH),   // tot_len
        _this->job_reg(DIMC_HWPE_KB_D0_LENGTH),   // d0_len
        _this->job_reg(DIMC_HWPE_KB_D0_STRIDE),   // d0_stride
        _this->job_reg(DIMC_HWPE_KB_D1_LENGTH),   // d1_len
        _this->job_reg(DIMC_HWPE_KB_D1_STRIDE),   // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the feature (FB) input streamer
    blk.input_stream.configure(
        _this->job_reg(DIMC_HWPE_JOB_FB_SRC_ADDR) + blk_id * fb_span,  // base_addr
        _this->job_reg(DIMC_HWPE_FB_TOTAL_LENGTH),   // tot_len
        _this->job_reg(DIMC_HWPE_FB_D0_LENGTH),   // d0_len
        _this->job_reg(DIMC_HWPE_FB_D0_STRIDE),   // d0_stride
        0,                                                      // d1_len
        0,                                                      // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the output streamer
    blk.out_stream.configure(
        _this->job_reg(DIMC_HWPE_JOB_DST_ADDR) + blk_id * out_span,  // base_addr
        _this->job_reg(DIMC_HWPE_OUT_TOTAL_LENGTH),  // tot_len
        _this->job_reg(DIMC_HWPE_OUT_D0_LENGTH),  // d0_len
        _this->job_reg(DIMC_HWPE_OUT_D0_STRIDE),  // d0_stride
        0,                                                      // d1_len
        0,                                                      // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );
    blk_id++;
    }

    // Latch the per-job compute configuration once at commit, then broadcast it
    // to every macro (precision, sign mode, thermometric mask, dimc select).
    uint8_t compe     = (uint8_t)(_this->register_file[DIMC_HWPE_COMPE     >> 2] & 0x1);
    uint8_t ci        = (uint8_t)(_this->register_file[DIMC_HWPE_CFG_CI    >> 2] & 0x3);
    uint8_t sign_mode = (uint8_t)(_this->register_file[DIMC_HWPE_SIGN_MODE >> 2] & 0x3);
    uint8_t mct       = (uint8_t)(_this->register_file[DIMC_HWPE_MCT       >> 2] & 0xFF);
    uint8_t accum_en  = (uint8_t)(_this->job_reg(DIMC_HWPE_ACCUM_EN) & 0x1);
    _this->sel_dimc   = (uint8_t)(_this->register_file[DIMC_HWPE_SEL_DIMC  >> 2] & 0xFF);
    for (Dimc_InnerBlock &blk : _this->inner_blocks)
    for (auto &m : blk.macros) {
        // `compe` (memory-vs-compute mode) is latched but never read: compute_PP
        // always performs the dot product, and COMPE=0 memory mode is not
        // implemented.
        m.compe = compe; m.ci = ci; m.sign_mode = sign_mode; m.mct = mct;
        m.accumulate = accum_en;
    }
    // `sel_dimc` is stored but never consulted: macro selection goes through
    // NUM_MACROS (num_active). It exists for register-map fidelity.

    // job_running / running_job were latched by start_next_job() at commit.
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x0;   // busy

    // Reset the cycle-accurate engine for THIS job: the timestamp measures
    // cycles inside one job, so it must start from zero every time (otherwise
    // the reported makespan keeps accumulating across jobs).
    _this->fsm_timestamp = 0;
    _this->phase_planned = false;
    for (Dimc_InnerBlock &blk : _this->inner_blocks) blk.reset_job_state();
    // The outer port's busy-until stamp is job-relative, like fsm_timestamp.
    for (Dimc_OuterPort &p : _this->outer_ports) p.reset();

    _this->state.set(DIMC_STARTING);
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
    // Every phase returns latency=1, so this runs fsm() once per call. The job
    // advances one cycle per fsm_event and the makespan accrues in
    // fsm_timestamp, not here.
    uint32_t latency = 0;

    do {
        latency = this->fsm();
    } while (latency == 0 && state.get() != DIMC_FINISHED);

    // On completion fsm() returns 1, not the makespan: those cycles were already
    // spent stepping. Returning the makespan here would count them twice.
    if (state.get() == DIMC_FINISHED && !this->fsm_end_event->is_enqueued()) {
        this->event_enqueue(this->fsm_end_event, latency);
    } else if (!this->fsm_event->is_enqueued()) {
        this->event_enqueue(this->fsm_event, latency);
    }
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

static inline size_t retire_due(std::queue<uint64_t> &q, uint64_t now)
{
    while (!q.empty() && q.front() <= now) q.pop();
    return q.size();
}

// ================= Outer port =================
void Dimc_OuterPort::configure(uint32_t bandwidth, uint32_t latency)
{
    this->bandwidth_bytes = bandwidth ? bandwidth : 1;
    this->burst_latency   = latency;
    this->next_free_cycle = 0;
}

void Dimc_OuterPort::reset()
{
    this->next_free_cycle = 0;
}

// Reserve the port for `bytes` no earlier than `now`; return the cycle the data
// has landed. A client arriving while the port is still busy is pushed out by
// however long the previous burst still runs. That is the whole contention
// mechanism, the same one interco/router uses.
int64_t Dimc_OuterPort::request(int64_t now, uint64_t bytes)
{
    int64_t burst = (int64_t)((bytes + this->bandwidth_bytes - 1) / this->bandwidth_bytes);
    int64_t start = std::max(now, this->next_free_cycle);
    this->next_free_cycle = start + burst;
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
    uint64_t per_macro = this->job_skip_kb
        ? (uint64_t)DIMC_MACRO_FB_EW
        : ((uint64_t)this->job_row_count * DIMC_MACRO_KB_EW + DIMC_MACRO_FB_EW);
    return (uint64_t)this->job_num_active * per_macro;
}

// ================= STARTING =================
bool Dimc_HWPE::preload_iter(int *latency)
{
    *latency = 1;
    const uint32_t port_bytes = this->inner_port_bytes;

    // ---- first cycle of the phase: latch the job shape, plan every block ----
    if (!this->phase_planned) {
        uint32_t num_active = this->register_file[DIMC_HWPE_NUM_MACROS >> 2];
        if (num_active == 0 || num_active > this->num_macros) num_active = this->num_macros;
        uint32_t row_count = this->job_reg(DIMC_HWPE_ROW_COUNT);
        if (row_count == 0) row_count = 1;
        if (row_count > DIMC_MACRO_KB_LEN) row_count = DIMC_MACRO_KB_LEN;

        uint32_t kb_src  = this->job_reg(DIMC_HWPE_JOB_KB_SRC_ADDR);
        bool     skip_kb = (kb_src == this->last_kb_src);
        this->last_kb_src = kb_src;

        this->job_num_active  = num_active;
        this->job_row_count   = row_count;
        this->job_row_base    = this->job_reg(DIMC_HWPE_ROW_SEL_BASE);
        this->job_bias        = (int32_t)this->job_reg(DIMC_HWPE_CFG_BIAS);
        this->job_compute_cyc = row_count + DIMC_MACRO_LATENCY;
        this->job_skip_kb     = skip_kb;

        this->kb_beats_per_row   = (DIMC_MACRO_KB_EW + port_bytes - 1) / port_bytes;
        this->fb_beats_per_macro = (DIMC_MACRO_FB_EW + port_bytes - 1) / port_bytes;
        uint32_t kb_rows = skip_kb ? 0 : row_count;
        this->beats_per_macro = kb_rows * this->kb_beats_per_row + this->fb_beats_per_macro;

        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            blk.beat_total      = num_active * this->beats_per_macro;
            blk.beat_index      = 0;
            blk.phase_done      = false;
            blk.fill_requested  = false;
            blk.data_ready_cycle = 0;
            blk.out_buf.assign(num_active, std::vector<uint8_t>(row_count * 4, 0));
            blk.load_done.assign(num_active, 0);
            blk.out_lat         = 0;
        }
        this->fsm_timestamp += this->inner_noc_lat;   // burst head, paid once
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
            blk.data_ready_cycle =
                port->request((int64_t)this->fsm_timestamp, this->block_working_set());
        }
        blk.fill_requested = true;
    }

    // ---- one beat per block per cycle ----
    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        if (!blk.phase_done) this->preload_block(blk);
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        size_t after = retire_due(blk.pending_req_queue, this->fsm_timestamp);
        if (blk.beat_index >= blk.beat_total && after == 0) blk.phase_done = true;
        else                                                all_done = false;
    }

    if (!all_done) return false;

    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.beat_total = 0;
        blk.beat_index = 0;
        blk.rows_issued = 0;
        blk.phase_done = false;
    }
    this->phase_planned = false;
    return true;
}

// Issue at most one beat of one inner block. The linear beat_index is the only
// position state; (macro, row, offset) are derived, so they cannot drift apart.
bool Dimc_HWPE::preload_block(Dimc_InnerBlock &blk)
{
    const uint32_t port_bytes = this->inner_port_bytes;

    // The block's data has not landed from the outer port yet.
    if ((int64_t)this->fsm_timestamp < blk.data_ready_cycle) return false;

    if (blk.beat_index >= blk.beat_total ||
        blk.pending_req_queue.size() >= this->outstanding_depth) return false;

    uint32_t macro  = blk.beat_index / this->beats_per_macro;
    uint32_t within = blk.beat_index % this->beats_per_macro;
    uint32_t kb_span = this->job_skip_kb ? 0
                     : this->job_row_count * this->kb_beats_per_row;
    int lat;

    if (within < kb_span) {                       // ---- kernel beat ----
        uint32_t row  = within / this->kb_beats_per_row;
        uint32_t sub  = within % this->kb_beats_per_row;
        uint32_t off  = sub * port_bytes;
        uint32_t w    = DIMC_MACRO_KB_EW - off;
        if (w > port_bytes) w = port_bytes;
        lat = blk.weight_stream.issue_beat((int)w, blk.row_buffer + off);
        if (sub == this->kb_beats_per_row - 1) {  // row complete -> commit
            uint32_t row_idx = (this->job_row_base + row) % DIMC_MACRO_KB_LEN;
            blk.macros[macro].write_row((int)row_idx, blk.row_buffer);
        }
    } else {                                      // ---- feature beat ----
        uint32_t sub = within - kb_span;
        uint32_t off = sub * port_bytes;
        uint32_t w   = DIMC_MACRO_FB_EW - off;
        if (w > port_bytes) w = port_bytes;
        lat = blk.input_stream.issue_beat((int)w, blk.row_buffer + off);
        if (sub == this->fb_beats_per_macro - 1) { // feature complete
            blk.macros[macro].write_fb(blk.row_buffer);
            blk.macros[macro].kb_ready = true;
            blk.macros[macro].fb_ready = true;
            blk.macros[macro].pipe.clear();
            blk.macros[macro].psin = (int32_t)this->job_reg(DIMC_HWPE_PSIN);
            blk.load_done[macro] = (uint32_t)(this->fsm_timestamp + 1);
        }
    }

    if (lat < 1) lat = 1;
    blk.pending_req_queue.push(this->fsm_timestamp + (uint64_t)lat);
    blk.beat_index++;
    return false;
}

// ================= COMPUTING =================
bool Dimc_HWPE::compute_iter(int *latency)
{
    *latency = 1;

    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        if (!blk.phase_done && this->compute_block(blk)) blk.phase_done = true;
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks)
        if (!blk.phase_done) all_done = false;

    if (!all_done) return false;

    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.beat_total = 0;
        blk.beat_index = 0;
        blk.phase_done = false;
    }
    return true;
}

void Dimc_HWPE::drain_ready_rows(Dimc_InnerBlock &blk)
{
    const uint32_t out_w = 4;
    for (uint32_t m = 0; m < this->job_num_active; m++) {
        uint8_t *out_buf = blk.out_buf[m].data();
        while (blk.macros[m].has_ready()) {
            DimcPipeEntry e = blk.macros[m].drain();
            uint32_t psout_u = (uint32_t)e.psout;
            std::memcpy(out_buf + e.job_row * out_w, &psout_u, out_w);
        }
    }
}

// Advance one block's macro pipelines by a cycle. The macros inside a block run
// in lockstep, so a row enters all of them or none of them, and the row counter
// advances once, never per macro.
bool Dimc_HWPE::compute_block(Dimc_InnerBlock &blk)
{
    const uint32_t num_active = this->job_num_active;
    const uint32_t row_count  = this->job_row_count;

    this->drain_ready_rows(blk);

    if (blk.rows_issued < row_count) {
        bool all_ready = true;
        for (uint32_t m = 0; m < num_active; m++)
            if (!blk.macros[m].can_accept()) { all_ready = false; break; }
        if (all_ready) {
            uint32_t row_idx = (this->job_row_base + blk.rows_issued) % DIMC_MACRO_KB_LEN;
            for (uint32_t m = 0; m < num_active; m++)
                blk.macros[m].issue((int)row_idx, (int)blk.rows_issued, this->job_bias);
            blk.rows_issued++;
        }
    }
    for (uint32_t m = 0; m < num_active; m++) blk.macros[m].tick();

    bool pipes_empty = true;
    for (uint32_t m = 0; m < num_active; m++)
        if (!blk.macros[m].pipe.empty()) { pipes_empty = false; break; }

    if (blk.rows_issued >= row_count && pipes_empty) {
        this->drain_ready_rows(blk);
        return true;
    }
    return false;
}

// ================= STORING =================
bool Dimc_HWPE::store_iter(int *latency)
{
    *latency = 1;
    const uint32_t num_active = this->job_num_active;
    const uint32_t row_count  = this->job_row_count;
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t out_bytes  = row_count * 4;
    const uint32_t out_beats  = (out_bytes + port_bytes - 1) / port_bytes;

    if (!this->phase_planned) {
        for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
            Dimc_InnerBlock &blk = this->inner_blocks[b];
            blk.beat_total = num_active * out_beats;
            blk.beat_index = 0;
            blk.phase_done = false;
            // Results leave through the same outer port, so a shared port
            // serialises the drains too. The drain is requested at the start of
            // the store phase and runs in parallel with the inner store beats:
            // per the D-tile spec the hardware overlaps drain with loading, it
            // does not wait for the whole result to reach L1. Unlike the input
            // fill, store is not store-and-forward.
            Dimc_OuterPort *port = this->block_port(b);
            if (port != NULL) {
                uint64_t out_ws = (uint64_t)num_active * (uint64_t)out_bytes;
                blk.data_ready_cycle =
                    port->request((int64_t)this->fsm_timestamp, out_ws);
            } else {
                blk.data_ready_cycle = 0;
            }
        }
        this->fsm_timestamp += this->inner_noc_lat;   // drain burst head
        this->phase_planned = true;
    }

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        if (!blk.phase_done) this->store_block(blk);
    }

    this->fsm_timestamp++;

    bool all_done = true;
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        size_t after = retire_due(blk.pending_req_queue, this->fsm_timestamp);
        bool beats_done = (blk.beat_index >= blk.beat_total) && (after == 0);
        bool drain_landed = (int64_t)this->fsm_timestamp >= blk.data_ready_cycle;
        if (beats_done && drain_landed) blk.phase_done = true;
        else                            all_done = false;
    }

    if (!all_done) return false;

    // ---- job closed: the elapsed cycles ARE the makespan, for the whole
    // outer block. No formula is applied on top: the outer port already
    // charged its bandwidth and its serialization while the job ran.
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.beat_total = 0;
        blk.beat_index = 0;
        blk.phase_done = false;
    }
    this->phase_planned = false;

    uint64_t finish      = this->fsm_timestamp;
    uint32_t compute_cyc = this->job_compute_cyc;
    bool     skip_kb     = this->job_skip_kb;

    for (uint32_t b = 0; b < this->inner_blocks.size(); b++) {
        Dimc_InnerBlock &blk = this->inner_blocks[b];
        for (uint32_t m = 0; m < num_active; m++) {
            this->trace.msg(vp::TraceLevel::WARNING,
                "macro[%u]: load_done=%u compute=%u OUT=%u\n",
                b * num_active + m, blk.load_done[m], compute_cyc, blk.out_lat);
        }
    }

    // Each phase advanced the clock one cycle per beat, so job start to here
    // already took `finish` cycles. The completion event therefore fires after
    // 1 cycle, not after `finish` more, which would count them twice. `finish`
    // only goes to the trace.
    *latency = 1;
    this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC double-buffer: num_active=%u row_count=%u l1bw=%u "
        "reuse=%u total_latency=%lu\n",
        num_active, row_count, this->inner_port_bytes,
        (unsigned)skip_kb, (unsigned long)finish);
    return true;
}

bool Dimc_HWPE::store_block(Dimc_InnerBlock &blk)
{
    const uint32_t row_count  = this->job_row_count;
    const uint32_t port_bytes = this->inner_port_bytes;
    const uint32_t out_bytes  = row_count * 4;
    const uint32_t out_beats  = (out_bytes + port_bytes - 1) / port_bytes;

    if (blk.beat_index >= blk.beat_total ||
        blk.pending_req_queue.size() >= this->outstanding_depth) return false;

    uint32_t macro = blk.beat_index / out_beats;
    uint32_t sub   = blk.beat_index % out_beats;
    uint32_t off   = sub * port_bytes;
    uint32_t w     = out_bytes - off;
    if (w > port_bytes) w = port_bytes;
    int lat = blk.out_stream.issue_beat((int)w, blk.out_buf[macro].data() + off);
    if (lat < 1) lat = 1;
    blk.pending_req_queue.push(this->fsm_timestamp + (uint64_t)lat);
    blk.out_lat = (uint32_t)(lat * (int)out_beats);
    blk.beat_index++;
    return false;
}
