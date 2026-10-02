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
#include <string>

Dimc_Tracer::Dimc_Tracer(const Dimc_HWPE &dimc) : dimc(dimc)
{
}

void Dimc_Tracer::build(vp::Component &owner, vp::Trace &text,
                        uint32_t nb_blocks, uint32_t nb_macros)
{
    this->text = &text;
    this->blocks.resize(nb_blocks);
    for (Block &blk : this->blocks) {
        blk.macros.resize(nb_macros);
        blk.why.assign(nb_macros, 0);
    }

    // Names become <component path>.<leaf> in the dump, which is what the
    // --include filter of gvsoc2perfetto matches on; a "/" nests them.
    owner.traces.new_trace_event("state", &this->state_event, 8);
    owner.traces.new_trace_event("busy", &this->busy_event, 1);
    owner.traces.new_trace_event("job_id", &this->job_event, 32);
    owner.traces.new_trace_event("outer_port/next_free", &this->next_free_event, 32);
    owner.traces.new_trace_event("outer_port/use", &this->port_use_event, 8);
    for (uint32_t b = 0; b < nb_blocks; b++) {
        Block &blk = this->blocks[b];
        std::string pfx = "block_" + std::to_string(b) + "/";
        owner.traces.new_trace_event(pfx + "beat_index",  &blk.beat_event, 32);
        owner.traces.new_trace_event(pfx + "rows_issued", &blk.rows_event, 32);
        owner.traces.new_trace_event(pfx + "load_active", &blk.load_event, 1);
        owner.traces.new_trace_event(pfx + "fill_grant",  &blk.fill_grant_event, 8);
        owner.traces.new_trace_event(pfx + "comp_active", &blk.comp_event, 1);
        owner.traces.new_trace_event(pfx + "wb_active",   &blk.wb_event,   1);
        for (uint32_t m = 0; m < nb_macros; m++) {
            Macro &mac = blk.macros[m];
            std::string mpfx = pfx + "macro_" + std::to_string(m) + "/";
            owner.traces.new_trace_event(mpfx + "load_active", &mac.load_event, 1);
            owner.traces.new_trace_event(mpfx + "comp_active", &mac.comp_event, 1);
            owner.traces.new_trace_event(mpfx + "wb_active",   &mac.wb_event,   1);
            owner.traces.new_trace_event(mpfx + "why",         &mac.why_event,  8);
            owner.traces.new_trace_event(mpfx + "load_kind",   &mac.load_kind_event, 8);
        }
    }
    owner.traces.new_trace_event("commit_job_id", &this->commit_event, 32);
    owner.traces.new_trace_event("idle_why", &this->idle_why_event, 8);
    owner.traces.new_trace_event("outer_grant", &this->outer_grant_event, 8);
}

void Dimc_Tracer::reset()
{
    uint8_t st = DIMC_IDLE, zero8 = 0;
    uint32_t zero32 = 0;
    this->cur_state = DIMC_IDLE;
    this->state_event.event(&st);
    this->busy_event.event(&zero8);
    this->job_event.event((uint8_t *)&zero32);
    for (Block &blk : this->blocks) {
        blk.beat_event.event((uint8_t *)&zero32);
        blk.rows_event.event((uint8_t *)&zero32);
    }
}

void Dimc_Tracer::commit(uint32_t job_id)
{
    this->commit_event.event((uint8_t *)&job_id);
}

void Dimc_Tracer::job_queued()
{
    uint8_t why = DIMC_IDLE_WHY_START;
    this->idle_why_event.event(&why);
}

void Dimc_Tracer::job_start(uint64_t now_cycle)
{
    this->text->msg(vp::TraceLevel::WARNING, "DIMC job start\n");

    this->job_entry_cycle = now_cycle;
    if (this->jobs_measured != 0)
        this->acc_gap_cycles += now_cycle - this->last_job_end;
    this->cur_state = DIMC_STARTING;
    for (Block &blk : this->blocks) {
        blk.out_beat_lat_est = 0;
        for (Macro &mac : blk.macros) mac.load_done = 0;
    }

    uint8_t one = 1, st = DIMC_STARTING, why = DIMC_IDLE_WHY_BUSY;
    uint32_t job = this->dimc.running_job;
    this->busy_event.event(&one);
    this->idle_why_event.event(&why);
    this->job_event.event((uint8_t *)&job);
    this->state_event.event(&st);
}

void Dimc_Tracer::state(uint32_t next_state)
{
    this->cur_state = next_state;
    uint8_t st = (uint8_t)next_state;
    this->state_event.event(&st);
}

void Dimc_Tracer::fill_beat(uint32_t b, uint32_t macro, uint32_t within,
                            bool macro_filled, bool feature_done, int lat)
{
    Block &blk = this->blocks[b];
    Macro &mac = blk.macros[macro];
    const uint64_t ts = this->dimc.fsm_timestamp, start = this->dimc.job_start_cycle;
    if (within == 0)   mac.fill_start = (uint32_t)(ts - start);
    if (feature_done)  mac.load_done  = (uint32_t)(ts + 1 - start);
    if (macro_filled)  mac.fill_done  = (uint32_t)(ts + (uint64_t)lat - start);
    this->acc_beats++;
    this->acc_beat_lat += (uint64_t)(lat < 1 ? 1 : lat);
    blk.loaded = true;
    mac.loaded = 1;
    blk.fill_grant = (uint8_t)macro;   // a FIFO section was written into this macro
    // Beat kind from the engine's own beat_kind.
    {
        const uint32_t slot = this->dimc.inner_blocks[b].macros[macro].write_slot;
        const Dimc_HWPE::JobGeom &fg = this->dimc.job_geom[slot];
        const uint8_t k = this->dimc.beat_kind(fg, within), prev = mac.load_kind;
        const bool kb_psin = (prev == DIMC_LOAD_KB && k == DIMC_LOAD_PSIN) || (prev == DIMC_LOAD_PSIN && k == DIMC_LOAD_KB);
        mac.load_kind = kb_psin ? DIMC_LOAD_KB_PSIN : k;
    }
    uint32_t idx = this->dimc.inner_blocks[b].fill.beat_index;
    blk.beat_event.event((uint8_t *)&idx);
}

void Dimc_Tracer::fill_skip(uint32_t b, uint8_t why)
{
    if (why == DIMC_WHY_WAIT_DEPTH) this->acc_stall_full++;
    this->blocks[b].fill_skip = why;
}

void Dimc_Tracer::kernel_skip(uint32_t b, uint8_t why)
{
    this->blocks[b].kernel_skip = why;
    if (why == DIMC_WHY_WAIT_OUTER_PORT) this->blocks[b].kernel_port_refused = this->dimc.fsm_timestamp;
}

void Dimc_Tracer::input_skip(uint32_t b, uint8_t why)
{
    this->blocks[b].input_skip = why;
}

void Dimc_Tracer::row_issued(uint32_t b, uint32_t m, uint32_t rows_issued,
                             uint32_t row_count)
{
    Block &blk = this->blocks[b];
    Macro &mac = blk.macros[m];
    const uint64_t ts = this->dimc.fsm_timestamp, start = this->dimc.job_start_cycle;
    if (rows_issued == 1)         mac.compute_start = (uint32_t)(ts - start);
    if (rows_issued == row_count) mac.compute_end   = (uint32_t)(ts - start);
    if (m == 0) blk.rows_event.event((uint8_t *)&rows_issued);
    blk.computed = true;
    mac.computed = 1;
}

void Dimc_Tracer::store_beat(uint32_t b, uint32_t macro, int lat, uint32_t out_beats)
{
    Block &blk = this->blocks[b];
    blk.wrote_back = true;
    blk.macros[macro].wrote_back = 1;
    blk.out_beat_lat_est = (uint32_t)((lat < 1 ? 1 : lat) * (int)out_beats);
}

void Dimc_Tracer::store_skip(uint32_t b, uint8_t why)
{
    this->blocks[b].store_skip = why;
}

void Dimc_Tracer::outer_port_booked(uint32_t who, uint8_t kind)
{
    const bool is_store = kind == DIMC_PORT_WB;
    const uint32_t shift = is_store ? 6u : kind == DIMC_LOAD_FB ? 2u : kind == DIMC_LOAD_PSIN ? 4u : 0u;
    this->port_use = (uint8_t)(this->port_use + (1u << shift));
    // Stores are offset by 16 so one track shows both: 0,1 = block 0,1 filling;
    // 16,17 = block 0,1 writing back.
    this->outer_grant = (uint8_t)(who + (is_store ? 16u : 0u));
    // The two directions have their own budget; report the one this beat booked.
    const Dimc_OuterPort &port = is_store ? this->dimc.outer_port_out
                                          : this->dimc.outer_port_in;
    uint32_t nf = (uint32_t)((port.cursor_bytes + port.bandwidth_bytes - 1)
                             / port.bandwidth_bytes);
    this->next_free_event.event((uint8_t *)&nf);
}

void Dimc_Tracer::end_cycle(uint32_t b)
{
    Block &blk = this->blocks[b];
    uint8_t ld = blk.loaded ? 1 : 0;
    uint8_t cp = blk.computed ? 1 : 0;
    uint8_t wb = blk.wrote_back ? 1 : 0;
    blk.load_event.event(&ld);
    blk.comp_event.event(&cp);
    blk.wb_event.event(&wb);

    // Every cause before any flag is cleared: a macro's cause can depend on
    // what a lower macro did in the same cycle.
    for (uint32_t m = 0; m < blk.macros.size(); m++) blk.why[m] = this->macro_why(b, m);
    for (uint32_t m = 0; m < blk.macros.size(); m++) {
        Macro &mac = blk.macros[m];
        mac.load_event.event(&mac.loaded);
        mac.comp_event.event(&mac.computed);
        mac.wb_event.event(&mac.wrote_back);
        mac.why_event.event(&blk.why[m]);
        mac.load_kind_event.event(&mac.load_kind);
        mac.loaded = mac.computed = mac.wrote_back = 0;
        mac.load_kind = DIMC_LOAD_NONE;
    }
    blk.fill_grant_event.event(&blk.fill_grant);
    blk.loaded = blk.computed = blk.wrote_back = false;
    blk.fill_skip = blk.store_skip = blk.kernel_skip = blk.input_skip = 0;
    blk.fill_grant = DIMC_GRANT_NONE;
    // The outer port is shared: flush it with the last block, after every block has booked.
    if (b + 1 == this->blocks.size()) {
        this->outer_grant_event.event(&this->outer_grant);
        this->outer_grant = DIMC_GRANT_NONE;
    }
}

void Dimc_Tracer::port_cycle()
{
    this->port_use_event.event(&this->port_use);
    this->port_use = 0;
}

uint8_t Dimc_Tracer::macro_why(uint32_t b, uint32_t m) const
{
    const Block &tb = this->blocks[b];
    const Macro &tm = tb.macros[m];
    if (tm.loaded)     return tm.load_kind == DIMC_LOAD_KB || tm.load_kind == DIMC_LOAD_KB_PSIN ? DIMC_WHY_LOAD_KB
                            : tm.load_kind == DIMC_LOAD_FB   ? DIMC_WHY_LOAD_FB
                            : tm.load_kind == DIMC_LOAD_PSIN ? DIMC_WHY_LOAD_PSIN
                                                             : DIMC_WHY_UNKNOWN;
    if (tm.computed)   return DIMC_WHY_COMPUTE;
    if (tm.wrote_back) return DIMC_WHY_WRITE_BACK;
    // No job running: nothing to wait for.
    if (this->cur_state == DIMC_IDLE) return DIMC_WHY_IDLE;

    const Dimc_InnerBlock &blk = this->dimc.inner_blocks[b];
    const Dimc_HWPE::JobGeom &g = this->dimc.job_geom[this->dimc.exec_slot];
    if (m >= g.num_active) return DIMC_WHY_IDLE;
    const Dimc_Macro &mac = blk.macros[m];

    // Inside a kernel window (sections written, not all): the weight FIFO's head says why no
    // section went in. In flight: late because the kernel feed lost the outer port within the
    // fetch latency, else the response is still out. Another macro's section: the dual's FIFO
    // is in order. Landed: beat_writable refused it. Empty: this cycle's kernel-feed refusal.
    if (mac.write_job != Dimc_Macro::JOB_NONE && !mac.stamped) {
        const Dimc_HWPE::JobGeom &wg = this->dimc.job_geom[mac.write_slot];
        const uint32_t kb_all = wg.row_count * wg.kb_beats_per_row;
        if (mac.kw > 0 && mac.kw < kb_all) {
            const uint64_t now = this->dimc.fsm_timestamp;
            const bool port_late = tb.kernel_port_refused != ~0ull && now - tb.kernel_port_refused <= 3;
            if (blk.wgt_fifo.empty())
                return tb.kernel_skip ? tb.kernel_skip : port_late ? DIMC_WHY_WAIT_OUTER_PORT : DIMC_WHY_WAIT_FILL_ACK;
            const Dimc_InnerBlock::FeedEntry &h = blk.wgt_fifo.front();
            if (h.macro != m) return DIMC_WHY_WAIT_WGT_FIFO;
            if (h.ready > now) return port_late ? DIMC_WHY_WAIT_OUTER_PORT : DIMC_WHY_WAIT_FILL_ACK;
            return DIMC_WHY_WAIT_COMPUTE;
        }
    }
    // Kernel not started: its sections (fetched, or next in its program) are behind another
    // macro's in the dual's weight FIFO, which pops in order.
    if (this->cur_state == DIMC_STARTING && !blk.wgt_fifo.empty() && blk.wgt_fifo.front().macro != m) {
        bool own = false;
        for (const Dimc_InnerBlock::FeedEntry &q : blk.wgt_fifo) if (q.macro == m) { own = true; break; }
        const Dimc_InnerBlock::Cursor &f = blk.fill;
        if (!own && m < f.macro_beat_index.size() && f.macro_beat_index[m] < f.macro_beat_total[m])
            own = this->dimc.beat_pos(this->dimc.job_geom[mac.fill_slot], f.macro_beat_index[m]).kind == DIMC_LOAD_KB;
        if (own) return DIMC_WHY_WAIT_WGT_FIFO;
    }
    // Owes an input section (partial sums or feature): its first one in the input FIFO says
    // why it is not in. In flight: the response is out. Behind another macro's section: the
    // FIFO is in order. At the head: beat_writable refused it. Not fetched: this cycle's
    // input-feed refusal, else the heuristic below.
    if (this->cur_state == DIMC_STARTING) {
        const Dimc_InnerBlock::Cursor &f = blk.fill;
        if (m < f.macro_beat_index.size() && f.macro_beat_index[m] < f.macro_beat_total[m]
            && this->dimc.beat_pos(this->dimc.job_geom[mac.fill_slot], f.macro_beat_index[m]).kind != DIMC_LOAD_KB) {
            const std::deque<Dimc_InnerBlock::FeedEntry> &q = blk.inp_fifo[0];
            for (size_t i = 0; i < q.size(); i++) {
                if (q[i].macro != m) continue;
                if (q[i].ready > this->dimc.fsm_timestamp) return DIMC_WHY_WAIT_FILL_ACK;
                return i ? DIMC_WHY_WAIT_INNER_PORT : DIMC_WHY_WAIT_COMPUTE;
            }
            if (tb.input_skip) return tb.input_skip;
        }
    }
    // Still owes fill sections. A heuristic: fill_feed picks a macro per feed, not by
    // index, but a lower macro still owing is reported as holding the port.
    if (this->cur_state == DIMC_STARTING) {
        const Dimc_InnerBlock::Cursor &f = blk.fill;
        if (f.macro_beat_index[m] < f.macro_beat_total[m]) {
            // A lower macro that still owes sections, or was written one this cycle.
            for (uint32_t k = 0; k < m; k++)
                if (f.macro_beat_index[k] < f.macro_beat_total[k] || tb.macros[k].loaded)
                    return DIMC_WHY_WAIT_INNER_PORT;
            return tb.fill_skip ? tb.fill_skip : DIMC_WHY_UNKNOWN;
        }
        // Filled but issued no row: compute waits for the fill's response.
        if (mac.rows_issued < g.row_count)
            return this->dimc.fsm_timestamp < mac.fill_done_cycle ? DIMC_WHY_WAIT_FILL_ACK
                                                                   : DIMC_WHY_UNKNOWN;
    }

    // Every row issued. Rows are checked before the port: store_block tests the
    // port first, which would blame the port for a beat whose rows had not
    // retired anyway.
    const uint32_t first = m * g.out_beats, last = first + g.out_beats;
    const uint32_t cur   = blk.store.beat_index;
    if (cur < first)
        return mac.pipe.empty() ? DIMC_WHY_WAIT_STORE_ORDER : DIMC_WHY_PIPE_DRAIN;
    if (cur < last) {
        const uint32_t per = this->dimc.inner_port_bytes / DIMC_OUT_SLOT_BYTES;
        uint32_t need = (cur - first + 1) * per;   // rows retired through this beat, cumulative
        if (need > g.row_count) need = g.row_count;
        const uint32_t s = this->dimc.running_job & 1;
        if (mac.set_job[s] != this->dimc.running_job || mac.rows_retired_set[s] < need)
            return DIMC_WHY_PIPE_DRAIN;
        return tb.store_skip ? tb.store_skip : DIMC_WHY_UNKNOWN;
    }

    // All of this macro's beats are issued.
    for (const Dimc_InnerBlock &other : this->dimc.inner_blocks)
        if (other.store.beat_index < other.store.beat_total) return DIMC_WHY_DONE_WAIT_JOB;
    for (const Dimc_InnerBlock &other : this->dimc.inner_blocks)
        if (!other.port_pending.empty()) return DIMC_WHY_WAIT_BEAT_ACK;
    return DIMC_WHY_FSM_STEP;
}

void Dimc_Tracer::job_closed(uint64_t now_cycle)
{
    const Dimc_HWPE::JobGeom &g = this->dimc.job_geom[this->dimc.exec_slot];
    const uint32_t num_active  = g.num_active;
    const uint64_t finish      = this->dimc.fsm_timestamp - this->dimc.job_start_cycle;

    for (uint32_t b = 0; b < this->blocks.size(); b++) {
        const Dimc_InnerBlock &blk = this->dimc.inner_blocks[b];
        const Block &tb = this->blocks[b];
        for (uint32_t m = 0; m < num_active; m++) {
            const Macro &mac = tb.macros[m];
            this->text->msg(vp::TraceLevel::WARNING,
                "macro[%u]: fill=[%u..%u] comp=[%u..%u] beats=%u "
                "load_done=%u compute=%u OUT=%u finish=%lu\n",
                b * num_active + m,
                mac.fill_start, mac.fill_done, mac.compute_start, mac.compute_end,
                blk.fill.macro_beat_total[m],
                mac.load_done, g.compute_cyc, tb.out_beat_lat_est, finish);
        }
        if (blk.out_accum.enable) {
            this->text->msg(vp::TraceLevel::WARNING,
                "block[%u] out_accum: acc=%d n=%u\n",
                b, blk.out_accum.acc, blk.out_accum.count);
        }
    }

    const uint64_t job_busy = now_cycle - this->job_entry_cycle;
    // Analytic minimum: every beat the fill must move through the inner port,
    // one per cycle, plus the macro pipeline's drain. One block's beats set the
    // floor, which holds while the blocks' combined demand still fits the
    // outer port.
    const uint64_t ideal = (uint64_t)g.num_active * g.beats_per_macro + DIMC_MACRO_LATENCY;
    this->acc_busy_cycles  += job_busy;
    this->acc_ideal_cycles += ideal;
    this->last_job_end      = now_cycle;
    this->jobs_measured++;
    this->text->msg(vp::TraceLevel::WARNING,
        "DIMC double-buffer: num_active=%u row_count=%u l1bw=%u "
        "nb_vec=%u | %lu ---> %lu cyc | period = %lu cyc | ideal = %lu | "
        "uti = %.3f | totals: jobs=%u busy=%lu gap=%lu uti=%.3f\n",
        num_active, g.row_count, this->dimc.inner_port_bytes,
        (unsigned)g.nb_vec,
        (unsigned long)this->job_entry_cycle, (unsigned long)now_cycle,
        (unsigned long)job_busy, (unsigned long)ideal,
        job_busy ? (1.0 * ideal) / (1.0 * job_busy) : 0.0,
        this->jobs_measured,
        (unsigned long)this->acc_busy_cycles,
        (unsigned long)this->acc_gap_cycles,
        this->acc_busy_cycles
            ? (1.0 * this->acc_ideal_cycles) / (1.0 * this->acc_busy_cycles) : 0.0);
    this->text->msg(vp::TraceLevel::WARNING,
        "  beats=%lu avg_lat=%.2f stalled_on_depth=%lu (depth=%u)\n",
        (unsigned long)this->acc_beats,
        this->acc_beats ? (1.0 * this->acc_beat_lat) / (1.0 * this->acc_beats) : 0.0,
        (unsigned long)this->acc_stall_full, this->dimc.outstanding_depth);
}

void Dimc_Tracer::job_end()
{
    uint8_t zero = 0, st = DIMC_IDLE, why = DIMC_IDLE_WHY_WAIT_COMMIT;
    this->cur_state = DIMC_IDLE;
    this->busy_event.event(&zero);
    this->state_event.event(&st);
    // job_queued, called after this when another job is waiting, overrides it.
    this->idle_why_event.event(&why);
    // Levels hold until the next event, so drop every activity level here or
    // the job's last cycle stays drawn across the idle gap.
    for (Block &blk : this->blocks) {
        blk.load_event.event(&zero);
        blk.comp_event.event(&zero);
        blk.wb_event.event(&zero);
        for (Macro &mac : blk.macros) {
            mac.load_event.event(&zero);
            mac.comp_event.event(&zero);
            mac.wb_event.event(&zero);
            mac.why_event.event(&zero);
            mac.load_kind_event.event(&zero);
        }
    }
    this->text->msg(vp::TraceLevel::WARNING,
        "DIMC job done, STATUS=1, finished_jobs=%u\n", this->dimc.finished_jobs);
}
