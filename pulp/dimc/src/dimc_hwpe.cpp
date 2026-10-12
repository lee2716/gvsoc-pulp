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

#include <cstring>
#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>

#include <string>
#include <dimc.hpp>

Dimc_HWPE::Dimc_HWPE(vp::ComponentConf &config) : vp::Component(config), tracer(*this)
{
    // Registered first: every message this component emits, here and in
    // reset(), goes through it.
    this->traces.new_trace("trace", &this->trace);

    // Architecture, from the systree. Dimc() in dimc.py writes every property.
    this->num_macros        = (uint32_t)this->get_js_config()->get_child_int("num_macros");
    this->inner_port_bytes  = (uint32_t)this->get_js_config()->get_child_int("inner_port_bytes");
    this->nb_inner_blocks   = (uint32_t)this->get_js_config()->get_child_int("nb_inner_blocks");
    // HWPE slave port
    this->hwpe_slv.set_req_meth(&Dimc_HWPE::hwpe_slave);
    this->new_slave_port("hwpe_slv", &this->hwpe_slv);

    // Streamer master port
    this->new_master_port("stream_mst", &this->stream_mst);

    // Completion interrupt master port (standard HWPE done_irq)
    this->new_master_port("done_irq", &this->irq);

    // ---- Inner blocks ----
    // Each block owns its streamers and macros; this component drives them all.
    this->inner_blocks.resize(this->nb_inner_blocks);
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.macros.resize(this->num_macros);
        for (uint32_t m = 0; m < this->num_macros; m++) {
            blk.weight_stream.emplace_back(this, false);
            blk.input_stream .emplace_back(this, false);
        }
        blk.out_stream.emplace_back(this, true);
        blk.r_channel.configure(DIMC_DUAL_PORT_BYTES);
        blk.w_channel.configure(DIMC_DUAL_PORT_BYTES);
        blk.reset_all();
    }

    // Every trace signal lives in the tracer, registered once the blocks are
    // sized.
    this->tracer.build(*this, this->trace, (uint32_t)this->inner_blocks.size(),
                       this->num_macros);

    // Event handlers
    this->fsm_start_event = this->event_new(&Dimc_HWPE::fsm_start_handler);
    this->fsm_event       = this->event_new(&Dimc_HWPE::fsm_handler);
    this->fsm_end_event   = this->event_new(&Dimc_HWPE::fsm_end_handler);

    this->next_gemm_id  = 0;
    this->finished_jobs = 0;
    this->next_job_id   = 0;
    std::memset(this->gemm_live, 0, sizeof(this->gemm_live));
    std::memset(this->register_file, 0, sizeof(this->register_file));
    this->clear_engine();
    this->outstanding_depth = 4;    // max in-flight requests per feed (kernel, input, store) per block
    this->fsm_timestamp     = 0;
    this->job_start_cycle   = 0;
    this->job_geom[0].kb_beats_per_row  = 1;
    this->job_geom[0].fb_beats_per_macro= 1;
    this->job_geom[0].beats_per_macro   = 1;
    this->fill_active = false;
    this->state.set(DIMC_IDLE);


    this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC systree config: num_macros=%u section_bytes=%u nb_blocks=%u blocks=%u\n",
        this->num_macros, this->inner_port_bytes,
        this->nb_inner_blocks, (uint32_t)this->inner_blocks.size());
}

void Dimc_HWPE::reset(bool active)
{
    if (active) {
        // A zero means the property never reached the model. Abort rather than
        // substitute a default, which would run a different machine silently.
        // Checked at reset, so the abort happens once the trace engine runs.
        if (this->num_macros == 0 || this->inner_port_bytes == 0 || this->nb_inner_blocks == 0) {
            // trace.fatal ends in abort(), which does not flush stdio, so its message
            // is lost when the trace goes to a pipe; stderr is unbuffered.
            fprintf(stderr,
                    "DIMC systree incomplete: num_macros=%u inner_port_bytes=%u "
                    "nb_inner_blocks=%u (all must be non-zero)\n",
                    this->num_macros, this->inner_port_bytes, this->nb_inner_blocks);
            this->trace.fatal("DIMC systree incomplete\n");
        }
        this->clear_engine();
        std::memset(this->register_file, 0, sizeof(this->register_file));
        std::memset(this->gemm_live, 0, sizeof(this->gemm_live));
        this->next_gemm_id  = 0;
        this->finished_jobs = 0;
        this->next_job_id   = 0;
        this->outstanding_depth = 4;    // max in-flight requests per feed (kernel, input, store) per block
        this->fsm_timestamp     = 0;
        this->job_start_cycle   = 0;

        this->tracer.reset();
    }
}

// The engine and the job queue as a reset leaves them: no FSM event pending, no job running or
// queued, every context free, every FIFO, cursor and pending queue empty, the accumulators
// zero. Job ids, the cycle count and the registers are left to the caller.
void Dimc_HWPE::clear_engine()
{
    for (vp::ClockEvent *ev : {this->fsm_start_event, this->fsm_event, this->fsm_end_event})
        if (ev->is_enqueued()) this->event_cancel(ev);
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.r_channel.reset();
        blk.w_channel.reset();
        for (Dimc_Macro &mc : blk.macros) {
            mc.reset();
            mc.filled_job  = Dimc_Macro::JOB_NONE;
            mc.issue_job   = Dimc_Macro::JOB_NONE;
            mc.fill_job    = Dimc_Macro::JOB_NONE;
            mc.owed        = 0;
            mc.write_job   = Dimc_Macro::JOB_NONE;
            mc.written     = 0;
            mc.kpf_job     = Dimc_Macro::JOB_NONE;
            mc.kfetched    = 0;
            mc.ffetched    = 0;
            mc.kw = 0; mc.f0 = false; mc.stamped = false;
            mc.runs_issued = 0;
            mc.set_job[0]  = mc.set_job[1] = Dimc_Macro::JOB_NONE;
        }
        blk.reset_all();
        blk.out_fifo.clear();
        blk.last_acc_cycle = -1000;
        blk.acc_results[0] = blk.acc_results[1] = 0;
        blk.present_ack_cycle = -1000;
        for (int32_t &a : blk.acc) a = 0;
    }
    this->gemm_queue.clear();
    this->gemm_running  = false;
    this->gemm_acquired = false;
    this->gemm_error    = false;
    this->job_running   = false;
    this->running_job   = 0;
    this->running_ctx   = -1;
    this->ctx_queue.clear();
    for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++) {
        this->ctx_busy[ctx] = false;
        this->geom_job[ctx] = Dimc_Macro::JOB_NONE;
    }
    this->job_geom[0].kb_beats_per_row   = 1;
    this->job_geom[0].fb_beats_per_macro = 1;
    this->job_geom[0].beats_per_macro    = 1;
    this->fill_active = false;
    this->out_dropped = 0;
    this->state.set(DIMC_IDLE);
}

// A free engine context; -1 when both are taken.
int Dimc_HWPE::ctx_alloc()
{
    for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++)
        if (!this->ctx_busy[ctx]) return ctx;
    return -1;
}

uint32_t Dimc_HWPE::job_reg_ctx(int ctx, uint32_t idx) const
{
    return this->ctx_regs[ctx < 0 ? 0 : ctx][idx];
}

// Launch the tile product at the head of the engine's queue, if the engine is free.
void Dimc_HWPE::start_next_job()
{
    if (this->job_running || this->ctx_queue.empty()) return;
    this->running_ctx = this->ctx_queue.front();
    this->ctx_queue.pop_front();
    this->job_running = true;
    this->running_job = this->ctx_job_id[this->running_ctx];
    this->event_enqueue(this->fsm_start_event, 1);
    this->tracer.job_queued();
}

// The checks of dimc_ctrl.sv: complete 8-bit tiles, matching inner dimension, aligned
// addresses, no matrix past the end of the address space.
bool Dimc_HWPE::gemm_valid(const Gemm &g) const
{
    const uint64_t wr = g.reg[DIMC_REG_WEIGHT_ROWS], wc = g.reg[DIMC_REG_WEIGHT_COLS];
    const uint64_t ir = g.reg[DIMC_REG_INPUT_ROWS], ic = g.reg[DIMC_REG_INPUT_COLS];
    const uint64_t end = (uint64_t)1 << 32;
    return wr != 0 && wc != 0 && ic != 0 && wc == ir
        && wr % DIMC_MACRO_KB_LEN == 0 && wc % DIMC_MACRO_KB_EW == 0 && ic % DIMC_ACC_VECTORS == 0
        && (g.reg[DIMC_REG_FORMAT] & 0x3) == DIMC_CI_8BIT
        && g.reg[DIMC_REG_INPUT_ADDR] % 8 == 0 && g.reg[DIMC_REG_KERNEL_ADDR] % 32 == 0
        && g.reg[DIMC_REG_OUTPUT_ADDR] % 32 == 0
        && g.reg[DIMC_REG_KERNEL_ADDR] + wr * wc <= end
        && g.reg[DIMC_REG_INPUT_ADDR] + ir * ic <= end
        && g.reg[DIMC_REG_OUTPUT_ADDR] + wr * ic * 4 <= end;
}

// Describe the next tile product of the running job in engine context ctx and queue it:
// output tiles gemm_k.. (one per inner block), vectors 8 * gemm_q.., columns 128 * gemm_l...
// The tile products of one output tile run on consecutive l; then the next q, then the next k.
void Dimc_HWPE::gemm_commit_tile(int ctx)
{
    const uint32_t *r = this->gemm.reg;
    const uint32_t wc = r[DIMC_REG_WEIGHT_COLS], ic = r[DIMC_REG_INPUT_COLS];
    const uint32_t tiles_k = r[DIMC_REG_WEIGHT_ROWS] / DIMC_MACRO_KB_LEN;
    const uint32_t tiles_l = wc / DIMC_MACRO_KB_EW, tiles_q = ic / DIMC_ACC_VECTORS;
    const uint32_t k = this->gemm_k, q = this->gemm_q, l = this->gemm_l;
    const uint32_t blocks = std::min((uint32_t)this->inner_blocks.size(), tiles_k - k);
    uint32_t *d = this->ctx_regs[ctx];
    d[DIMC_TP_KB_SRC]         = r[DIMC_REG_KERNEL_ADDR] + k * DIMC_MACRO_KB_LEN * wc + l * DIMC_MACRO_KB_EW;
    d[DIMC_TP_KB_ROW_STRIDE]  = wc;
    d[DIMC_TP_FB_SRC]         = r[DIMC_REG_INPUT_ADDR] + l * DIMC_MACRO_FB_EW * ic + q * DIMC_ACC_VECTORS;
    d[DIMC_TP_FB_ROW_STRIDE]  = ic;
    d[DIMC_TP_DST]            = r[DIMC_REG_OUTPUT_ADDR]
                              + (k * DIMC_MACRO_KB_LEN * ic + q * DIMC_ACC_VECTORS) * DIMC_OUT_SLOT_BYTES;
    d[DIMC_TP_OUT_ROW_STRIDE] = ic * DIMC_OUT_SLOT_BYTES;
    d[DIMC_TP_BIAS]           = r[DIMC_REG_BIAS];
    d[DIMC_TP_TILE_LAST]      = l + 1 == tiles_l;
    d[DIMC_TP_NB_BLOCKS]      = blocks;
    this->ctx_ci[ctx]           = (uint8_t)(r[DIMC_REG_FORMAT] & 0x3);
    this->ctx_sign_8b[ctx]      = (uint8_t)((r[DIMC_REG_FORMAT] >> 2) & 0x3);
    this->ctx_compute_mask[ctx] = (uint16_t)(r[DIMC_REG_COMPUTE_MASK] & 0x3FF);
    this->ctx_busy[ctx]   = true;
    this->ctx_job_id[ctx] = this->next_job_id++;
    this->tracer.commit(this->ctx_job_id[ctx]);
    this->ctx_queue.push_back(ctx);
    this->gemm_issued++;
    if (++this->gemm_l == tiles_l) {
        this->gemm_l = 0;
        if (++this->gemm_q == tiles_q) {
            this->gemm_q = 0;
            this->gemm_k += blocks;
        }
    }
}

// The running job has ended, or was rejected: count it and raise done.
void Dimc_HWPE::gemm_finish(bool error)
{
    this->gemm_running = false;
    this->gemm_error   = error;
    this->finished_jobs++;
    this->register_file[DIMC_HWPE_STATUS   >> 2] = 0x1;
    this->register_file[DIMC_HWPE_FIN_JOBS >> 2] = this->finished_jobs;
    if (this->irq.is_bound()) {
        this->irq.sync(true);
        this->irq.sync(false);
    }
}

// The job sequencer: end the running job once its last tile product has, start the next
// queued one, and keep the engine's two contexts filled with its tile products.
void Dimc_HWPE::gemm_step()
{
    for (;;) {
        if (this->gemm_running && this->gemm_issued == this->gemm_total
            && !this->job_running && this->ctx_queue.empty())
            this->gemm_finish(false);
        if (this->gemm_running) break;
        if (this->gemm_queue.empty()) return;
        this->gemm = this->gemm_queue.front();
        this->gemm_queue.pop_front();
        this->register_file[DIMC_HWPE_RUN_TASK >> 2] = this->gemm.id;
        this->register_file[DIMC_HWPE_STATUS   >> 2] = 0x0;
        if (!this->gemm_valid(this->gemm)) {
            this->gemm_finish(true);
            continue;
        }
        const uint32_t *r = this->gemm.reg;
        const uint32_t nb = (uint32_t)this->inner_blocks.size();
        this->gemm_running = true;
        this->gemm_error   = false;
        this->gemm_k = this->gemm_q = this->gemm_l = 0;
        this->gemm_issued = 0;
        this->gemm_total  = ((r[DIMC_REG_WEIGHT_ROWS] / DIMC_MACRO_KB_LEN + nb - 1) / nb)
                          * (r[DIMC_REG_INPUT_COLS] / DIMC_ACC_VECTORS)
                          * (r[DIMC_REG_WEIGHT_COLS] / DIMC_MACRO_KB_EW);
    }
    while (this->gemm_issued < this->gemm_total) {
        const int ctx = this->ctx_alloc();
        if (ctx < 0) break;
        this->gemm_commit_tile(ctx);
    }
    this->start_next_job();
}

vp::IoReqStatus Dimc_HWPE::hwpe_slave(vp::Block *__this, vp::IoReq *req)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    const uint32_t address = req->get_addr();
    uint32_t *data = (uint32_t *)req->get_data();

    if (address > DIMC_HWPE_REG_MAX || (address & 0x3)) {
        _this->trace.fatal("Trying to access invalid address 0x%x\n", address);
        return vp::IO_REQ_INVALID;
    }
    const uint32_t pending = (uint32_t)_this->gemm_queue.size() + (_this->gemm_running ? 1u : 0u);

    if (!req->get_is_write()) {
        if (address == DIMC_HWPE_ACQ) {
            // -2 while an acquired job is not committed, -1 when the queue is full.
            if (_this->gemm_acquired)            *data = 0xFFFFFFFEu;
            else if (pending >= DIMC_NB_CONTEXT) *data = 0xFFFFFFFFu;
            else { _this->gemm_acquired = true;  *data = _this->next_gemm_id; }
        } else if (address == DIMC_HWPE_EXT_STATUS) {
            *data = (_this->gemm_error ? 0x2u : 0x0u) | (pending ? 0x1u : 0x0u);
        } else if (address >= DIMC_IO_BASE) {
            *data = _this->gemm_live[(address - DIMC_IO_BASE) >> 2];
        } else {
            *data = _this->register_file[address >> 2];
        }
        return vp::IO_REQ_OK;
    }

    if (address >= DIMC_IO_BASE) {
        _this->gemm_live[(address - DIMC_IO_BASE) >> 2] = *data;
    } else if (address == DIMC_HWPE_TRIG) {
        // Commit the registers as one job and start it, or queue it behind the running one.
        if (pending < DIMC_NB_CONTEXT) {
            Gemm g;
            g.id = _this->next_gemm_id++;
            std::memcpy(g.reg, _this->gemm_live, sizeof(g.reg));
            _this->gemm_queue.push_back(g);
            _this->gemm_acquired = false;
            _this->gemm_step();
        }
    } else if (address == DIMC_HWPE_SOFT_CLEAR) {
        const bool active = _this->job_running || _this->state.get() != DIMC_IDLE;
        _this->clear_engine();
        if (active) _this->tracer.idle();
        // The aborted jobs count as finished: job id + 1 stays the FIN_JOBS value it ends at.
        _this->finished_jobs = _this->next_gemm_id;
        _this->register_file[DIMC_HWPE_FIN_JOBS >> 2] = _this->finished_jobs;
        _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x1;
    }
    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Dimc_HWPE(config);
}
