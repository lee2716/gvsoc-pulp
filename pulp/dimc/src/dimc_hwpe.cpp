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

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>

#include <dimc.hpp>

Dimc_HWPE::Dimc_HWPE(vp::ComponentConf &config) : vp::Component(config)
{
    // Registered first so the systree check below can report through it.
    this->traces.new_trace("trace", &this->trace);

    // Architecture, from the systree. Dimc() in dimc.py writes every property.
    this->num_macros        = (uint32_t)this->get_js_config()->get_child_int("num_macros");
    this->inner_port_bytes  = (uint32_t)this->get_js_config()->get_child_int("inner_port_bytes");
    this->port_sync_cycles  = (uint32_t)this->get_js_config()->get_child_int("port_sync_cycles");
    this->inner_noc_lat     = (uint32_t)this->get_js_config()->get_child_int("inner_noc_lat");
    this->nb_inner_blocks   = (uint32_t)this->get_js_config()->get_child_int("nb_inner_blocks");
    this->outer_port_shared = (uint32_t)this->get_js_config()->get_child_int("outer_port_shared");
    this->outer_port_bytes  = (uint32_t)this->get_js_config()->get_child_int("outer_port_bytes");
    this->outer_noc_lat     = (uint32_t)this->get_js_config()->get_child_int("outer_noc_lat");

    this->last_kb_src     = 0xFFFFFFFF;   // no resident weights yet
    // HWPE slave port
    this->hwpe_slv.set_req_meth(&Dimc_HWPE::hwpe_slave);
    this->new_slave_port("hwpe_slv", &this->hwpe_slv);

    // Streamer master port
    this->new_master_port("stream_mst", &this->stream_mst);

    // Completion interrupt master port (standard HWPE done_irq)
    this->new_master_port("done_irq", &this->irq);

    // ---- Inner blocks ----
    // num_macros macros per block, nb_inner_blocks blocks. Each block owns its
    // streamers and macros; this component drives them all, like redmule/ne16.
    this->inner_blocks.resize(this->nb_inner_blocks);
    for (Dimc_InnerBlock &blk : this->inner_blocks) {
        blk.weight_stream = Dimc_HWPE_Streamer(this, false);
        blk.input_stream  = Dimc_HWPE_Streamer(this, false);
        blk.out_stream    = Dimc_HWPE_Streamer(this, true);
        blk.macros.resize(this->num_macros);
        blk.reset_job_state();
    }

    // ---- Outer ports ----
    // One inner block reaches memory directly, so no port is created and the
    // engine matches the single-block model bit for bit. With more blocks,
    // outer_port_shared picks the topology: one shared port or one each.
    if (this->nb_inner_blocks > 1) {
        uint32_t nb_ports = this->outer_port_shared ? 1 : this->nb_inner_blocks;
        this->outer_ports.resize(nb_ports);
        for (Dimc_OuterPort &p : this->outer_ports)
            p.configure(this->outer_port_bytes, this->outer_noc_lat);
    }

    // Event handlers
    this->fsm_start_event = this->event_new(&Dimc_HWPE::fsm_start_handler);
    this->fsm_event       = this->event_new(&Dimc_HWPE::fsm_handler);
    this->fsm_end_event   = this->event_new(&Dimc_HWPE::fsm_end_handler);

    // Initial state of the controller FSM + standard HWPE offload bookkeeping
    this->sel_dimc      = 0;
    this->running_job   = 0;
    this->next_job_id   = 0;
    this->finished_jobs = 0;
    this->job_running   = false;
    this->acquired_ctx  = -1;
    this->running_ctx   = -1;
    this->next_ctx   = -1;
    this->spare_ctx    = -1;
    for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++) {
        this->ctx_busy[ctx]   = false;
        this->ctx_job_id[ctx] = 0;
        for (uint32_t i = 0; i < DIMC_HWPE_NB_JOB_REGS; i++) this->ctx_regs[ctx][i] = 0;
    }
    this->outstanding_depth = 4;    // in-flight TCDM beats (light_redmule queue_depth)
    this->fsm_timestamp     = 0;
    this->kb_beats_per_row  = 1;
    this->fb_beats_per_macro= 1;
    this->beats_per_macro   = 1;
    this->phase_planned     = false;
    this->state.set(DIMC_IDLE);

    this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC systree config: num_macros=%u l1bw=%u sync=%u noc_lat=%u "
        "nb_blocks=%u blocks=%u ports=%u outer_bw=%u outer_noc=%u\n",
        this->num_macros,
        this->inner_port_bytes, this->port_sync_cycles, this->inner_noc_lat,
        this->nb_inner_blocks, (uint32_t)this->inner_blocks.size(),
        (uint32_t)this->outer_ports.size(), this->outer_port_bytes,
        this->outer_noc_lat);
}

void Dimc_HWPE::reset(bool active)
{
    if (active) {
        // A zero means the property never reached the model. Abort rather than
        // substitute a default, which would run a different machine silently.
        // trace_file is only valid once the trace engine has started, so this
        // check cannot live in the constructor.
        if (this->num_macros == 0 || this->inner_port_bytes == 0 ||
            this->nb_inner_blocks == 0 || this->outer_port_bytes == 0) {
            // trace.fatal writes to stdout and ends in abort(), which does not
            // flush stdio, so its message is lost whenever stdout is a pipe.
            // stderr is unbuffered and always reaches the user.
            fprintf(stderr,
                    "DIMC systree incomplete: num_macros=%u inner_port_bytes=%u "
                    "nb_inner_blocks=%u outer_port_bytes=%u (all must be non-zero)\n",
                    this->num_macros, this->inner_port_bytes,
                    this->nb_inner_blocks, this->outer_port_bytes);
            this->trace.fatal("DIMC systree incomplete\n");
        }
        for (uint32_t i = 0; i < N_CFG_REGS; i++) {
            this->register_file[i] = 0x0;
        }
        for (Dimc_InnerBlock &blk : this->inner_blocks) for (auto &m : blk.macros) m.reset();
        for (Dimc_OuterPort &p : this->outer_ports) p.reset();
        this->sel_dimc = 0;
        this->last_kb_src = 0xFFFFFFFF;
        this->running_job   = 0;
        this->next_job_id   = 0;
        this->finished_jobs = 0;
        this->job_running   = false;
        this->acquired_ctx  = -1;
        this->running_ctx   = -1;
        this->next_ctx   = -1;
        this->spare_ctx    = -1;
        for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++) {
            this->ctx_busy[ctx]   = false;
            this->ctx_job_id[ctx] = 0;
            for (uint32_t i = 0; i < DIMC_HWPE_NB_JOB_REGS; i++) this->ctx_regs[ctx][i] = 0;
        }
        this->outstanding_depth = 4;    // in-flight TCDM beats (light_redmule queue_depth)
        this->fsm_timestamp     = 0;
        this->kb_beats_per_row  = 1;
        this->fb_beats_per_macro= 1;
        this->beats_per_macro   = 1;
        this->phase_planned     = false;
        for (Dimc_InnerBlock &blk : this->inner_blocks) blk.reset_job_state();
        this->state.set(DIMC_IDLE);
    }
}

// Reserve a free job context; -1 when every context is occupied.
int Dimc_HWPE::ctx_alloc()
{
    for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++)
        if (!this->ctx_busy[ctx]) return ctx;
    return -1;
}

// Read a job-dependent register out of the context the engine is executing.
uint32_t Dimc_HWPE::job_reg(uint32_t addr) const
{
    int ctx = (this->running_ctx >= 0) ? this->running_ctx : 0;
    return this->ctx_regs[ctx][(addr - DIMC_HWPE_JOB_BASE) >> 2];
}

// Launch the committed-but-waiting context, if the engine is free.
void Dimc_HWPE::start_next_job()
{
    if (this->job_running || this->next_ctx < 0) return;
    this->running_ctx = this->next_ctx;
    this->next_ctx = this->spare_ctx;   // a 2nd committed job moves up the queue
    this->spare_ctx  = -1;
    this->job_running = true;
    this->running_job = this->ctx_job_id[this->running_ctx];
    this->register_file[DIMC_HWPE_RUN_TASK >> 2] = this->running_job;
    this->event_enqueue(this->fsm_start_event, 1);
}

vp::IoReqStatus Dimc_HWPE::hwpe_slave(vp::Block *__this, vp::IoReq *req)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    uint32_t address = req->get_addr();

    if (req->get_is_write()) {
        uint32_t data = *((uint32_t *) (req->get_data()));

        _this->trace.msg(vp::TraceLevel::DEBUG, "Write request, address: 0x%x\n", address);

        if (address > DIMC_HWPE_CHK_STATE) {
            if (address > DIMC_HWPE_REG_MAX) {
                _this->trace.fatal("Trying to access invalid address 0x%x\n", address);
                return vp::IO_REQ_INVALID;
            }
            if (address >= DIMC_HWPE_JOB_BASE) {
                // Job-dependent write -> goes into the context SW acquired.
                // Software that never reads ACQUIRE gets one allocated here, so
                // a plain "configure then trigger" sequence also works.
                if (_this->acquired_ctx < 0) _this->acquired_ctx = _this->ctx_alloc();
                int ctx = (_this->acquired_ctx >= 0) ? _this->acquired_ctx : 0;
                _this->ctx_busy[ctx] = true;
                _this->ctx_regs[ctx][(address - DIMC_HWPE_JOB_BASE) >> 2] = data;
            } else {
                // Mandatory / generic (job-independent) registers stay unbanked.
                _this->register_file[(address >> 2)] = data;
            }
        } else {
            switch (address) {
            case DIMC_HWPE_TRIG: {
                // commit_trigger (hwpe-ctrl): the written value selects the mode.
                //   0 : commit the acquired job and start it
                //   1 : commit only; it runs on a later trigger or on an explicit 0x2
                //   2 : trigger the existing queue, commit nothing new
                uint32_t mode = data & 0x3;

                if (mode != 0x2) {                           // modes 0 and 1 commit
                    int ctx = _this->acquired_ctx;
                    if (ctx < 0) ctx = _this->ctx_alloc();       // commit with no writes
                    if (ctx < 0) break;                        // all contexts busy: drop
                    _this->ctx_busy[ctx]   = true;
                    _this->ctx_job_id[ctx] = _this->next_job_id++;
                    _this->acquired_ctx  = -1;               // SW must ACQUIRE again
                    // Queue it. A job already waiting keeps its place: the queue is
                    // FIFO over the contexts, next_ctx points at the head.
                    if (_this->next_ctx < 0) _this->next_ctx = ctx;
                    else                        _this->spare_ctx  = ctx;
                }

                // Modes 0 and 2 release the queue; mode 1 only commits.
                if (mode != 0x1) _this->start_next_job();
                break;
            }
            case DIMC_HWPE_ACQ:    // acquire is observed on the read path; write is a no-op
                break;
            case DIMC_HWPE_SOFT_CLEAR: {
                // soft_clear (hwpe-ctrl): the written value selects the scope.
                //   0 : clear IP state and the register file
                //   1 : clear IP state, keep the register file
                //   2 : clear the register file only
                uint32_t scope = data & 0x3;

                if (scope != 0x2) {                 // scopes 0 and 1 clear IP state
                    for (Dimc_InnerBlock &blk : _this->inner_blocks)
                        for (auto &m : blk.macros) m.reset();
                    _this->sel_dimc = 0;
                    _this->last_kb_src = 0xFFFFFFFF;   // resident weights invalidated
                    // Aborts any in-flight job: release every context, otherwise
                    // ACQUIRE would report busy forever and acquire_block() hangs.
                    _this->job_running  = false;
                    _this->acquired_ctx = -1;
                    _this->running_ctx  = -1;
                    _this->next_ctx  = -1;
                    _this->spare_ctx   = -1;
                    for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++) _this->ctx_busy[ctx] = false;
                }
                if (scope != 0x1) {                 // scopes 0 and 2 clear the regfile
                    for (uint32_t i = 0; i < N_CFG_REGS; i++)
                        _this->register_file[i] = 0x0;
                    for (int ctx = 0; ctx < DIMC_NB_CONTEXT; ctx++)
                        for (uint32_t i = 0; i < DIMC_HWPE_NB_JOB_REGS; i++)
                            _this->ctx_regs[ctx][i] = 0x0;
                }
                if (scope != 0x2) _this->state.set(DIMC_IDLE);
                // Re-publish the monotonic counters the register_file wipe cleared.
                _this->register_file[DIMC_HWPE_FIN_JOBS >> 2] = _this->finished_jobs;
                _this->register_file[DIMC_HWPE_RUN_TASK >> 2] = _this->running_job;
                break;
            }
            default:
                break;
            }
        }
    } else {
        // ACQUIRE: on read, start a job offload and lock the controller.
        // Reserving a context is the lock: job-dependent writes are routed into
        // it and no other offload can claim it until commit_trigger (0x0/0x1)
        // or soft_clear releases it. Returns 0xFFFFFFFF only when all contexts
        // are busy, so with DIMC_NB_CONTEXT=2 a second job can be queued.
        if (address == DIMC_HWPE_ACQ) {
            if (_this->acquired_ctx < 0) _this->acquired_ctx = _this->ctx_alloc();
            if (_this->acquired_ctx < 0) {
                *(uint32_t *)req->get_data() = 0xFFFFFFFFu;
            } else {
                _this->ctx_busy[_this->acquired_ctx] = true;
                *(uint32_t *)req->get_data() = _this->next_job_id;
            }
            return vp::IO_REQ_OK;
        }
        if (address > DIMC_HWPE_REG_MAX) {
            _this->trace.fatal("Trying to access invalid address 0x%x\n", address);
            return vp::IO_REQ_INVALID;
        }
        if (address >= DIMC_HWPE_JOB_BASE) {
            // Job-dependent reads must come from the same bank the writes went to,
            // otherwise a read-back of a just-written job register returns 0.
            int ctx = (_this->acquired_ctx >= 0) ? _this->acquired_ctx
                  : (_this->running_ctx  >= 0) ? _this->running_ctx : 0;
            *(uint32_t *)req->get_data() =
                _this->ctx_regs[ctx][(address - DIMC_HWPE_JOB_BASE) >> 2];
            return vp::IO_REQ_OK;
        }
        *(uint32_t *)req->get_data() = _this->register_file[(address >> 2)];
    }

    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Dimc_HWPE(config);
}
