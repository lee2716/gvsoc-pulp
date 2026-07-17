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
    // Configuration from the systree. gvrun --param / set_stream_params set these
    // at launch without recompiling the SW test:
    //   make run runner_args="--param dimc_chunk=64 --param dimc_l1bw=128 --param dimc_sync=0"
    this->num_macros         = (uint32_t)this->get_js_config()->get_child_int("num_macros");
    this->stream_chunk_bytes = (uint32_t)this->get_js_config()->get_child_int("stream_chunk_bytes");
    this->stream_bank_bytes  = (uint32_t)this->get_js_config()->get_child_int("stream_bank_bytes");
    this->stream_sync        = (uint32_t)this->get_js_config()->get_child_int("stream_sync");
    this->stream_noc_lat     = (uint32_t)this->get_js_config()->get_child_int("stream_noc_lat");
    this->stream_min_bank    = (uint32_t)this->get_js_config()->get_child_int("stream_min_bank");
    this->stream_prefetch    = (uint32_t)this->get_js_config()->get_child_int("stream_prefetch");
    if (this->stream_min_bank == 0)    this->stream_min_bank = 4;   // MAGIA bank word = 4 B
    if (this->num_macros == 0)         this->num_macros = 2;
    if (this->stream_chunk_bytes == 0) this->stream_chunk_bytes = 128; // 1 KB row/cycle
    if (this->stream_bank_bytes == 0)  this->stream_bank_bytes  = 128; // 32 banks * 4 B
    // ---- OUTER BLOCK knobs (default = today's behavior => no-op) ----
    // Read AFTER stream_bank_bytes / stream_noc_lat are finalized so the coupled
    // defaults (l2bw=l1bw, noc_l2=noc_lat) resolve against the effective values.
    int64_t nb = this->get_js_config()->get_child_int("stream_num_block");
    int64_t ls = this->get_js_config()->get_child_int("stream_l2_shared");
    int64_t ld = this->get_js_config()->get_child_int("stream_l1_depth");
    int64_t lb = this->get_js_config()->get_child_int("stream_l2_bw");
    int64_t n2 = this->get_js_config()->get_child_int("stream_noc_l2");
    this->stream_num_block = (nb <= 0) ? 1 : (uint32_t)nb;
    this->stream_l2_shared = (ls <  0) ? 1 : (uint32_t)ls;   // default 1; 0 is valid (parallel blocks)
    this->stream_l1_depth  = (ld <= 0) ? 1 : (uint32_t)ld;
    this->stream_l2_bw     = (lb <= 0) ? this->stream_bank_bytes : (uint32_t)lb; // 0 => same as l1bw
    this->stream_noc_l2    = (n2 <  0) ? this->stream_noc_lat   : (uint32_t)n2;  // <0 => same as noc_lat
    // stream_sync defaults to 0 (pipelined, RTL gnt=1). stream_noc_lat defaults
    // to 1 (dimc.py always sets it; 0 is a valid "ideal NoC" value so not forced).
    // stream_prefetch defaults to 0 (off): keeps every existing figure's numbers.
    this->last_kb_src     = 0xFFFFFFFF;   // no resident weights yet
    this->prev_drain_slack = 0;           // no previous trigger to prefetch from
    // HWPE slave port
    this->hwpe_slv.set_req_meth(&Dimc_HWPE::hwpe_slave);
    this->new_slave_port("hwpe_slv", &this->hwpe_slv);

    // Streamer master port
    this->new_master_port("stream_mst", &this->stream_mst);

    // Completion interrupt master port (standard HWPE done_irq)
    this->new_master_port("irq", &this->irq);

    // Streamers
    this->weight_stream = Dimc_HWPE_Streamer(this, false);
    this->input_stream  = Dimc_HWPE_Streamer(this, false);
    this->out_stream    = Dimc_HWPE_Streamer(this, true);

    // Macros
    this->macros.resize(this->num_macros);

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
    this->state.set(DIMC_IDLE);

    // Traces
    this->traces.new_trace("trace", &this->trace);

    this->trace.msg(vp::TraceLevel::WARNING,
        "DIMC systree config: num_macros=%u chunk=%u l1bw=%u sync=%u noc_lat=%u\n",
        this->num_macros, this->stream_chunk_bytes,
        this->stream_bank_bytes, this->stream_sync, this->stream_noc_lat);
}

void Dimc_HWPE::reset(bool active)
{
    if (active) {
        for (uint32_t i = 0; i < N_CFG_REGS; i++) {
            this->register_file[i] = 0x0;
        }
        for (auto &m : this->macros) m.reset();
        this->sel_dimc = 0;
        this->last_kb_src = 0xFFFFFFFF;
        this->prev_drain_slack = 0;        // no prefetch across a reset boundary
        this->running_job   = 0;
        this->next_job_id   = 0;
        this->finished_jobs = 0;
        this->job_running   = false;
        this->state.set(DIMC_IDLE);
    }
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
            _this->register_file[(address >> 2)] = data;
        } else {
            switch (address) {
            case DIMC_HWPE_TRIG:   // commit_and_trigger: latch this job's id and start
                _this->running_job = _this->next_job_id++;
                _this->register_file[DIMC_HWPE_RUN_TASK >> 2] = _this->running_job;
                _this->event_enqueue(_this->fsm_start_event, 1);
                break;
            case DIMC_HWPE_ACQ:    // acquire is observed on the read path; write is a no-op
                break;
            case DIMC_HWPE_SOFT_CLEAR:
                for (auto &m : _this->macros) m.reset();
                _this->sel_dimc = 0;
                _this->last_kb_src = 0xFFFFFFFF;   // resident weights invalidated
                _this->prev_drain_slack = 0;       // and no prefetch across soft_clear (single trigger)
                for (uint32_t i = 0; i < N_CFG_REGS; i++)
                    _this->register_file[i] = 0x0;
                _this->state.set(DIMC_IDLE);
                break;
            default:
                break;
            }
        }
    } else {
        // ACQUIRE: hand out a job id if a context is free, 0xFFFFFFFF (-1) if busy.
        if (address == DIMC_HWPE_ACQ) {
            *(uint32_t *)req->get_data() = _this->job_running ? 0xFFFFFFFFu
                                                              : _this->next_job_id;
            return vp::IO_REQ_OK;
        }
        if (address > DIMC_HWPE_REG_MAX) {
            _this->trace.fatal("Trying to access invalid address 0x%x\n", address);
            return vp::IO_REQ_INVALID;
        }
        *(uint32_t *)req->get_data() = _this->register_file[(address >> 2)];
    }

    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Dimc_HWPE(config);
}
