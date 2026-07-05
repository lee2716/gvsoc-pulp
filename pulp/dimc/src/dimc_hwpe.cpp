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
 *
 * Author: Cong Li <cong.li3@unibo.it>
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
    if (this->num_macros == 0)         this->num_macros = 2;
    if (this->stream_chunk_bytes == 0) this->stream_chunk_bytes = 128; // 1 KB row/cycle
    if (this->stream_bank_bytes == 0)  this->stream_bank_bytes  = 128; // 32 banks * 4 B
    // stream_sync defaults to 0 (pipelined, RTL gnt=1). stream_noc_lat defaults
    // to 1 (dimc.py always sets it; 0 is a valid "ideal NoC" value so not forced).
    this->last_kb_src = 0xFFFFFFFF;   // no resident weights yet
    // HWPE slave port
    this->hwpe_slv.set_req_meth(&Dimc_HWPE::hwpe_slave);
    this->new_slave_port("hwpe_slv", &this->hwpe_slv);

    // Streamer master port
    this->new_master_port("stream_mst", &this->stream_mst);

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

    // Initial state of the controller FSM
    this->sel_dimc = 0;
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
            case DIMC_HWPE_TRIG:
                _this->event_enqueue(_this->fsm_start_event, 1);
                break;
            case DIMC_HWPE_SOFT_CLEAR:
                for (auto &m : _this->macros) m.reset();
                _this->sel_dimc = 0;
                _this->last_kb_src = 0xFFFFFFFF;   // resident weights invalidated
                for (uint32_t i = 0; i < N_CFG_REGS; i++)
                    _this->register_file[i] = 0x0;
                _this->state.set(DIMC_IDLE);
                break;
            default:
                break;
            }
        }
    } else {
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
