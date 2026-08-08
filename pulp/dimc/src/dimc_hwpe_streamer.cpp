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


Dimc_HWPE_Streamer::Dimc_HWPE_Streamer(Dimc_HWPE* dimc, bool is_write) {
    this->dimc = dimc;

    this->base_addr = 0;
    this->tot_len   = 0;
    this->d0_len    = 0;
    this->d0_stride = 0;
    this->d1_len    = 0;
    this->d1_stride = 0;
    this->d2_len    = 0;
    this->d2_stride = 0;
    this->d3_stride = 0;
    this->pos       = 0;
    this->tot_iters = 0;
    this->req       = this->dimc->stream_mst.req_new(0, 0, 0, is_write);
    this->is_write  = is_write;
}

Dimc_HWPE_Streamer::Dimc_HWPE_Streamer() {
    this->dimc = (Dimc_HWPE *) NULL;
}

void Dimc_HWPE_Streamer::configure(
        uint32_t base_addr,
        uint32_t tot_len,
        uint32_t d0_len,
        uint32_t d0_stride,
        uint32_t d1_len,
        uint32_t d1_stride,
        uint32_t d2_len,
        uint32_t d2_stride,
        uint32_t d3_stride
) {
    this->base_addr = base_addr;
    this->tot_len   = tot_len;
    this->d0_len    = d0_len;
    this->d0_stride = d0_stride;
    this->d1_len    = d1_len;
    this->d1_stride = d1_stride;
    this->d2_len    = d2_len;
    this->d2_stride = d2_stride;
    this->d3_stride = d3_stride;
    this->pos       = 0;
    this->tot_iters = 0;

    this->dimc->trace.msg("base addr %x\ntot len %d\nd0 len %d\nd0 stride %d\nd1 len %d\nd1 stride %d\nd2 stride %d\nd3 stride %d\n",
        this->base_addr,
        this->tot_len,
        this->d0_len,
        this->d0_stride,
        this->d1_len,
        this->d1_stride,
        this->d2_stride,
        this->d3_stride
    );
}

// tot_len is a BYTE count. This bound is looser than the engine's beat_total,
// so it never ends a phase early: TOTAL_LENGTH and NUM_MACROS come from the same
// num_jobs, and the engine only clamps num_active downwards.
bool Dimc_HWPE_Streamer::is_done() { return this->pos >= this->tot_len; }

// Issue one beat of at most inner_port_bytes and return the latency the memory
// reported. The caller advances one beat per cycle and tracks the in-flight
// response itself, which is what lets several accesses overlap.
int Dimc_HWPE_Streamer::issue_beat(int width, void* buf) {
    uint32_t base = this->base_addr + this->pos;

    if (this->is_done()) {
        return 1;
    }
    if (width <= 0) {
        return (int) this->dimc->port_sync_cycles;
    }

    // One beat carries at most a port word. Bank-granularity over-fetch is not
    // modelled: L1 is addressed at byte granularity here.
    const uint32_t port_bytes = this->dimc->inner_port_bytes;
    int beat = (width < (int)port_bytes) ? width : (int)port_bytes;

    int64_t latency = 1;
    if (buf != NULL) {
        this->req->prepare();
        this->req->set_addr(base);
        this->req->set_data((uint8_t *) buf);
        this->req->set_size(beat);
        vp::IoReqStatus err = this->dimc->stream_mst.req(this->req);
        if (err != vp::IO_REQ_OK) {
            this->dimc->trace.fatal("Error while issuing a TCDM beat\n");
            return 0;
        }
        latency = (int64_t) this->req->get_latency();
    }

    this->pos += (uint32_t)beat;
    this->tot_iters++;

    return (int)latency + (int)this->dimc->port_sync_cycles;
}
