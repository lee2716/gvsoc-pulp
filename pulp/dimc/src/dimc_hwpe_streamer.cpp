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
    this->d0_iters  = 0;
    this->d1_iters  = 0;
    this->d2_iters  = 0;
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
    this->d0_iters  = 0;
    this->d1_iters  = 0;
    this->d2_iters  = 0;

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

void Dimc_HWPE_Streamer::set_base_addr(uint32_t addr) { this->base_addr = addr; }
uint32_t Dimc_HWPE_Streamer::get_base_addr() { return this->base_addr; }
// tot_len is a BYTE count; the linear DIMC stream is done once pos (bytes
// consumed from base) reaches it. This keeps termination independent of the
// per-call chunk size, which can be swept at run time.
bool Dimc_HWPE_Streamer::is_done() { return this->pos >= this->tot_len; }

int Dimc_HWPE_Streamer::rw_data(int width, void* buf, strobe_t strb) {
    uint32_t offs = (this->base_addr + this->pos);

    if (this->is_done()) {
        return 1;
    }

    // Functional transfer: one request of `width` bytes. The tile L1 interleaver
    // (HWPEInterleaver) splits it across the 32 word-interleaved TCDM banks; we
    // only need the data movement here and model the timing ourselves below.
    if (buf != NULL) {
        this->req->prepare();
        this->req->set_addr(offs);
        this->req->set_data((uint8_t *) buf);
        this->req->set_size(width);
        vp::IoReqStatus err = this->dimc->stream_mst.req(this->req);
        if (err != vp::IO_REQ_OK) {
            this->dimc->trace.fatal("There was an error while reading/writing data\n");
            return 0;
        }
    }

    this->pos += (uint32_t)width;
    this->tot_iters++;

    // ---- L1 bandwidth + bank-conflict timing model ----
    // The TCDM delivers  l1_bw = (nb_banks * bank_width)  bytes per cycle for an
    // aligned contiguous access. A request wider than l1_bw hits some banks more
    // than once (intra-request bank conflict) and serialises into ceil(width/l1_bw)
    // cycles. stream_bank_bytes carries l1_bw (bytes/cycle, default 128 = 32*4).
    uint32_t l1_bw = this->dimc->stream_bank_bytes;
    if (l1_bw == 0) l1_bw = 128;
    int transfer = ((int)width + (int)l1_bw - 1) / (int)l1_bw;   // ceil, >=1
    return transfer + (int) this->dimc->stream_sync;
}

int Dimc_HWPE_Streamer::iterate(void* buf, strobe_t strb) {
    int latency = 1;
    return latency;
}
