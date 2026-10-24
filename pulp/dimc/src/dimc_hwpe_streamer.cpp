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

// tot_len is a byte count; the stream is done once pos has consumed all of it.
bool Dimc_HWPE_Streamer::is_done() { return this->pos >= this->tot_len; }

// Where the next beat reads or writes. `pos` counts bytes consumed, which is
// also the address offset while the walk is linear (d0_len == 0).
// A non-zero d0_len selects a strided walk: runs of d0_len contiguous bytes whose starts
// are d0_stride apart, in groups of d1_len runs whose starts are d1_stride apart.
// Only the kernel streamer uses it (kernel_ctrl in dual_DIMC/rtl/dimc_streamer.sv).
uint32_t Dimc_HWPE_Streamer::walk_addr() const {
    if (this->d0_len == 0) {
        return this->base_addr + this->pos;
    }

    const uint32_t off_in_run = this->pos % this->d0_len;
    const uint32_t run        = this->pos / this->d0_len;
    const uint32_t d1_len     = (this->d1_len == 0) ? 1u : this->d1_len;
    const uint32_t run_in_grp = run % d1_len;
    const uint32_t grp        = run / d1_len;

    return this->base_addr
         + grp        * this->d1_stride
         + run_in_grp * this->d0_stride
         + off_in_run;
}

// Issue one beat of at most inner_port_bytes and return the latency the memory
// reported. The caller tracks the in-flight response.
int Dimc_HWPE_Streamer::issue_beat(int width, void* buf) {
    uint32_t base = this->walk_addr();

    if (this->is_done()) {
        return 1;
    }
    if (width <= 0) {
        return 0;
    }

    // One beat carries at most a port word. Bank-granularity over-fetch is not
    // modelled: L1 is addressed at byte granularity here.
    const uint32_t port_bytes = this->dimc->inner_port_bytes;
    int beat = (width < (int)port_bytes) ? width : (int)port_bytes;

    // A beat is one request at one address, so it must not straddle the end of
    // a contiguous run. Clamp it to what is left of the current run.
    uint32_t left = this->tot_len - this->pos;
    if (this->d0_len != 0) {
        const uint32_t left_in_run = this->d0_len - (this->pos % this->d0_len);
        if (left_in_run < left) left = left_in_run;
    }
    if ((uint32_t)beat > left) beat = (int)left;

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

    return (int)latency;
}

// Point macro m's kernel streamer at the kernel rows of dual blk_id for the tile product in
// context ctx: row_count rows of 128 bytes, a row stride apart.
void Dimc_HWPE::configure_kernel_stream(uint32_t blk_id, uint32_t m, int ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    this->inner_blocks[blk_id].weight_stream[m].configure(
        this->job_reg_ctx(ctx, blk_id ? DIMC_TP_KB_SRC_2 : DIMC_TP_KB_SRC),
        g.row_count * DIMC_MACRO_KB_EW, DIMC_MACRO_KB_EW, g.kb_row_stride, g.row_count, 0, 0, 0, 0);
}

// Point the dual's output streamer at output row `row` of the tile product in context ctx:
// one result per vector.
void Dimc_HWPE::configure_out_stream(uint32_t blk_id, uint32_t row, uint32_t ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    this->inner_blocks[blk_id].out_stream[0].configure(
        this->job_reg_ctx((int)ctx, blk_id ? DIMC_TP_DST_2 : DIMC_TP_DST) + row * g.out_row_stride,
        g.nb_vec * DIMC_OUT_SLOT_BYTES, 0, 0, 0, 0, 0, 0, 0);
}

void Dimc_Port::configure(uint32_t bandwidth)
{
    this->bandwidth_bytes = bandwidth ? bandwidth : 1;
    this->cursor_bytes    = 0;
}

void Dimc_Port::reset()
{
    this->cursor_bytes = 0;
}

int64_t Dimc_Port::busy_until() const
{
    return this->cursor_bytes / (int64_t)this->bandwidth_bytes;
}

void Dimc_Port::request(int64_t now, uint64_t bytes)
{
    // Reserve in bytes, report in cycles: a beat narrower than the port takes a
    // fraction of a cycle, so several of them can share one.
    int64_t start = now * (int64_t)this->bandwidth_bytes;
    if (this->cursor_bytes > start) start = this->cursor_bytes;
    this->cursor_bytes = start + (int64_t)bytes;
}
