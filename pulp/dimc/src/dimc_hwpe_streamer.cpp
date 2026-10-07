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
    this->pair_valid = false;
    this->pair_addr  = 0;
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
    this->pair_valid = false;

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

// tot_len is a byte count. The engine's beats sum to exactly it, so this is only
// the streamer's guard; the engine's own section counts end a program or a store.
bool Dimc_HWPE_Streamer::is_done() { return this->pos >= this->tot_len; }

bool Dimc_HWPE_Streamer::pair_ready() const {
    return !this->is_write && this->pair_valid && this->walk_addr() == this->pair_addr;
}

// Where the next beat reads or writes. `pos` counts bytes consumed, which is
// also the address offset while the walk is linear (d0_len == 0).
// A non-zero d0_len selects a strided walk: d0_len contiguous bytes, then a jump
// by d0_stride to the next run, and after d1_len runs a jump by d1_stride; the
// *_D0_* / *_D1_* job registers set it. Not the RTL's addressing:
// dual_DIMC/rtl/dimc_streamer.sv derives its strides from the matrix shape in
// dimc_config_t and has no stride registers.
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
// reported. The caller advances one beat per cycle and tracks the in-flight
// response itself, which is what lets several accesses overlap.
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

    // The second section of a port word already fetched: no request.
    if (!this->is_write && buf != NULL && this->pair_valid && base == this->pair_addr
        && beat == (int)sizeof(this->pair)) {
        std::memcpy(buf, this->pair, sizeof(this->pair));
        this->pair_valid = false;
        this->pos += (uint32_t)beat;
        this->tot_iters++;
        return NO_REQUEST;
    }
    // A read of a port word's first section fetches the whole word (one address per
    // request on the 512-bit port); the second section waits in `pair`.
    const bool whole_word = !this->is_write && buf != NULL && beat == (int)sizeof(this->pair)
        && (base & (2 * sizeof(this->pair) - 1)) == 0 && left >= 2 * sizeof(this->pair);
    uint8_t word[2 * sizeof(this->pair)];

    int64_t latency = 1;
    if (buf != NULL) {
        this->req->prepare();
        this->req->set_addr(base);
        this->req->set_data(whole_word ? word : (uint8_t *) buf);
        this->req->set_size(whole_word ? (int)sizeof(word) : beat);
        vp::IoReqStatus err = this->dimc->stream_mst.req(this->req);
        if (err != vp::IO_REQ_OK) {
            this->dimc->trace.fatal("Error while issuing a TCDM beat\n");
            return 0;
        }
        latency = (int64_t) this->req->get_latency();
        if (whole_word) {
            std::memcpy(buf, word, sizeof(this->pair));
            std::memcpy(this->pair, word + sizeof(this->pair), sizeof(this->pair));
            this->pair_addr  = base + (uint32_t)sizeof(this->pair);
            this->pair_valid = true;
        }
    }

    this->pos += (uint32_t)beat;
    this->tot_iters++;

    return (int)latency;
}

// ---- Stream address setup, per macro ----
uint32_t Dimc_HWPE::slot_offset(const JobGeom &g, uint32_t blk_id, uint8_t kind) const
{
    // One kernel block per dual, whichever macro runs the job; one feature block for all.
    return kind == DIMC_LOAD_KB ? blk_id * g.row_count * DIMC_MACRO_KB_EW : 0;
}

// Point macro m's input streamers at the operands of vector `run` of the job in context ctx,
// which is the job being filled, not necessarily the running one.
void Dimc_HWPE::configure_macro_streams(uint32_t blk_id, uint32_t m, int ctx, uint32_t run,
                                        bool kernel, bool feature)
{
    const JobGeom &g = this->job_geom[ctx];
    Dimc_InnerBlock &blk = this->inner_blocks[blk_id];
    if (kernel)
        blk.weight_stream[m].configure(
            this->job_reg_ctx(ctx, DIMC_HWPE_JOB_KB_SRC_ADDR)
                + this->slot_offset(g, blk_id, DIMC_LOAD_KB), g.row_count * DIMC_MACRO_KB_EW,
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D0_LENGTH),
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D0_STRIDE),
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D1_LENGTH),
            this->job_reg_ctx(ctx, DIMC_HWPE_KB_D1_STRIDE), 0, 0, 0);
    if (feature)
        blk.input_stream[m].configure(
            this->job_reg_ctx(ctx, DIMC_HWPE_JOB_FB_SRC_ADDR) + run * g.fb_vec_stride
                + this->slot_offset(g, blk_id, DIMC_LOAD_FB), DIMC_MACRO_FB_EW,
            this->job_reg_ctx(ctx, DIMC_HWPE_FB_D0_LENGTH),
            this->job_reg_ctx(ctx, DIMC_HWPE_FB_D0_STRIDE), 0, 0, 0, 0, 0);
}

// Point the dual's output streamer at output row `row` of the job in context ctx: NB_VEC
// results, one per vector.
void Dimc_HWPE::configure_out_stream(uint32_t blk_id, uint32_t row, uint32_t ctx)
{
    const JobGeom &g = this->job_geom[ctx];
    this->inner_blocks[blk_id].out_stream[0].configure(
        this->job_reg_ctx((int)ctx, DIMC_HWPE_JOB_DST_ADDR)
            + (blk_id * g.row_count + row) * g.out_row_stride,
        g.nb_vec * DIMC_OUT_SLOT_BYTES, 0, 0, 0, 0, 0, 0, 0);
}

void Dimc_OuterPort::configure(uint32_t bandwidth)
{
    this->bandwidth_bytes = bandwidth ? bandwidth : 1;
    this->cursor_bytes    = 0;
}

void Dimc_OuterPort::reset()
{
    this->cursor_bytes = 0;
}

int64_t Dimc_OuterPort::busy_until() const
{
    return this->cursor_bytes / (int64_t)this->bandwidth_bytes;
}

void Dimc_OuterPort::request(int64_t now, uint64_t bytes)
{
    // Reserve in bytes, report in cycles: a beat narrower than the port takes a
    // fraction of a cycle, so several of them can share one.
    int64_t start = now * (int64_t)this->bandwidth_bytes;
    if (this->cursor_bytes > start) start = this->cursor_bytes;
    this->cursor_bytes = start + (int64_t)bytes;
}
