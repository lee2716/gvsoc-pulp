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

    // Configuration of the weight (KB) input streamer
    _this->weight_stream.configure(
        _this->register_file[DIMC_HWPE_JOB_KB_SRC_ADDR >> 2],   // base_addr
        _this->register_file[DIMC_HWPE_KB_TOTAL_LENGTH >> 2],   // tot_len
        _this->register_file[DIMC_HWPE_KB_D0_LENGTH    >> 2],   // d0_len
        _this->register_file[DIMC_HWPE_KB_D0_STRIDE    >> 2],   // d0_stride
        _this->register_file[DIMC_HWPE_KB_D1_LENGTH    >> 2],   // d1_len
        _this->register_file[DIMC_HWPE_KB_D1_STRIDE    >> 2],   // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the feature (FB) input streamer
    _this->input_stream.configure(
        _this->register_file[DIMC_HWPE_JOB_FB_SRC_ADDR >> 2],   // base_addr
        _this->register_file[DIMC_HWPE_FB_TOTAL_LENGTH >> 2],   // tot_len
        _this->register_file[DIMC_HWPE_FB_D0_LENGTH    >> 2],   // d0_len
        _this->register_file[DIMC_HWPE_FB_D0_STRIDE    >> 2],   // d0_stride
        0,                                                      // d1_len
        0,                                                      // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the output streamer
    _this->out_stream.configure(
        _this->register_file[DIMC_HWPE_JOB_DST_ADDR     >> 2],  // base_addr
        _this->register_file[DIMC_HWPE_OUT_TOTAL_LENGTH >> 2],  // tot_len
        _this->register_file[DIMC_HWPE_OUT_D0_LENGTH    >> 2],  // d0_len
        _this->register_file[DIMC_HWPE_OUT_D0_STRIDE    >> 2],  // d0_stride
        0,                                                      // d1_len
        0,                                                      // d1_stride
        0,                                                      // d2_len
        0,                                                      // d2_stride
        0                                                       // d3_stride
    );

    _this->state.set(DIMC_WRITE_RF);
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
    _this->register_file[DIMC_HWPE_STATUS >> 2] = 0x1;
    _this->trace.msg(vp::TraceLevel::WARNING, "DIMC job done, STATUS=1\n");
}

void Dimc_HWPE::fsm_loop()
{
    uint32_t latency = 0;

    do {
        latency = this->fsm();
    } while (latency == 0 && state.get() != DIMC_FINISHED);

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
    case DIMC_WRITE_RF:
        next_state = DIMC_CONFIG;
        break;

    case DIMC_CONFIG: {
        uint8_t compe     = (uint8_t)(this->register_file[DIMC_HWPE_COMPE     >> 2] & 0x1);
        uint8_t ci        = (uint8_t)(this->register_file[DIMC_HWPE_CFG_CI    >> 2] & 0x3);
        uint8_t sign_mode = (uint8_t)(this->register_file[DIMC_HWPE_SIGN_MODE >> 2] & 0x3);
        uint8_t mct       = (uint8_t)(this->register_file[DIMC_HWPE_MCT       >> 2] & 0xFF);
        this->sel_dimc    = (uint8_t)(this->register_file[DIMC_HWPE_SEL_DIMC  >> 2] & 0xFF);
        for (auto &m : this->macros) {
            m.compe     = compe;
            m.ci        = ci;
            m.sign_mode = sign_mode;
            m.mct       = mct;
        }
        // Streamer bandwidth (chunk / l1bw / sync) is a fixed hardware property,
        // set once from the systree / gvrun --param (this->stream_*), not per
        // trigger. Nothing to read here.
        next_state = DIMC_EXEC;
        break;
    }

    case DIMC_EXEC: {
        uint32_t num_active = this->register_file[DIMC_HWPE_NUM_MACROS  >> 2];
        if (num_active == 0 || num_active > this->num_macros) num_active = this->num_macros;
        uint32_t row_base   = this->register_file[DIMC_HWPE_ROW_SEL_BASE >> 2];
        uint32_t row_count  = this->register_file[DIMC_HWPE_ROW_COUNT    >> 2];
        if (row_count == 0) row_count = 1;
        if (row_count > DIMC_MACRO_KB_LEN) row_count = DIMC_MACRO_KB_LEN;
        int32_t  bias       = (int32_t)this->register_file[DIMC_HWPE_CFG_BIAS >> 2];
        int32_t  psin0      = (int32_t)this->register_file[DIMC_HWPE_PSIN     >> 2];

        const uint32_t out_w = 4;
        uint8_t  buf[DIMC_MACRO_KB_EW];
        uint8_t  out_buf[DIMC_MACRO_KB_LEN * out_w];
        const uint32_t compute_cyc = row_count + DIMC_MACRO_LATENCY;   // 36 @ row=32

        // Double-buffer: each active macro processes its OWN matvec (own kernel +
        // own feature + own output, NO broadcast). One shared streamer (single L1
        // port) serialises all traffic, N compute pipelines run in parallel.
        //
        // For real load/compute overlap the schedule front-loads ALL loads on the
        // streamer (loads have priority), each compute starts when its own load is
        // done, and the OUT writebacks are drained afterwards (results are held in
        // the output FIFO). This lets macro m+1's load run DURING macro m's compute
        // instead of waiting behind macro m's OUT.
        // Reuse auto-detect (weight-stationary): the KERNEL (weights) is the big
        // 32-row load. If this trigger's KB source address matches the previously
        // loaded one, the weights are still resident in the IMC array, so that
        // load is skipped -- exactly like a real weight cache. A soft_clear or a
        // different address forces a reload.
        //
        // The FEATURE (activations) is ALWAYS streamed fresh: a new matvec means
        // new inputs. Keeping the feature load flowing is what keeps the double
        // buffer alive during weight reuse -- each macro's compute overlaps the
        // NEXT macro's feature load, so reuse converts the pipeline from
        // weight-load-bound into feature-stream / compute bound instead of
        // collapsing it (which would happen if we skipped all loads).
        uint32_t kb_src  = this->register_file[DIMC_HWPE_JOB_KB_SRC_ADDR >> 2];
        bool     skip_kb = (kb_src == this->last_kb_src);
        bool     skip_fb = false;   // activations always stream (double buffer stays alive)
        this->last_kb_src = kb_src;

        std::vector<uint32_t> load_lat(num_active, 0);
        std::vector<uint32_t> out_lat (num_active, 0);

        for (uint32_t m = 0; m < num_active; m++) {
            // --- Load THIS macro's own kernel (no broadcast) ---
            // width clamped to the row boundary so a chunk wider than the row does
            // not overrun the buffer (chunk >= row => one row per call).
            uint32_t kb_lat = 0;
            if (!skip_kb) {
                for (uint32_t r = 0; r < row_count; r++) {
                    for (uint32_t off = 0; off < DIMC_MACRO_KB_EW; off += this->stream_chunk_bytes) {
                        uint32_t w = DIMC_MACRO_KB_EW - off;
                        if (w > this->stream_chunk_bytes) w = this->stream_chunk_bytes;
                        kb_lat += this->weight_stream.rw_data((int)w, (void *)(buf + off), (strobe_t)-1);
                    }
                    uint32_t row_idx = (row_base + r) % DIMC_MACRO_KB_LEN;
                    this->macros[m].write_row((int)row_idx, buf);   // ★ only macro m
                }
            }
            // else: macro[m].KB keeps the resident kernel from a previous trigger.

            // --- Load THIS macro's own feature ---
            uint32_t fb_lat = 0;
            if (!skip_fb) {
                for (uint32_t off = 0; off < DIMC_MACRO_FB_EW; off += this->stream_chunk_bytes) {
                    uint32_t w = DIMC_MACRO_FB_EW - off;
                    if (w > this->stream_chunk_bytes) w = this->stream_chunk_bytes;
                    fb_lat += this->input_stream.rw_data((int)w, (void *)(buf + off), (strobe_t)-1);
                }
                this->macros[m].write_fb(buf);                  // ★ only macro m
            }
            // else: macro[m].FB keeps the resident feature from a previous trigger.
            this->macros[m].kb_ready = true;
            this->macros[m].fb_ready = true;
            this->macros[m].pipe.clear();
            this->macros[m].psin     = psin0;

            // --- Compute THIS macro's matvec through its 4-deep pipeline ---
            uint32_t rows_issued = 0, rows_done = 0, cyc = 0;
            uint32_t cap = row_count + DIMC_MACRO_LATENCY * 4 + 16;
            while (rows_done < row_count && cyc < cap) {
                while (this->macros[m].has_ready()) {
                    DimcPipeEntry e = this->macros[m].drain();
                    uint32_t psout_u = (uint32_t)e.psout;
                    std::memcpy(out_buf + e.job_row * out_w, &psout_u, out_w);
                    rows_done++;
                }
                if (rows_issued < row_count && this->macros[m].can_accept()) {
                    uint32_t row_idx = (row_base + rows_issued) % DIMC_MACRO_KB_LEN;
                    this->macros[m].issue((int)row_idx, (int)rows_issued, bias);
                    rows_issued++;
                }
                this->macros[m].tick();
                cyc++;
            }

            // --- Write THIS macro's own output ---
            uint32_t o_lat     = 0;
            uint32_t out_total = row_count * out_w, off = 0;
            while (off < out_total) {
                uint32_t chunk = (out_total - off >= this->stream_chunk_bytes)
                                     ? this->stream_chunk_bytes : (out_total - off);
                o_lat += this->out_stream.rw_data(chunk, (void *)(out_buf + off), (strobe_t)-1);
                off += this->stream_chunk_bytes;
            }

            // One NoC round-trip latency per streamer burst (bandwidth-independent):
            // the input-load burst (weights+features) and the output-store burst
            // each pay it once at the head, then the transfers pipeline behind it.
            uint32_t noc = this->stream_noc_lat;
            load_lat[m] = noc + kb_lat + fb_lat;   // default noc=1 -> 1+32+1 = 34 (non-reuse)
            out_lat[m]  = noc + o_lat;
        }

        // ---- Overlapped timeline accounting ----
        // 1) Loads back-to-back on the shared streamer (single L1 port). This is
        //    the throughput bottleneck: 34 cycles of data load per macro.
        // 2) The N DIMC macros compute IN PARALLEL (independent in-memory-compute
        //    arrays, not sel-muxed): compute[m] starts as soon as its OWN load is
        //    done and runs concurrently with the other macros' computes, hidden
        //    under the following macros' loads (classic double buffering).
        // 3) OUTs drain on the streamer once all loads are done and each compute
        //    is complete (results buffered in the output FIFO meanwhile).
        uint64_t load_done = 0;
        uint64_t finish    = 0;
        std::vector<uint64_t> compute_done(num_active, 0);
        for (uint32_t m = 0; m < num_active; m++) {
            load_done      += load_lat[m];                  // serial loads (bottleneck)
            compute_done[m] = load_done + compute_cyc;      // parallel compute, overlaps next loads
        }
        uint64_t out_free = load_done;                       // streamer free after last load
        for (uint32_t m = 0; m < num_active; m++) {
            uint64_t out_start = std::max(out_free, compute_done[m]);
            out_free = out_start + out_lat[m];
            finish   = std::max(finish, out_free);
            this->trace.msg(vp::TraceLevel::WARNING,
                "macro[%u]: load=%u compute=%u OUT=%u compute_done=%lu out_done=%lu\n",
                m, load_lat[m], compute_cyc, out_lat[m],
                (unsigned long)compute_done[m], (unsigned long)out_free);
        }

        latency += (int)finish;
        this->trace.msg(vp::TraceLevel::WARNING,
            "DIMC double-buffer: num_active=%u row_count=%u chunk=%u l1bw=%u total_latency=%lu\n",
            num_active, row_count, this->stream_chunk_bytes, this->stream_bank_bytes,
            (unsigned long)finish);

        this->register_file[DIMC_HWPE_FIN_JOBS >> 2] = 1;
        next_state = DIMC_FINISHED;
        break;
    }

    case DIMC_FINISHED:
        break;

    default:
        this->trace.fatal("DIMC HWPE FSM: UNKNOWN STATE (%d)!\n", this->state.get());
    }

    this->state.set(next_state);
    return latency;
}
