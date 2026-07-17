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

#ifndef __DIMC_HPP__
#define __DIMC_HPP__

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>
#include <cstdint>
#include <vector>

#include <dimc_hwpe_archi.hpp>
#include <dimc_macro.hpp>

typedef uint64_t strobe_t;

// Standard HWPE controller states (mirrors redmule: IDLE / STARTING /
// COMPUTING / STORING / FINISHED). A job is offloaded via the acquire/commit
// protocol, then the FSM steps preload -> compute -> store as event-driven
// iterations (each iter returns a latency and the loop re-enqueues).
enum dimc_hwpe_state_t {
    DIMC_IDLE,
    DIMC_STARTING,
    DIMC_COMPUTING,
    DIMC_STORING,
    DIMC_FINISHED
};

class Dimc_HWPE;

class Dimc_HWPE_Streamer {
    public:
        Dimc_HWPE_Streamer(Dimc_HWPE* dimc, bool is_write);
        Dimc_HWPE_Streamer();
        int  iterate(void* buf, strobe_t strb);
        void configure(
                uint32_t base_addr,
                uint32_t tot_len,
                uint32_t d0_len,
                uint32_t d0_stride,
                uint32_t d1_len,
                uint32_t d1_stride,
                uint32_t d2_len,
                uint32_t d2_stride,
                uint32_t d3_stride
        );
        void set_base_addr(uint32_t addr);
        uint32_t get_base_addr();
        bool is_done();
        int rw_data(int width, void* buf, strobe_t strb);

    private:
        Dimc_HWPE*  dimc;
        vp::IoReq*  req;

        uint32_t    pos;
        uint32_t    tot_iters;
        uint32_t    d0_iters;
        uint32_t    d1_iters;
        uint32_t    d2_iters;

        uint32_t    base_addr;
        uint32_t    tot_len;
        uint32_t    d0_len;
        uint32_t    d0_stride;
        uint32_t    d1_len;
        uint32_t    d1_stride;
        uint32_t    d2_len;
        uint32_t    d2_stride;
        uint32_t    d3_stride;
        bool        is_write;
};

class Dimc_HWPE : public vp::Component {
    public:
        Dimc_HWPE(vp::ComponentConf &config);

        void reset(bool active);

        // HWPE RF
        uint32_t register_file[N_CFG_REGS];

        // Streamer master port (data path to L1/TCDM)
        vp::IoMaster stream_mst;

        // HWPE slave port (memory-mapped register interface)
        vp::IoSlave hwpe_slv;

        // Completion interrupt line (standard HWPE done_irq)
        vp::WireMaster<bool> irq;

        // Streamers (weight, input, out)
        Dimc_HWPE_Streamer weight_stream;
        Dimc_HWPE_Streamer input_stream;
        Dimc_HWPE_Streamer out_stream;

        // Macros
        std::vector<Dimc_Macro> macros;
        uint8_t sel_dimc;

        // Configuration
        uint32_t num_macros;
        // Streamer bandwidth: fixed hardware properties, set once from the
        // systree / gvrun --param (no per-trigger MMIO override).
        uint32_t stream_chunk_bytes;   // bytes per rw_data call (Layer 3)
        uint32_t stream_bank_bytes;    // L1 bandwidth bytes/cycle, 32 banks*4=128 (Layer 2)
        uint32_t stream_sync;          // per-call wrapper sync cycle (0/1)
        uint32_t stream_noc_lat;       // NoC round-trip latency per streamer burst (bandwidth-independent)
        uint32_t stream_min_bank;      // min transfer per bank = bank word width (bytes); over-fetch when finer
        uint32_t stream_prefetch;      // cross-trigger prefetch on/off (0=off, default). When on, the
                                       // first macro's load is hidden under the previous trigger's drain.
        // ---- OUTER BLOCK (outer double buffer) ----
        // An INNER BLOCK = "num_macro macros share one inner (L1) port, double-buffered".
        // An OUTER BLOCK = "num_block inner blocks share one outer (L2) port, double-
        // buffered against each other": while inner block b runs its whole block-job,
        // inner block b+1's data is filled from L2 into its L1. All default to today's
        // behavior (num_block=1 => no-op).
        uint32_t stream_num_block;     // 1 = single inner block (today); 2 = outer block
        uint32_t stream_l2_shared;     // 1 = blocks share ONE L2 port (serial fills, real L2 double buffer);
                                       // 0 = each block has its own L2 port (independent/parallel, no upper buffer)
        uint32_t stream_l1_depth;      // 1 = single L1 buffer per block; 2 = ping-pong L1 (a block's next
                                       // L2 fill overlaps its current compute)
        uint32_t stream_l2_bw;         // L2 port bytes/cycle (0 sentinel => same as l1bw / stream_bank_bytes)
        uint32_t stream_noc_l2;        // L2 burst latency (sentinel <0 => same as stream_noc_lat)
        uint64_t prev_drain_slack;     // streamer-idle tail (compute+OUT) left by the previous trigger;
                                       // reset by soft_clear so a single/independent trigger gets no prefetch.
        // Reuse auto-detect: the KB (weight) source address of the last loaded
        // job. A trigger whose KB address matches reuses the resident weights
        // (skips that load), like a real weight cache. 0xFFFFFFFF = none yet.
        uint32_t last_kb_src;

        // Traces
        vp::Trace trace;

        // Internal state
        vp::reg_32 state;

        // ---- Standard HWPE offload/context bookkeeping ----
        uint32_t running_job;   // id of the job currently executing (RUNNING_JOB reg)
        uint32_t next_job_id;   // id handed out by the next ACQUIRE
        uint32_t finished_jobs; // count of completed jobs (FINISHED reg)
        bool     job_running;   // a committed job is in flight

        // ---- Event-driven engine phase iterators (scheduler) ----
        // Each returns true when its phase is complete and writes the cycles
        // consumed by this step into *latency (mirrors redmule preload/compute/
        // store_iter). Phase 1: compute_iter carries the analytical makespan.
        bool preload_iter(int *latency);
        bool compute_iter(int *latency);
        bool store_iter(int *latency);

    private:
        static vp::IoReqStatus hwpe_slave(vp::Block *__this, vp::IoReq *req);

        static void fsm_start_handler(vp::Block *__this, vp::ClockEvent *event);
        static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
        static void fsm_end_handler(vp::Block *__this, vp::ClockEvent *event);

        void fsm_loop();
        int  fsm();

        vp::ClockEvent *fsm_start_event;
        vp::ClockEvent *fsm_event;
        vp::ClockEvent *fsm_end_event;
};

#endif
