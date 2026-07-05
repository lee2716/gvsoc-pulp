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

#ifndef __DIMC_MACRO_HPP__
#define __DIMC_MACRO_HPP__

#include <cstdint>
#include <deque>

// DIMC macro design parameters
#define DIMC_MACRO_KB_LEN   32
#define DIMC_MACRO_KB_EW    128
#define DIMC_MACRO_FB_EW    128
// Pipeline depth: 4 cycles per spatz_DIMC.sv (1 result/cycle throughput after fill)
#define DIMC_MACRO_LATENCY  4

struct DimcPipeEntry {
    int32_t  psout;
    int      job_row;
    int      cycles_remaining;
};

// Compute mode
#define DIMC_COMPE_MEM      0
#define DIMC_COMPE_COMPUTE  1

// Ci precision mode
#define DIMC_CI_1BIT        0
#define DIMC_CI_2BIT        1
#define DIMC_CI_4BIT        2
#define DIMC_CI_8BIT        3

// Sign mode for INT8 (bit 1 = K_signed, bit 0 = F_signed)
#define DIMC_SIGN_UU        0
#define DIMC_SIGN_US        1
#define DIMC_SIGN_SU        2
#define DIMC_SIGN_SS        3


class Dimc_Macro {
    public:
        Dimc_Macro();

        void reset();

        // Memory mode (COMPE = 0)
        void write_row(int row, const uint8_t *src);
        void read_row(int row, uint8_t *dst) const;
        void write_fb(const uint8_t *src);

        // Compute mode (COMPE = 1)
        int32_t compute_PP(int row_sel);
        int32_t final_compute(int32_t bias);

        // Pipelined scheduling: issue is non-blocking unless pipeline is full;
        // tick advances the pipeline; has_ready/drain pop the front entry
        void issue(int row, int job_row, int32_t bias);
        void tick();
        bool can_accept() const;
        bool has_ready() const;
        DimcPipeEntry drain();

        // Runtime configuration
        uint8_t  compe      = DIMC_COMPE_COMPUTE;
        uint8_t  ci         = DIMC_CI_8BIT;
        uint8_t  sign_mode  = DIMC_SIGN_UU;
        uint8_t  mct        = 0;
        int32_t  psin       = 0;

        // Buffers
        uint8_t  KB[DIMC_MACRO_KB_LEN][DIMC_MACRO_KB_EW];
        uint8_t  FB[DIMC_MACRO_FB_EW];

        // Outputs
        int32_t  psout = 0;
        uint8_t  sout  = 0;

        // Pipeline state: in-flight entries waiting to drain
        std::deque<DimcPipeEntry> pipe;
        bool     kb_ready         = false;
        bool     fb_ready         = false;
};

#endif
