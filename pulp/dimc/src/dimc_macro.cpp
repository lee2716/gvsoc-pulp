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

#include <dimc_macro.hpp>
#include <cstring>

Dimc_Macro::Dimc_Macro()
{
    this->reset();
}

void Dimc_Macro::reset()
{
    for (int r = 0; r < DIMC_MACRO_KB_LEN; r++)
        for (int c = 0; c < DIMC_MACRO_KB_EW; c++)
            this->KB[r][c] = 0;
    for (int c = 0; c < DIMC_MACRO_FB_EW; c++)
        this->FB[c] = 0;

    this->compe     = DIMC_COMPE_COMPUTE;
    this->ci        = DIMC_CI_8BIT;
    this->sign_mode = DIMC_SIGN_UU;
    this->mct       = 0;
    this->psin      = 0;
    this->psout     = 0;
    this->accumulate = 0;
    for (int r = 0; r < DIMC_MACRO_KB_LEN; r++) this->accum[r] = 0;
    this->sout      = 0;

    this->kb_ready = false;
    this->fb_ready = false;
    this->pipe.clear();
}

bool Dimc_Macro::can_accept() const
{
    return this->kb_ready && this->fb_ready
        && (int)this->pipe.size() < DIMC_MACRO_LATENCY;
}

void Dimc_Macro::issue(int row, int job_row, int32_t bias)
{
    if (!this->can_accept()) return;

    this->compute_PP(row);
    this->final_compute(bias);
    this->pipe.push_back({this->psout, job_row, DIMC_MACRO_LATENCY});
}

void Dimc_Macro::tick()
{
    for (auto &e : this->pipe) {
        e.cycles_remaining--;
    }
}

bool Dimc_Macro::has_ready() const
{
    return !this->pipe.empty() && this->pipe.front().cycles_remaining <= 0;
}

DimcPipeEntry Dimc_Macro::drain()
{
    DimcPipeEntry e = this->pipe.front();
    this->pipe.pop_front();
    return e;
}

void Dimc_Macro::write_row(int row, const uint8_t *src)
{
    if (row < 0 || row >= DIMC_MACRO_KB_LEN) return;
    std::memcpy(this->KB[row], src, DIMC_MACRO_KB_EW);
}

// Dangling: no caller. This is the COMPE=0 memory-mode read-back path; the
// compute pipeline never uses it. Kept for register/behaviour fidelity.
void Dimc_Macro::read_row(int row, uint8_t *dst) const
{
    if (row < 0 || row >= DIMC_MACRO_KB_LEN) return;
    std::memcpy(dst, this->KB[row], DIMC_MACRO_KB_EW);
}

void Dimc_Macro::write_fb(const uint8_t *src)
{
    std::memcpy(this->FB, src, DIMC_MACRO_FB_EW);
}

int32_t Dimc_Macro::compute_PP(int row_sel)
{
    // Apply thermometric MCT mask: valid_bits = 1024 - MCT*4
    uint32_t valid_bits = 1024u - (uint32_t)this->mct * 4u;
    if (valid_bits > 1024u) valid_bits = 0;
    uint32_t valid_bytes = valid_bits / 8u;
    uint32_t tail_bits   = valid_bits & 7u;

    int32_t comp = 0;

    switch (this->ci) {
    case DIMC_CI_1BIT: {
        // 1-bit multiply = XNOR + popcount (bipolar encoding: bit 0 = -1, bit 1 = +1,
        // so the product is +1 exactly when the two bits AGREE). Confirmed 2026-07-18.
        // Alternative convention: AND is the literal unsigned {0,1} multiply.
        for (uint32_t i = 0; i < valid_bytes; i++) {
            uint8_t x = ~((uint8_t)(this->KB[row_sel][i] ^ this->FB[i]));
            for (int b = 0; b < 8; b++)
                comp += (x >> b) & 1;
        }
        if (tail_bits) {
            uint8_t x = ~((uint8_t)(this->KB[row_sel][valid_bytes] ^ this->FB[valid_bytes]));
            for (uint32_t b = 0; b < tail_bits; b++)
                comp += (x >> b) & 1;
        }
        break;
    }
    case DIMC_CI_2BIT: {
        for (uint32_t i = 0; i < valid_bytes; i++) {
            uint8_t k = this->KB[row_sel][i];
            uint8_t f = this->FB[i];
            for (int s = 0; s < 4; s++)
                comp += ((k >> (s*2)) & 0x3) * ((f >> (s*2)) & 0x3);
        }
        // Partial trailing byte: the MCT mask moves in 4-bit steps, so a masked
        // row can end mid-byte. Only WHOLE 2-bit elements inside it still count.
        if (tail_bits) {
            uint8_t k = this->KB[row_sel][valid_bytes];
            uint8_t f = this->FB[valid_bytes];
            for (uint32_t s = 0; s < tail_bits / 2u; s++)
                comp += ((k >> (s*2)) & 0x3) * ((f >> (s*2)) & 0x3);
        }
        break;
    }
    case DIMC_CI_4BIT: {
        for (uint32_t i = 0; i < valid_bytes; i++) {
            uint8_t k = this->KB[row_sel][i];
            uint8_t f = this->FB[i];
            comp += ( k        & 0xF) * ( f        & 0xF)
                  + ((k >> 4) & 0xF) * ((f >> 4) & 0xF);
        }
        // Partial trailing byte: only WHOLE 4-bit elements inside it count.
        if (tail_bits) {
            uint8_t k = this->KB[row_sel][valid_bytes];
            uint8_t f = this->FB[valid_bytes];
            for (uint32_t n = 0; n < tail_bits / 4u; n++)
                comp += ((k >> (n*4)) & 0xF) * ((f >> (n*4)) & 0xF);
        }
        break;
    }
    case DIMC_CI_8BIT:
    default: {
        // No tail handling needed: the mask steps in 4-bit units, so a partial
        // trailing byte can never hold a whole 8-bit element, so it is dropped.
        for (uint32_t i = 0; i < valid_bytes; i++) {
            int32_t k = (this->sign_mode & 0x2) ? (int32_t)(int8_t)this->KB[row_sel][i]
                                                : (int32_t)(uint8_t)this->KB[row_sel][i];
            int32_t f = (this->sign_mode & 0x1) ? (int32_t)(int8_t)this->FB[i]
                                                : (int32_t)(uint8_t)this->FB[i];
            comp += k * f;
        }
        break;
    }
    }

    comp += this->psin;

    // Partial-sum chain. accumulate=0 starts a fresh sum for this row, so no
    // explicit clear is needed between chains; accumulate=1 continues the one
    // the previous chunk left in accum[row_sel].
    if (this->accumulate) comp += this->accum[row_sel];
    this->accum[row_sel] = comp;

    this->psout = comp;
    return comp;
}

// DANGLING PATH: the value this returns is DISCARDED by issue(), and `sout` is
// never read by the FSM (the pipeline carries `psout`). Consequences:
//   * the `bias` argument NEVER reaches the 32-bit output the test reads;
//   * the ReLU + 8-bit saturation result (`sout`) is not wired out anywhere;
//   * there is no requantisation, so `sout` saturates for INT4/INT8 ranges.
// Confirmed as an unused path on 2026-07-18; left as-is.
int32_t Dimc_Macro::final_compute(int32_t bias)
{
    int32_t psum = this->psout + bias;

    // ReLU + 8-bit saturation (ARCHYTAS PDF: Sout = 8-bit port)
    if (psum < 0)
        this->sout = 0;
    else if (psum > 255)
        this->sout = 255;
    else
        this->sout = (uint8_t)psum;

    return psum;
}
