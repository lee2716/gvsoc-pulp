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

    this->ci        = DIMC_CI_8BIT;
    this->sign_8b   = DIMC_SIGN_UU;
    this->compute_mask = 0;
    this->psin_scalar = 0;
    this->psin_rows   = 0;
    this->psout     = 0;
    for (int s = 0; s < 2; s++)
        for (int r = 0; r < DIMC_MACRO_KB_LEN; r++) this->psin_buf_set[s][r] = 0;
    this->psin_sel = 0;
    this->sout      = 0;

    this->kb_ready = false;
    this->fb_ready = false;
    this->pipe.clear();
}

bool Dimc_Macro::can_accept() const
{
    // The RTL pipeline never back-pressures: one trigger per cycle always fits.
    return this->kb_ready && this->fb_ready
        && (int)this->pipe.size() < DIMC_MACRO_LATENCY + DIMC_OUT_FIFO_DELAY;
}

void Dimc_Macro::issue(int row, int job_row, int set, int run)
{
    if (!this->can_accept()) return;

    this->compute_PP(row);
    this->final_compute();
    this->pipe.push_back({this->psout, job_row, DIMC_MACRO_LATENCY + DIMC_OUT_FIFO_DELAY, set, run});
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

// The COMPE=0 memory-mode read-back path. No caller: the compute pipeline
// never uses it. Kept for register/behaviour fidelity.
void Dimc_Macro::read_row(int row, uint8_t *dst) const
{
    if (row < 0 || row >= DIMC_MACRO_KB_LEN) return;
    std::memcpy(dst, this->KB[row], DIMC_MACRO_KB_EW);
}

void Dimc_Macro::write_fb(const uint8_t *src)
{
    std::memcpy(this->FB, src, DIMC_MACRO_FB_EW);
}

// One 32-bit partial sum, stored against the row it belongs to. compute_PP()
// picks it up when that row is selected, and only while PSIN_EN is set.
void Dimc_Macro::write_psin_row(int row, const uint8_t *src, int set)
{
    if (row < 0 || row >= DIMC_MACRO_KB_LEN) return;
    std::memcpy(&this->psin_buf_set[set & 1][row], src, 4);
}

// Bit-level model of the macro datapath. Stage 1 zeroes every bit i >= 1024 - compute_mask
// of both operands, so an element straddling that boundary keeps its low bits; stage 2
// sums per MODE; stage 3 adds ADDIN. Bit i of the row is bit (i % 8) of byte i / 8.
int32_t Dimc_Macro::compute_PP(int row_sel)
{
    uint32_t valid_bits = 1024u - ((uint32_t)this->compute_mask & 0x3FFu);
    uint8_t k[DIMC_MACRO_KB_EW], f[DIMC_MACRO_FB_EW];
    for (uint32_t i = 0; i < DIMC_MACRO_KB_EW; i++) {
        uint32_t keep = valid_bits <= 8 * i ? 0 : valid_bits - 8 * i;
        uint8_t  m    = keep >= 8 ? 0xFF : (uint8_t)((1u << keep) - 1u);
        k[i] = this->KB[row_sel][i] & m;
        f[i] = this->FB[i] & m;
    }

    int32_t comp = 0;
    switch (this->ci) {
    case DIMC_CI_1BIT:
        // sum(kernel[i] * feature[i]): AND, not XNOR, as in the RTL.
        for (uint32_t i = 0; i < DIMC_MACRO_KB_EW; i++)
            comp += __builtin_popcount((unsigned)(k[i] & f[i]));
        break;
    case DIMC_CI_2BIT:
        for (uint32_t i = 0; i < DIMC_MACRO_KB_EW; i++)
            for (int s = 0; s < 4; s++)
                comp += ((k[i] >> (s * 2)) & 0x3) * ((f[i] >> (s * 2)) & 0x3);
        break;
    case DIMC_CI_4BIT:
        for (uint32_t i = 0; i < DIMC_MACRO_KB_EW; i++)
            comp += (k[i] & 0xF) * (f[i] & 0xF) + ((k[i] >> 4) & 0xF) * ((f[i] >> 4) & 0xF);
        break;
    case DIMC_CI_8BIT:
    default:
        // sign_8b bit 0: kernel signed, bit 1: feature signed. The sign bit is the
        // masked byte's bit 7, as in the RTL.
        for (uint32_t i = 0; i < DIMC_MACRO_KB_EW; i++) {
            int32_t kv = (this->sign_8b & 0x1) ? (int32_t)(int8_t)k[i] : (int32_t)k[i];
            int32_t fv = (this->sign_8b & 0x2) ? (int32_t)(int8_t)f[i] : (int32_t)f[i];
            comp += kv * fv;
        }
        break;
    }

    // ADDIN: the row's partial sum, or the job's constant when PSIN_EN is off.
    comp += this->psin_rows ? this->psin_buf_set[this->psin_sel][row_sel] : this->psin_scalar;

    this->psout = comp;
    return comp;
}

// SOUT: ReLU, then unsigned 8-bit saturation of PSOUT. Nothing reads SOUT, as in the RTL dual.
void Dimc_Macro::final_compute()
{
    if (this->psout < 0)        this->sout = 0;
    else if (this->psout > 255) this->sout = 255;
    else                        this->sout = (uint8_t)this->psout;
}
