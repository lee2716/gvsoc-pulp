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
    this->sout      = 0;
}

void Dimc_Macro::write_row(int row, const uint8_t *src)
{
    if (row < 0 || row >= DIMC_MACRO_KB_LEN) return;
    std::memcpy(this->KB[row], src, DIMC_MACRO_KB_EW);
}

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
        break;
    }
    case DIMC_CI_4BIT: {
        for (uint32_t i = 0; i < valid_bytes; i++) {
            uint8_t k = this->KB[row_sel][i];
            uint8_t f = this->FB[i];
            comp += ( k        & 0xF) * ( f        & 0xF)
                  + ((k >> 4) & 0xF) * ((f >> 4) & 0xF);
        }
        break;
    }
    case DIMC_CI_8BIT:
    default: {
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

    this->psout = comp;
    return comp;
}

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
