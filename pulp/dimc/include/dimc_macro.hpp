#ifndef __DIMC_MACRO_HPP__
#define __DIMC_MACRO_HPP__

#include <cstdint>

// DIMC macro design parameters (ARCHYTAS PDF)
#define DIMC_MACRO_KB_LEN   32
#define DIMC_MACRO_KB_EW    128
#define DIMC_MACRO_FB_EW    128
#define DIMC_MACRO_LATENCY  5

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

        // Double-buffer ping-pong scheduling
        void issue(int row, int32_t bias);
        void tick();
        bool can_accept() const;

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

        // Ping-pong pipeline state
        bool     busy             = false;
        bool     kb_ready         = false;
        bool     fb_ready         = false;
        bool     result_ready     = false;
        int      row_assigned     = -1;
        int      cycles_remaining = 0;
};

#endif
