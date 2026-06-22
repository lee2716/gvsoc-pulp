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
#include <dimc_fifo.hpp>

typedef uint64_t strobe_t;

enum dimc_hwpe_state_t {
    DIMC_IDLE,
    DIMC_WRITE_RF,
    DIMC_CONFIG,
    DIMC_EXEC,
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

        // Streamer master port
        vp::IoMaster stream_mst;

        // HWPE slave port
        vp::IoSlave hwpe_slv;

        // Streamers (weight, input, out)
        Dimc_HWPE_Streamer weight_stream;
        Dimc_HWPE_Streamer input_stream;
        Dimc_HWPE_Streamer out_stream;

        // FIFOs
        Dimc_Fifo<uint8_t> weight_fifo;
        Dimc_Fifo<uint8_t> input_fifo;
        Dimc_Fifo<uint8_t> out_fifo;

        // Macros
        std::vector<Dimc_Macro> macros;
        uint8_t sel_dimc;

        // Configuration
        uint32_t num_macros;
        uint32_t fifo_depth;
        uint32_t dimc_latency;

        // Traces
        vp::Trace trace;

        // Internal state
        vp::reg_32 state;

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
