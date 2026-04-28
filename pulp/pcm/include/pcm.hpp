#ifndef __PCM_HPP__
#define __PCM_HPP__

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>
#include <pthread.h>

#include <pcm_hwpe_archi.hpp>

// User Registers addresses
#define STATUS_REGISTER 0x41000
#define CMD_REGISTER    0x41004

// User Commands
#define COMPUTE_CMD     0x0
#define PARAMETER_CMD   0x1
#define ABORT_CMD       0x4

typedef uint64_t strobe_t;

// MVM threading configuration
#define NUM_THREADS 4

// PCM design parameters
#define PCM_NUM_LAYERS 8

// Structure to pass data to pthread worker threads
struct MvmThreadData {
    int32_t* full_prec_res;
    const int8_t* Xi_buf;
    const int8_t* weights;
    uint32_t j;
    uint32_t sec;
    uint32_t layer;
    uint32_t start_i;
    uint32_t end_i;
};

class Pcm_HWPE;

enum pcm_hwpe_state_t {
    IDLE,
    WRITE_RF,
    CONFIG,
    STREAMIN,
    EXEC_JOB,
    STREAMOUT,
    FINISHED
};

class Pcm_HWPE_Engine {
    public:
        Pcm_HWPE_Engine(Pcm_HWPE* pcm);
        Pcm_HWPE_Engine();
        void compute_mvm(Pcm_HWPE* pcm);
        int8_t Xi_buf[512];
        int handle_config(
            Pcm_HWPE    *pcm,
            uint32_t    ext_ctrl_addr,
            uint32_t    *ext_ctrl,
            bool        ext_write_ctrl
        );
        vp::IoReqStatus handle_compute(
            Pcm_HWPE        *pcm,
            uint8_t         ext_addr,
            bool            ext_write,
            bool            ext_data_in_shift,
            uint64_t        ext_data_in,
            uint64_t        ext_data_out,
            int             *latency
        );

    private:
        Pcm_HWPE*       pcm                 ;
        uint32_t        ext_ctrl_addr       ;
        uint32_t        *ext_ctrl           ;
        bool            ext_write_ctrl      ;
        uint32_t        ext_addr            ;
        bool            ext_write           ;
        bool            ext_data_in_shift   ;
        uint64_t        ext_data_in         ;
        uint64_t        ext_data_out        ; // 8x 8-bits elements: w.r.t. the modeled IP, we provide the outputs already quantized to 8-bits
        uint32_t        user_registers[2]   ;
        int8_t          Xi[512]             ;
        int8_t          weights[8*512*512]  ;
        int8_t          Yi[512]             ;
        uint32_t        mvm_latency         ;
};

class Pcm_HWPE_Streamer {
    public:
        Pcm_HWPE_Streamer(Pcm_HWPE* pcm, bool is_write);
        Pcm_HWPE_Streamer();
        int iterate(void* buf, strobe_t strb);
        void configure(
                uint32_t base_addr  ,
                uint32_t tot_len    ,
                uint32_t d0_len     ,
                uint32_t d0_stride  ,
                uint32_t d1_len     ,
                uint32_t d1_stride  ,
                uint32_t d2_len     ,
                uint32_t d2_stride  ,
                uint32_t d3_stride  
        );
        void set_base_addr(uint32_t addr);
        uint32_t get_base_addr();
        bool is_done();
        int rw_data(int width, void* buf, strobe_t strb);

    private:
        Pcm_HWPE*   pcm         ;
        vp::IoReq*  req         ;

        uint32_t    pos         ;
        uint32_t    tot_iters   ;
        uint32_t    d0_iters    ;
        uint32_t    d1_iters    ;
        uint32_t    d2_iters    ;

        uint32_t    base_addr   ;
        uint32_t    tot_len     ;
        uint32_t    d0_len      ;
        uint32_t    d0_stride   ;
        uint32_t    d1_len      ;
        uint32_t    d1_stride   ;
        uint32_t    d2_len      ;
        uint32_t    d2_stride   ;
        uint32_t    d3_stride   ;
        bool        is_write    ;
};

class Pcm_HWPE_Controller {
    public:
        Pcm_HWPE_Controller(Pcm_HWPE* pcm);
    private:
        Pcm_HWPE*           pcm     ;
        pcm_hwpe_state_t    state   ;
};

class Pcm_HWPE : public vp::Component
{

public:
    Pcm_HWPE(vp::ComponentConf &config);

    void reset(bool active);

    // HWPE RF
    uint32_t register_file[56];

    // Streamer master port
    vp::IoMaster stream_mst;

    // HWPE slave port
    vp::IoSlave hwpe_slv;

    // Streamers
    Pcm_HWPE_Streamer inp_stream;
    Pcm_HWPE_Streamer out_stream;

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
    int fsm();

    Pcm_HWPE_Engine engine;

    vp::ClockEvent *fsm_start_event;
    vp::ClockEvent *fsm_event;
    vp::ClockEvent *fsm_end_event;
};

#endif