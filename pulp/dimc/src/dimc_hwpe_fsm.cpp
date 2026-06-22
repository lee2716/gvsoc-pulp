#include <dimc.hpp>
#include <cstring>
#include <algorithm>

void Dimc_HWPE::fsm_start_handler(vp::Block *__this, vp::ClockEvent *event)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;

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
    _this->trace.msg(vp::TraceLevel::DEBUG, "DIMC job done, STATUS=1\n");
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

        uint8_t buf[DIMC_MACRO_KB_EW];

        // Load KB rows from L1; broadcast each row to all active macros' KB
        for (uint32_t r = 0; r < row_count; r++) {
            for (uint32_t off = 0; off < DIMC_MACRO_KB_EW; off += 64) {
                int l = this->weight_stream.rw_data(64, (void *)(buf + off), (strobe_t)-1);
                latency += l;
            }
            uint32_t row_idx = (row_base + r) % DIMC_MACRO_KB_LEN;
            for (uint32_t m = 0; m < num_active; m++) {
                this->macros[m].write_row((int)row_idx, buf);
            }
        }

        // Load FB once; broadcast to all active macros' FB
        for (uint32_t off = 0; off < DIMC_MACRO_FB_EW; off += 64) {
            int l_fb = this->input_stream.rw_data(64, (void *)(buf + off), (strobe_t)-1);
            latency += l_fb;
        }
        for (uint32_t m = 0; m < num_active; m++) {
            this->macros[m].write_fb(buf);
        }

        // Compute: for each row, cascade across macros; select sel_dimc output
        const uint32_t out_w = 4;
        uint8_t  out_buf[DIMC_MACRO_KB_LEN * out_w];

        for (uint32_t r = 0; r < row_count; r++) {
            uint32_t row_idx = (row_base + r) % DIMC_MACRO_KB_LEN;
            int32_t  cur_psin = psin0;
            for (uint32_t m = 0; m < num_active; m++) {
                this->macros[m].psin = cur_psin;
                this->macros[m].compute_PP((int)row_idx);
                this->macros[m].final_compute(bias);
                cur_psin = this->macros[m].psout;
            }
            uint32_t sel = this->sel_dimc;
            if (sel >= num_active) sel = num_active - 1;
            uint32_t psout_u = (uint32_t)this->macros[sel].psout;
            std::memcpy(out_buf + r * out_w, &psout_u, out_w);
            latency += DIMC_MACRO_LATENCY;
        }

        // Stream the results back to L1 in 64-byte chunks
        uint32_t out_total = row_count * out_w;
        uint32_t off = 0;
        while (off < out_total) {
            uint32_t chunk = (out_total - off >= 64) ? 64 : (out_total - off);
            int l_out = this->out_stream.rw_data(chunk, (void *)(out_buf + off), (strobe_t)-1);
            latency += l_out;
            off += 64;
        }

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
