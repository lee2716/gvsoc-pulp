#include <pcm.hpp>

void Pcm_HWPE::fsm_start_handler(vp::Block *__this, vp::ClockEvent *event) {
    Pcm_HWPE* _this = (Pcm_HWPE *)__this;

    // Configuration of the input streamer
    _this->trace.msg(vp::TraceLevel::DEBUG, "Configuring input stream...\n");
    _this->inp_stream.configure(
        _this->register_file[PCM_HWPE_JOB_SRC_ADDR >> 2],       // base_addr
        _this->register_file[PCM_HWPE_TOTAL_LENGTH >> 2],       // tot_len -- TO BE CHECKED
        _this->register_file[PCM_HWPE_D0_LENGTH >> 2],          // d0_len
        _this->register_file[PCM_HWPE_D0_STRIDE >> 2],          // d0_stride
        _this->register_file[PCM_HWPE_D1_LENGTH >> 2],          // d1_len
        _this->register_file[PCM_HWPE_D1_STRIDE >> 2],          // d1_stride
        0,                                                      // d2_len
        _this->register_file[PCM_HWPE_D2_STRIDE >> 2],          // d2_stride
        0                                                       // d3_stride
    );

    // Configuration of the output streamer
    _this->trace.msg(vp::TraceLevel::DEBUG, "Configuring output stream...\n");
    _this->out_stream.configure(
        _this->register_file[PCM_HWPE_JOB_DST_ADDR >> 2],       // base_addr
        _this->register_file[PCM_HWPE_OUT_TOTAL_LENGTH >> 2],   // tot_len
        _this->register_file[PCM_HWPE_OUT_D0_LENGTH >> 2],      // d0_len
        _this->register_file[PCM_HWPE_OUT_D0_STRIDE >> 2],      // d0_stride
        _this->register_file[PCM_HWPE_OUT_D1_LENGTH >> 2],      // d1_len
        _this->register_file[PCM_HWPE_OUT_D1_STRIDE >> 2],      // d1_stride
        0,                                                      // d2_len
        _this->register_file[PCM_HWPE_OUT_D2_STRIDE >> 2],      // d2_stride
        0                                                       // d3_stride
    );

    _this->state.set(WRITE_RF); // TO BE CHECKED!!
    _this->fsm_loop();
}

void Pcm_HWPE::fsm_handler(vp::Block *__this, vp::ClockEvent *event) {
    Pcm_HWPE* _this = (Pcm_HWPE *)__this;

    _this->fsm_loop();
}

void Pcm_HWPE::fsm_end_handler(vp::Block *__this, vp::ClockEvent *event){
    Pcm_HWPE* _this = (Pcm_HWPE *) __this;
    _this->state.set(IDLE);
}

void Pcm_HWPE::fsm_loop() {
    uint32_t latency = 0;

    do
    {
        latency = this->fsm();
    } while (latency == 0 && state.get() != FINISHED); // FINISHED not defined, figure out what it corresponds to

    if (state.get() == FINISHED && !this->fsm_end_event->is_enqueued())  // FINISHED not defined, figure out what it corresponds to
    {
        this->event_enqueue(this->fsm_end_event, latency);
    } else if (!this->fsm_event->is_enqueued())
    {
        this->event_enqueue(this->fsm_event, latency);
    }
    this->trace.msg(vp::TraceLevel::DEBUG, "FSM ended\n");
}

int Pcm_HWPE::fsm() {
    auto next_state = this->state.get();

    int latency = 0;
    switch (this->state.get())
    {
    case WRITE_RF:
        next_state = CONFIG;
        latency++;
        break;

    case CONFIG:
        uint32_t ctrl_value;
        ctrl_value = 0x00FFFF01; // Just for testing
        this->trace.msg(vp::TraceLevel::DEBUG, "Configuring AIMC core with control value 0x%x\n", ctrl_value);

        latency += this->engine.handle_config(this, CMD_REGISTER, &ctrl_value, true);
        
        next_state = FINISHED;
        break;
    case FINISHED:
        this->trace.msg(vp::TraceLevel::DEBUG, "Finished MVM computation(s)\n");
        break;
    default:
        this->trace.fatal("PCM HWPE FSM: UNKNOWN STATE (%d)!\n", this->state.get());
    }

    this->state.set(next_state);
    return latency;
}