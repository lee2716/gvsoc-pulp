#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>

#include <pcm.hpp>

Pcm_HWPE::Pcm_HWPE(vp::ComponentConf &config) : vp::Component(config)
{
    // HWPE slave port
    this->hwpe_slv.set_req_meth(&Pcm_HWPE::hwpe_slave);
    this->new_slave_port("hwpe_slv", &this->hwpe_slv);
    
    // Streamer master port
    this->new_master_port("stream_mst", &this->stream_mst);

    // Input & output streamers
    this->inp_stream = Pcm_HWPE_Streamer(this, false);
    this->out_stream = Pcm_HWPE_Streamer(this, true);

    // Engine
    this->engine = Pcm_HWPE_Engine(this);

    // Event handlers
    this->fsm_start_event = this->event_new(&Pcm_HWPE::fsm_start_handler);
    this->fsm_event = this->event_new(&Pcm_HWPE::fsm_handler);
    this->fsm_end_event = this->event_new(&Pcm_HWPE::fsm_end_handler);

    // Initial state of the controller FSM
    this->state.set(IDLE);

    // Traces
    this->traces.new_trace("trace", &this->trace);
}

void Pcm_HWPE::reset(bool active) {
    if (active) {
        for (uint32_t i=0; i<56; i++)
            this->register_file[i] = 0x0;
    }
}

vp::IoReqStatus Pcm_HWPE::hwpe_slave(vp::Block *__this, vp::IoReq *req){
    Pcm_HWPE *_this = (Pcm_HWPE *)__this;
    uint32_t address = req->get_addr();

    if (req->get_is_write()) {
        uint32_t data = *((uint32_t *) (req->get_data()));

        _this->trace.msg(vp::TraceLevel::DEBUG, "Write request, address: 0x%x\n", address);

        if (address>PCM_HWPE_CHK_STATE)
        {
            if (address > 0xDC)
            {
                _this->trace.fatal("Trying to access invalid address 0x%x\n", address);

                return vp::IO_REQ_INVALID;
            }
            
            _this->trace.msg(vp::TraceLevel::DEBUG, "Writing register %d\n", address >> 2);

            _this->register_file[(address >> 2)] = data;

            _this->trace.msg(vp::TraceLevel::DEBUG, "Wrote %x\n", _this->register_file[(address >> 2)]);
        } else {
            switch (address)
            {
            case PCM_HWPE_TRIG:
                _this->trace.msg(vp::TraceLevel::DEBUG, "Job triggered\n");
                _this->event_enqueue(_this->fsm_start_event, 1);
                break;
            default:
                // we manage only the trigger, the rest of the registers isn't relevant
                break;
            }
        }
    } else {
        _this->trace.msg(vp::TraceLevel::DEBUG, "Read request, address: 0x%x\n", address);

        *(uint32_t *)req->get_data() = _this->register_file[(address >> 2)];

        _this->trace.msg(vp::TraceLevel::DEBUG, "Read value: %x\n", *(uint32_t*)req->get_data());
    }

    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Pcm_HWPE(config);
}

