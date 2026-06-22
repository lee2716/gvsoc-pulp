#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>

#include <dimc.hpp>

Dimc_HWPE::Dimc_HWPE(vp::ComponentConf &config) : vp::Component(config)
{
    // Configuration from systree
    this->num_macros   = (uint32_t)this->get_js_config()->get_child_int("num_macros");
    this->fifo_depth   = (uint32_t)this->get_js_config()->get_child_int("fifo_depth");
    this->dimc_latency = (uint32_t)this->get_js_config()->get_child_int("dimc_latency");
    if (this->num_macros == 0) this->num_macros = 2;
    if (this->fifo_depth == 0) this->fifo_depth = 8;

    // HWPE slave port
    this->hwpe_slv.set_req_meth(&Dimc_HWPE::hwpe_slave);
    this->new_slave_port("hwpe_slv", &this->hwpe_slv);

    // Streamer master port
    this->new_master_port("stream_mst", &this->stream_mst);

    // Streamers
    this->weight_stream = Dimc_HWPE_Streamer(this, false);
    this->input_stream  = Dimc_HWPE_Streamer(this, false);
    this->out_stream    = Dimc_HWPE_Streamer(this, true);

    // FIFOs
    this->weight_fifo.set_depth(this->fifo_depth);
    this->input_fifo.set_depth(this->fifo_depth);
    this->out_fifo.set_depth(this->fifo_depth);

    // Macros
    this->macros.resize(this->num_macros);

    // Event handlers
    this->fsm_start_event = this->event_new(&Dimc_HWPE::fsm_start_handler);
    this->fsm_event       = this->event_new(&Dimc_HWPE::fsm_handler);
    this->fsm_end_event   = this->event_new(&Dimc_HWPE::fsm_end_handler);

    // Initial state of the controller FSM
    this->sel_dimc = 0;
    this->state.set(DIMC_IDLE);

    // Traces
    this->traces.new_trace("trace", &this->trace);
}

void Dimc_HWPE::reset(bool active)
{
    if (active) {
        for (uint32_t i = 0; i < N_CFG_REGS; i++) {
            this->register_file[i] = 0x0;
        }
        for (auto &m : this->macros) m.reset();
        this->weight_fifo.reset();
        this->input_fifo.reset();
        this->out_fifo.reset();
        this->sel_dimc = 0;
        this->state.set(DIMC_IDLE);
    }
}

vp::IoReqStatus Dimc_HWPE::hwpe_slave(vp::Block *__this, vp::IoReq *req)
{
    Dimc_HWPE *_this = (Dimc_HWPE *)__this;
    uint32_t address = req->get_addr();

    if (req->get_is_write()) {
        uint32_t data = *((uint32_t *) (req->get_data()));

        _this->trace.msg(vp::TraceLevel::DEBUG, "Write request, address: 0x%x\n", address);

        if (address > DIMC_HWPE_CHK_STATE) {
            if (address > DIMC_HWPE_REG_MAX) {
                _this->trace.fatal("Trying to access invalid address 0x%x\n", address);
                return vp::IO_REQ_INVALID;
            }
            _this->register_file[(address >> 2)] = data;
        } else {
            switch (address) {
            case DIMC_HWPE_TRIG:
                _this->event_enqueue(_this->fsm_start_event, 1);
                break;
            case DIMC_HWPE_SOFT_CLEAR:
                for (auto &m : _this->macros) m.reset();
                _this->weight_fifo.reset();
                _this->input_fifo.reset();
                _this->out_fifo.reset();
                _this->sel_dimc = 0;
                for (uint32_t i = 0; i < N_CFG_REGS; i++)
                    _this->register_file[i] = 0x0;
                _this->state.set(DIMC_IDLE);
                break;
            default:
                break;
            }
        }
    } else {
        if (address > DIMC_HWPE_REG_MAX) {
            _this->trace.fatal("Trying to access invalid address 0x%x\n", address);
            return vp::IO_REQ_INVALID;
        }
        *(uint32_t *)req->get_data() = _this->register_file[(address >> 2)];
    }

    return vp::IO_REQ_OK;
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Dimc_HWPE(config);
}
