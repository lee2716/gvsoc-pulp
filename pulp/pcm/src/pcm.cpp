#include <vp/vp.hpp>
#include <vp/itf/io.hpp>
#include <algorithm>
#include <stdio.h>

#include "pcm.hpp"

class Pcm : public vp::Component
{

public:
    Pcm(vp::ComponentConf &config);

private:
    static vp::IoReqStatus handle_req(vp::Block *__this, vp::IoReq *req);
    static vp::IoReqStatus handle_ctrl_req(vp::Block *__this, vp::IoReq *req);
    vp::IoReqStatus handle_compute(vp::Block *__this, vp::IoReq *req);

    vp::Trace trace;

    // EXT interfaces
    vp::IoSlave ext_ctrl_itf;
    vp::IoSlave ext_itf;

    // User registers
    uint32_t user_registers[2]; // 0 -> Status Register, 1 -> Command Register

    // Xi (inputs)
    int8_t Xi[512];
    
    // Weights
    int8_t weights[8*512*512];

    // Yi (results)
    int8_t Yi[512];
};

vp::IoReqStatus Pcm::handle_compute(vp::Block *__this, vp::IoReq *req){
    Pcm *_this = (Pcm *)__this;
    
    int32_t full_prec_res[512];
    //uint8_t sub_cmd;
    uint8_t layer;
    uint8_t active_sectors[4] = {0, 0, 0, 0};
    
    // Initialize accumulators
    for (uint32_t i=0; i<512; i++){
        //_this->Yi[i] = 0;
        std::fill(full_prec_res, full_prec_res+512, 0);
    }

    // Detect active sectors
    for (uint8_t sec=0; sec<4; sec++){
        active_sectors[sec] = (_this->user_registers[1]>>(8+sec*4)) & 0x0F;
        _this->trace.msg(vp::TraceLevel::DEBUG, "sector[%d] = %x\n", sec, active_sectors[sec]);
    }

    // Detect active layer
    layer = (_this->user_registers[2] & 0x000000FF);
    
    // Initialize Xi, just for first debugging - REMOVE
    for (uint32_t i=0; i<512; i++)
        _this->Xi[i] = 0x1;

    // Compute MVM
    for (uint32_t j=0; j<512; j++){
        for (uint32_t sec=0; sec<4; sec++){
            if (active_sectors[sec] != 0){
                for(uint32_t i=0; i<128; i++){
                    full_prec_res[j] += (int32_t)_this->Xi[i]*(int32_t)_this->weights[layer*512+512*sec*i+j];
                    //_this->trace.msg(vp::TraceLevel::DEBUG, "Intermediate Yi[%d] = %d\n", j , _this->Yi[j]);
                }
            }
        }
    }

    // Just for first debug purposes - REMOVE!!
    /* for (uint32_t i=0; i<512; i++){
        _this->trace.msg(vp::TraceLevel::DEBUG, "Yi[%d] = %d\n", i, full_prec_res[i]);
    } */

    // Cast full precision results to 8 bit
    for (uint32_t i=0; i<512; i++){
        _this->Yi[i] = std::max(-128, std::min(full_prec_res[i], 127));
    }

    _this->trace.msg(vp::TraceLevel::DEBUG, "Finished MVM computation\n");
    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::handle_ctrl_req(vp::Block *__this, vp::IoReq *req)
{
    Pcm *_this = (Pcm *)__this;
    uint8_t cmd;

    // R/W request
    _this->trace.msg(vp::TraceLevel::DEBUG, "Received CTRL request: address = %x, is_write = %d, size = %d\n", req->get_addr(), req->get_is_write(), req->get_size());
    if(!req->get_is_write() && req->get_addr() == 0 && req->get_size() == 4) {
        _this->trace.msg(vp::TraceLevel::DEBUG, "Read request at User Status Register, read value: %x\n", _this->user_registers[0]); // maybe in the future we can print the single fields
        *(uint32_t *)req->get_data() = _this->user_registers[0];
    } else if(req->get_is_write() && req->get_addr() == 4 && req->get_size() == 4) {
        _this->trace.msg(vp::TraceLevel::DEBUG, "Write request at Control Register, written value: %x\n", *(uint32_t *)req->get_data()); // maybe in the future we can print the single fields
        _this->user_registers[1] = *(uint32_t *)req->get_data();

        // Handle request for computation
        cmd = (_this->user_registers[1] & 0xF0000000) >> 24;
        if(cmd == COMPUTE_CMD){ // replace with case
            _this->trace.msg(vp::TraceLevel::DEBUG, "Requested MVM computation\n");
            return _this->handle_compute(_this, req);
        }

    } else {
        _this->trace.msg(vp::TraceLevel::DEBUG, "Invalid request\n");
        return vp::IO_REQ_INVALID;
    }

    return vp::IO_REQ_OK;
}

vp::IoReqStatus Pcm::handle_req(vp::Block *__this, vp::IoReq *req){

    Pcm *_this = (Pcm *)__this;
    int8_t Xi_buff[64];

    _this->trace.msg(vp::TraceLevel::DEBUG, "Received request: address = %x, is_write = %d, size = %d\n", req->get_addr(), req->get_is_write(), req->get_size());

    // R/W request, i.e. read Yis or write Xis
    if (!req->get_is_write() && req->get_size() == 1) {
        *(int8_t *)req->get_data() = _this->Yi[req->get_addr()];
        //req->set_data((uint8_t *)&(_this->Yi[req->get_addr()]));
        //_this->trace.msg(vp::TraceLevel::DEBUG, "req->get_data() = %d\n", _this->Yi[req->get_addr()]);

        return vp::IO_REQ_OK;
    } else if (req->get_is_write() && req->get_size() == 1) {
        _this->Xi[req->get_addr()] = *(int8_t *)(req->get_data());

        return vp::IO_REQ_OK;
    } else {
        return vp::IO_REQ_INVALID;
    }
    
    return vp::IO_REQ_OK;
}

Pcm::Pcm(vp::ComponentConf &config) : vp::Component(config)
{
    // EXT_CTRL interface
    this->ext_ctrl_itf.set_req_meth(&Pcm::handle_ctrl_req);
    this->new_slave_port("ext_ctrl", &this->ext_ctrl_itf);
    // EXT interface
    this->ext_itf.set_req_meth(&Pcm::handle_req);
    this->new_slave_port("ext", &this->ext_itf);

    // Traces
    this->traces.new_trace("trace", &this->trace);

    // Initialize weights
    js::Config *stim_file_conf = this->get_js_config()->get("stim_file");
    if (stim_file_conf != NULL)
    {
        string path = stim_file_conf->get_str();
        if (path != "")
        {
            trace.msg("Preloading Memory with stimuli file (path: %s)\n", path.c_str());

            FILE *file = fopen(path.c_str(), "rb");
            if (file == NULL)
            {
                this->trace.fatal("Unable to open stim file: %s, %s\n", path.c_str(), strerror(errno));
                return;
            }
            if (fread(this->weights, 1, 8*512*512, file) == 0)
            {
                this->trace.fatal("Failed to read stim file: %s, %s\n", path.c_str(), strerror(errno));
                return;
            }
        }
        else
        {
            this->trace.msg(vp::TraceLevel::DEBUG, "No path specified, preloading PCM weights with all ones\n");
            std::fill(this->weights, this->weights+(8*512*512), 1);
        }
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new Pcm(config);
}

