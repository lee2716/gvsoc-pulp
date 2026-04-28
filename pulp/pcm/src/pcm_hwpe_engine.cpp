#include <fstream>
#include <sstream>
#include <cstdint>
#include <pcm.hpp>

// Thread worker function for computing MVM innermost loop
void* mvm_thread_worker(void* arg) {
    MvmThreadData* data = (MvmThreadData*)arg;
    
    for (uint32_t i = data->start_i; i < data->end_i; i++) {
        data->full_prec_res[i] += (int32_t)data->Xi_buf[data->j + data->sec * 128] * 
                                  (int32_t)data->weights[512 * 512 * data->layer + i + 512 * (data->j + data->sec * 128)];
    }
    
    pthread_exit(NULL);
}

Pcm_HWPE_Engine::Pcm_HWPE_Engine(Pcm_HWPE* pcm){
    this->pcm = pcm;

    this->ext_ctrl_addr = 0;
    this->ext_ctrl = NULL;
    this->ext_write_ctrl = false;
    this->ext_addr = 0;
    this->ext_write = false;
    this->ext_data_in_shift = false;
    this->ext_data_in = 0;
    this->ext_data_out = 0;
    this->mvm_latency = this->pcm->get_js_config()->get_child_int("mvm_latency");

    for(uint32_t i=0; i<512; i++)
        this->Xi_buf[i] = 0;
}

Pcm_HWPE_Engine::Pcm_HWPE_Engine() {
	this->pcm = (Pcm_HWPE *) NULL;
}

void Pcm_HWPE_Engine::compute_mvm(Pcm_HWPE *pcm) {
    this->pcm = pcm;

    int32_t full_prec_res[512];
    uint8_t layer;
    uint8_t active_sectors[4] = {0, 0, 0, 0};
    
    // Initialize accumulators
    for (uint32_t i=0; i<512; i++){
        std::fill(full_prec_res, full_prec_res+512, 0);
    }

    // Detect active sectors
    active_sectors[0] = (this->pcm->register_file[PCM_HWPE_ACT_SECT_0>>2]) & 0x0F;
    active_sectors[1] = (this->pcm->register_file[PCM_HWPE_ACT_SECT_1>>2]) & 0x0F;
    active_sectors[2] = (this->pcm->register_file[PCM_HWPE_ACT_SECT_2>>2]) & 0x0F;
    active_sectors[3] = (this->pcm->register_file[PCM_HWPE_ACT_SECT_3>>2]) & 0x0F;
    
    for (uint32_t sec=0; sec<4; sec++){
        pcm->trace.msg(vp::TraceLevel::DEBUG, "sector[%d] = %x\n", sec, active_sectors[sec]);
    }

    // Detect active layer
    layer = this->pcm->register_file[PCM_HWPE_ACT_LAYER>>2] & 0xFF;

    // Compute MVM - only signed MVM implemented
    for (uint32_t sec=0; sec<4; sec++){
        if (active_sectors[sec] != 0){
            pcm->trace.msg(vp::TraceLevel::DEBUG, "sector %d is active\n", sec);
            for (uint32_t j=0; j<128; j++){
                // Parallelize the innermost loop with pthreads
                pthread_t threads[NUM_THREADS];
                MvmThreadData thread_data[NUM_THREADS];
                uint32_t items_per_thread = 512 / NUM_THREADS;
                
                // Create threads
                for (int t = 0; t < NUM_THREADS; t++) {
                    thread_data[t].full_prec_res = full_prec_res;
                    thread_data[t].Xi_buf = this->Xi_buf;
                    thread_data[t].weights = this->weights;
                    thread_data[t].j = j;
                    thread_data[t].sec = sec;
                    thread_data[t].layer = layer;
                    thread_data[t].start_i = t * items_per_thread;
                    thread_data[t].end_i = (t == NUM_THREADS - 1) ? 512 : (t + 1) * items_per_thread;
                    
                    pthread_create(&threads[t], NULL, mvm_thread_worker, &thread_data[t]);
                }
                
                // Wait for all threads to complete
                for (int t = 0; t < NUM_THREADS; t++) {
                    pthread_join(threads[t], NULL);
                }
            }
        }
    }

    // Just for first debug purposes - REMOVE!!
    /* for (uint32_t i=0; i<512; i++){
        pcm->trace.msg(vp::TraceLevel::DEBUG, "full_prec_res[%d] = %d\n", i, full_prec_res[i]);
    } */

    // Cast full precision results to 8 bit 
    for (uint32_t i=0; i<512; i++){
        this->Yi[i] = std::max(-128, std::min(full_prec_res[i], 127));
    }

    pcm->trace.msg(vp::TraceLevel::DEBUG, "Finished MVM computation\n");
}

vp::IoReqStatus Pcm_HWPE_Engine::handle_compute(
    Pcm_HWPE        *pcm,
    uint8_t         ext_addr,
    bool            ext_write,
    bool            ext_data_in_shift,
    uint64_t        ext_data_in,
    uint64_t        ext_data_out,
    int             *latency
){     
    this->pcm = pcm;

    // Clear HWPE status register
    this->pcm->register_file[PCM_HWPE_STATUS>>2] = 0x0;

    this->pcm->trace.msg(vp::TraceLevel::DEBUG, "HWPE_STATUS = 0x%x\n", this->pcm->register_file[PCM_HWPE_STATUS>>2]);
    
    // Initial fill of the Xi buffer
    this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Filling Xi buffer\n");
    for (int8_t i = 0; i<8; i++)
        *latency += this->pcm->inp_stream.rw_data(64, (void *)(this->Xi_buf+i*64), -1);

    // Read Xi buffer - just for debugging
    /* for (int i = 0; i < 512; i++)
    {
        this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Xi_buf[%d] = %d\n", i, (int8_t)this->Xi_buf[i]);
    } */
    

    /********************************************************************************************
    *                                   COMPUTE PIPELINE                                        *
    * We assume that, when the pipeline is full, the stream in/out is masked by the latency for *
    * the MVM computation so we pay only the latency related to the actual computation of each  *
    * MVM.                                                                                      *
    ********************************************************************************************/
    this->compute_mvm(this->pcm);
    *latency += this->mvm_latency;

    this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Streaming out results...\n");
    for (uint32_t i=0; i<8; i++) {
        this->pcm->out_stream.rw_data(64, (void *)(this->Yi+64*i), -1);
    }

    for(uint32_t j=1; j<(this->pcm->register_file[PCM_HWPE_NUM_JOBS >> 2]); j++) {
        // Refill Xi buffer
        for (int8_t i = 0; i<8; i++) {
            this->pcm->inp_stream.rw_data(64, (void *)(this->Xi_buf+i*64), -1);
        }

        // Compute MVM
        this->compute_mvm(this->pcm);
        *latency += mvm_latency;

        // Stream outputs (we take into account the latency for the last stream out)
        this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Streaming out results...\n");
        if(j<(this->pcm->register_file[PCM_HWPE_NUM_JOBS >> 2])-1) {
            for (uint32_t i=0; i<8; i++) {
            this->pcm->out_stream.rw_data(64, (void *)(this->Yi+64*i), -1);
            }
        }
    }

    if((this->pcm->register_file[PCM_HWPE_TOTAL_LENGTH >> 2] % 8) != 0) {
        this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Total length = %d, total_length % 8 = %d\n", (this->pcm->register_file[PCM_HWPE_TOTAL_LENGTH >> 2]), (this->pcm->register_file[PCM_HWPE_TOTAL_LENGTH >> 2] % 8));
        this->pcm->trace.fatal("Leftovers still not implemented!! Please zero-pad your inputs/outputs\n");
    }

    // Last stream out: we take into account the latency for this data movement
    this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Streaming out results...\n");
    for (uint32_t i=0; i<8; i++) {
    *latency+=this->pcm->out_stream.rw_data(64, (void *)(this->Yi+64*i), -1);
    }
   
    return vp::IO_REQ_OK;
}

int Pcm_HWPE_Engine::handle_config(
    Pcm_HWPE    *pcm,
    uint32_t    ext_ctrl_addr,
    uint32_t    *ext_ctrl,
    bool        ext_write_ctrl
){
    this->pcm = pcm;
    this->ext_ctrl_addr = ext_ctrl_addr;
    this->ext_ctrl = ext_ctrl;
    this-> ext_write_ctrl = ext_write_ctrl;
    uint8_t cmd;

    int latency = 0;

    this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Configuration handler\n");

    // Initialize weights
    js::Config *stim_file_conf = this->pcm->get_js_config()->get("stim_file");
    if (stim_file_conf != NULL)
    {
        string path = stim_file_conf->get_str();
        if (path != "")
        {
            this->pcm->trace.msg("Preloading Memory with stimuli file (path: %s)\n", path.c_str());
            
            std::ifstream file(path);
            std::string line;
            uint32_t idx = 0;
            const uint32_t max_weights = PCM_NUM_LAYERS * 512 * 512;

            while (std::getline(file, line) && idx < max_weights) {
                std::stringstream ss(line);
                std::string value_str;
                while (std::getline(ss, value_str, ',') && idx < max_weights) {
                    int val = std::stoi(value_str);
                    if (val < -128 || val > 127) {
                        this->pcm->trace.warning("Weight value out of int8_t range: %d at index %d\n", val, idx);
                    }
                    this->weights[idx++] = static_cast<int8_t>(val);
                }
            }
            
            this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Loaded %d layers from stimuli file\n", idx/(512*512));
        }
        else
        {
            this->pcm->trace.msg(vp::TraceLevel::DEBUG, "No path specified, preloading PCM weights with all ones\n");
            std::fill(this->weights, this->weights+(PCM_NUM_LAYERS*512*512), 1);
        }
    }
    
    if(this->ext_write_ctrl){
        if (this->ext_ctrl_addr == STATUS_REGISTER){
            this->pcm->trace.fatal("Trying to write RO User Register 0!!\n");
            return 0;
        } else if (this->ext_ctrl_addr == CMD_REGISTER) {
            this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Setting command register to 0x%x\n", *ext_ctrl);
            this->user_registers[1] = *ext_ctrl;

            cmd = ((this->user_registers[1]) >> 24) & 0xFF;

            this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Set command: 0x%x\n", cmd);

            if (cmd == COMPUTE_CMD)
            {                
                this->handle_compute(
                    this->pcm,
                    this->ext_addr,
                    this->ext_write,
                    this->ext_data_in_shift,
                    this->ext_data_in,
                    this->ext_data_out,
                    &latency
                );

                return latency;
            } else if (cmd == ABORT_CMD)
            {
                // ABORT command still not implemented
                this->pcm->trace.fatal("ABORT command not implemented!!\n");
                return 0;
            } else
            {
                return 0;
            }
        } else {
            this->pcm->trace.fatal("Trying to access invalid address!!\n");
            return 0;
        }
    } else {
        switch (this->ext_ctrl_addr)
        {
        case STATUS_REGISTER:
            this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Reading Engine status register, value: 0x%x\n", this->user_registers[0]);
            *(uint32_t *)this->ext_ctrl = this->user_registers[0];
            break;
        case CMD_REGISTER:
            this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Reading Engine status register, value: 0x%x\n", this->user_registers[0]);
            *(uint32_t *)this->ext_ctrl = this->user_registers[1];
            break;
        default:
            this->pcm->trace.fatal("Trying to access invalid address!!\n");
            return vp::IO_REQ_INVALID;
            break;
        }

        return 0;
    }

    return 0;
}