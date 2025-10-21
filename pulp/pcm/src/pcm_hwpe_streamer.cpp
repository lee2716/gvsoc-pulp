#include <pcm.hpp>

#define BYTES_PER_BANK 4

Pcm_HWPE_Streamer::Pcm_HWPE_Streamer(Pcm_HWPE* pcm, bool is_write) {
    this->pcm = pcm;
	
	this->base_addr = 0;
	this->tot_len   = 0;
	this->d0_len    = 0;
	this->d0_stride = 0;
	this->d1_len    = 0;
	this->d1_stride = 0;
	this->d2_len	= 0;
	this->d2_stride = 0;
	this->d3_stride = 0;
    this->pos       = 0;
	this->tot_iters = 0;
	this->d0_iters  = 0;
	this->d1_iters  = 0;
	this->d2_iters	= 0;
	this->req		= this->pcm->stream_mst.req_new(0, 0, 0, is_write);
	this->is_write	= is_write;
}

Pcm_HWPE_Streamer::Pcm_HWPE_Streamer() {
	this->pcm = (Pcm_HWPE *) NULL;
}

void Pcm_HWPE_Streamer::configure(
	uint32_t	base_addr	,
	uint32_t 	tot_len 	,
	uint32_t 	d0_len 		,
	uint32_t 	d0_stride	,
	uint32_t 	d1_len 		,
	uint32_t 	d1_stride 	,
	uint32_t	d2_len		,
	uint32_t 	d2_stride	,
	uint32_t	d3_stride	
) {
	this->base_addr = base_addr	;
	this->tot_len   = tot_len  	;
	this->d0_len    = d0_len   	;
	this->d0_stride = d0_stride	;
	this->d1_len    = d1_len   	;
	this->d1_stride = d1_stride	;
	this->d2_len	= d2_len	;
	this->d2_stride = d2_stride	;
	this->d3_stride	= d3_stride	;
    this->pos       = 0			;

	this->pcm->trace.msg("base addr %x\ntot len %d\nd0 len %x\nd0 stride %x\nd1 len %x\nd1 stride %x\nd2 stride %x\nd3 stride %x\n",
		this->base_addr	,
		this->tot_len  	,
		this->d0_len   	,
		this->d0_stride	,
		this->d1_len   	,
		this->d1_stride	,
		this->d2_stride ,
		this->d3_stride
	);
}

void Pcm_HWPE_Streamer::set_base_addr(uint32_t addr) {
	this->base_addr = addr;
}

uint32_t Pcm_HWPE_Streamer::get_base_addr() {
	return this->base_addr;
}

bool Pcm_HWPE_Streamer::is_done() {
	return this->tot_iters == this->tot_len;
}

int Pcm_HWPE_Streamer::rw_data(int width, void* buf, strobe_t strb) {
	uint32_t offs = (this->base_addr + this->pos);
	int64_t latency = 0;
	int64_t max_latency = 0;

	uint32_t tmp = 0;

	if (this->is_done()) {
		return 1;
	}

	if (buf != NULL) {
		if (offs % BYTES_PER_BANK != 0) {
			if (this->is_write) {	//TODO: strobe is not taken into account
				this->req->set_addr(offs);
				this->req->set_data((uint8_t *) buf);
				this->req->set_size(BYTES_PER_BANK - (offs % BYTES_PER_BANK));

				strb = strb >> BYTES_PER_BANK - (offs % BYTES_PER_BANK);
			} else {
				this->req->set_addr(offs - (offs % BYTES_PER_BANK));
				this->req->set_data((uint8_t *) &tmp);
				this->req->set_size(BYTES_PER_BANK);
			}

			vp::IoReqStatus err = this->pcm->stream_mst.req(this->req);

			if (err != vp::IO_REQ_OK) {
				this->pcm->trace.fatal("There was an error while reading/writing data\n");
				return 0;
			}

			latency = req->get_latency();

			if (!this->is_write) { 
				for (int i = 0; i < BYTES_PER_BANK - (offs % BYTES_PER_BANK); i++) {
					if (strb & 0x1) {
						* (((uint8_t *) buf) + i) = * (((uint8_t *) &tmp) + i + (offs % BYTES_PER_BANK));
					}

					strb = strb >> 1;
				}
			}
		
			max_latency = latency > max_latency ? latency : max_latency;
		}

		// this->pcm->trace.msg(vp::TraceLevel::DEBUG, "width = %d\n", width);
		
		for (int i = (offs % BYTES_PER_BANK) == 0 ? 0 : BYTES_PER_BANK - (offs % BYTES_PER_BANK); i < width; i += BYTES_PER_BANK) {
			if (strb == 0) {
				break;
			}

			/* this->pcm->trace.msg(vp::TraceLevel::DEBUG, "Accessing address 0x%x\n", offs+i);
			this->pcm->trace.msg(vp::TraceLevel::DEBUG, "d0_iter = %d\n", this->d0_iters); */

			if (i + BYTES_PER_BANK <= width) {
				if ((strb & 0xF) == 0xF) {
					this->req->set_addr(offs + i);
					this->req->set_data(((uint8_t *) buf) + i);
					this->req->set_size(BYTES_PER_BANK);

					// this->pcm->trace.msg(vp::TraceLevel::DEBUG, "[Streamer - mem. req]: address: 0x%x, size: 0x%x\n", this->req->get_addr(), this->req->get_size());
					vp::IoReqStatus err = this->pcm->stream_mst.req(this->req);

					if (err != vp::IO_REQ_OK) {
						this->pcm->trace.fatal("There was an error while reading/writing data\n");
						return 0;
					}

					latency = req->get_latency();

					strb = strb >> BYTES_PER_BANK;
				} else {
					if (this->is_write) {	//TODO: does not support strobes with 0s at the beginning
						int ones = sizeof(uint64_t) * 8 - __builtin_clzll (strb);

						this->req->set_addr(offs + i);
						this->req->set_data(((uint8_t *) buf) + i);
						this->req->set_size(ones);

						strb = strb >> BYTES_PER_BANK;
					} else {
						this->req->set_addr(offs + i);
						this->req->set_data((uint8_t *) &tmp);
						this->req->set_size(BYTES_PER_BANK);
					}

					vp::IoReqStatus err = this->pcm->stream_mst.req(this->req);

					if (err != vp::IO_REQ_OK) {
						this->pcm->trace.fatal("There was an error while reading/writing data\n");
						return 0;
					}

					latency = req->get_latency();

					if (!this->is_write) {
						for (int j = 0; j < BYTES_PER_BANK; j++) {
							if (strb & 0x1) {
								* (((uint8_t *) buf) + j + i) = * (((uint8_t *) &tmp) + j);
							}

							strb = strb >> 1;
						}
					}
				}
			} else {
				if (this->is_write) {	//TODO: strobe
					int ones = sizeof(uint64_t) * 8 - __builtin_clzll (strb);

					this->req->set_addr(offs + i);
					this->req->set_data(((uint8_t *) buf) + i);
					this->req->set_size(ones > width - i ? width - i : ones);
				} else {
					this->req->set_addr(offs + i);
					this->req->set_data((uint8_t *) &tmp);
					this->req->set_size(BYTES_PER_BANK);
				}

				if (req->get_size() != 0) {
					vp::IoReqStatus err = this->pcm->stream_mst.req(this->req);

					if (err != vp::IO_REQ_OK) {
						this->pcm->trace.fatal("There was an error while reading/writing data\n");
						return 0;
					}

					latency = req->get_latency();
				} else {
					latency = 0;
				}

				if (!this->is_write) {
					for (int j = 0; j < width - i; j++) {
						if (strb & 0x1) {
							* (((uint8_t *) buf) + j + i) = * (((uint8_t *) &tmp) + j);
						}

						strb = strb >> 1;
					}
				}
			}

			max_latency = latency > max_latency ? latency : max_latency;
		}
	}

	//this->pcm->trace.msg(vp::TraceLevel::DEBUG, "d0_iters = %d\n d1_iters = %d\n, tot_iters = %d\n", this->d0_iters, this->d1_iters, this->tot_iters);

	this->pos += this->d0_stride;
	this->d0_iters++;
	this->tot_iters++;

	if (this->d0_iters == this->d0_len) {
		this->pos -= this->d0_len * this->d0_stride;
		this->pos += this->d1_stride;
		this->d0_iters = 0;
		this->d1_iters++;

		if (this->d1_iters == this->d1_len) {
			this->pos -= this->d1_len * this->d1_stride;
			this->pos += this->d2_stride;
			this->d1_iters = 0;
			this->d2_iters++;

			if (this->d2_iters == this->d2_len) {
				this->pos -= this->d2_len * this->d2_stride;
				this->pos += this->d3_stride;
				this->d2_iters = 0;
			}
		}
	}

	return (int) max_latency + 1;
}

int Pcm_HWPE_Streamer::iterate(void* buf, strobe_t strb) {
	int latency = 1;

/* 
	latency = this->rw_data(sizeof(src_fmt_t) * (ARRAY_HEIGHT) * (PIPE_REGS + 1), buf, strb); */

	return latency;
}