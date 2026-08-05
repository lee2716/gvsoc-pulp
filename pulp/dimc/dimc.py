#
# Copyright (C) 2026 ETH Zurich, University of Bologna and Fondazione ChipsIT
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import gvsoc.systree


class Dimc(gvsoc.systree.Component):

    def __init__(self,
                 parent: gvsoc.systree.Component,
                 name: str,
                 num_macros: int = 2,
                 stream_l1bw: int = 32,     # inner port 256 bits
                 stream_sync: int = 0,      # MAGIA l1_spm gnt=1 -> pipelined, no extra cycle
                 stream_noc_lat: int = 1,   # TCDM bank read latency (memory latency=1)
                 stream_min_bank: int = 4,  # MAGIA BYTES_PER_WORD: TCDM bank word
                 stream_num_block: int = 2,
                 stream_l2_shared: int = 1,
                 stream_l2bw: int = 64,     # outer port 512 bits
                 stream_noc_l2: int = -1):
        super().__init__(parent, name)

        self.set_component('pulp.dimc.dimc')

        # Streamer timing knobs (fixed hardware properties). Set them here or via
        # gvrun --param (see set_stream_params) to sweep the design space without
        # recompiling the RISC-V test binary. stream_noc_lat = NoC round-trip
        # latency added once per streamer burst (input-load burst, output-store
        # burst); bandwidth-independent, so it models the interconnect delay.
        self.add_properties({
            "num_macros":         num_macros,
            "inner_port_bytes":  stream_l1bw,   # inner port B/cycle (256 bits)
            "port_sync_cycles":        stream_sync,
            "inner_noc_lat":     stream_noc_lat,
            "bank_word_bytes":    stream_min_bank,
            # OUTER BLOCK (outer double buffer). Default = the D-tile shape:
            # nb_inner=2 inner blocks x num_macros=2 macros = 4 DIMC macros,
            # Sentinels keep the "no explicit parameter" behaviour: stream_l2_bw=0
            # => same as inner_bw; stream_noc_l2=-1 => same as noc_lat (resolved in C++).
            "nb_inner_blocks":   stream_num_block,
            "outer_port_shared":   stream_l2_shared,
            "outer_port_bytes":       stream_l2bw,
            "outer_noc_lat":      stream_noc_l2,
        })

    # Called from the tile/soc/board configure() chain so gvrun --param can set
    # these at launch time (mirrors PCM's set_stim_file / weights_path pattern).
    def set_stream_params(self, inner_bw=None, sync=None, noc_lat=None,
                          bank_word=None, nb_inner=None, outer_shared=None,
                          outer_bw=None, outer_noc=None):
        props = {}
        if inner_bw       is not None: props["inner_port_bytes"]  = int(inner_bw)
        if sync       is not None: props["port_sync_cycles"]        = int(sync)
        if noc_lat    is not None: props["inner_noc_lat"]     = int(noc_lat)
        if bank_word   is not None: props["bank_word_bytes"]    = int(bank_word)
        if nb_inner  is not None: props["nb_inner_blocks"]   = int(nb_inner)
        if outer_shared  is not None: props["outer_port_shared"]   = int(outer_shared)
        if outer_bw       is not None: props["outer_port_bytes"]       = int(outer_bw)
        if outer_noc     is not None: props["outer_noc_lat"]      = int(outer_noc)
        if props:
            self.add_properties(props)

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')

    # Standard HWPE completion interrupt (done_irq). Optional to bind: the model
    # guards irq.sync() with is_bound(), so leaving it unwired is harmless.
    def o_DONE_IRQ(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('done_irq', itf, signature='wire<bool>')
