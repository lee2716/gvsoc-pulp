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
                 stream_chunk_bytes: int = 128,
                 stream_l1bw: int = 128,
                 stream_sync: int = 0,
                 stream_noc_lat: int = 1,
                 stream_min_bank: int = 4,
                 stream_prefetch: int = 0,
                 stream_num_block: int = 1,
                 stream_l2_shared: int = 1,
                 stream_l1_depth: int = 1,
                 stream_l2bw: int = 0,
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
            "stream_chunk_bytes": stream_chunk_bytes,
            "stream_bank_bytes":  stream_l1bw,   # L1 bandwidth bytes/cycle
            "stream_sync":        stream_sync,
            "stream_noc_lat":     stream_noc_lat,
            "stream_min_bank":    stream_min_bank,
            "stream_prefetch":    stream_prefetch,
            # OUTER BLOCK (outer double buffer). Sentinels: stream_l2_bw=0
            # => same as l1bw; stream_noc_l2=-1 => same as noc_lat (resolved in C++).
            "stream_num_block":   stream_num_block,
            "stream_l2_shared":   stream_l2_shared,
            "stream_l1_depth":    stream_l1_depth,
            "stream_l2_bw":       stream_l2bw,
            "stream_noc_l2":      stream_noc_l2,
        })

    # Called from the tile/soc/board configure() chain so gvrun --param can set
    # these at launch time (mirrors PCM's set_stim_file / weights_path pattern).
    def set_stream_params(self, chunk_bytes=None, l1bw=None, sync=None, noc_lat=None,
                          min_bank=None, prefetch=None, num_block=None, l2_shared=None,
                          l1_depth=None, l2bw=None, noc_l2=None):
        props = {}
        if chunk_bytes is not None: props["stream_chunk_bytes"] = int(chunk_bytes)
        if l1bw       is not None: props["stream_bank_bytes"]  = int(l1bw)
        if sync       is not None: props["stream_sync"]        = int(sync)
        if noc_lat    is not None: props["stream_noc_lat"]     = int(noc_lat)
        if min_bank   is not None: props["stream_min_bank"]    = int(min_bank)
        if prefetch   is not None: props["stream_prefetch"]    = int(prefetch)
        if num_block  is not None: props["stream_num_block"]   = int(num_block)
        if l2_shared  is not None: props["stream_l2_shared"]   = int(l2_shared)
        if l1_depth   is not None: props["stream_l1_depth"]    = int(l1_depth)
        if l2bw       is not None: props["stream_l2_bw"]       = int(l2bw)
        if noc_l2     is not None: props["stream_noc_l2"]      = int(noc_l2)
        if props:
            self.add_properties(props)

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')
