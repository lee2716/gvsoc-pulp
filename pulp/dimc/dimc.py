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

    # One D-tile DIMC: nb_inner_blocks inner blocks of macros_per_block macros,
    # reaching L1 through an inner port and L2 through an outer port.
    def __init__(self,
                 parent: gvsoc.systree.Component,
                 name: str,
                 macros_per_block: int = 2,
                 inner_port_bytes: int = 32,   # inner (L1) port, 256 bit/cycle
                 port_sync_cycles: int = 0,    # extra cycle per beat (0 = pipelined port)
                 tcdm_burst_latency: int = 1,
                 nb_inner_blocks: int = 2,     # 2 blocks x 2 macros = 4 macros
                 outer_port_shared: int = 1,   # 1 = both blocks contend on one L2 port
                 outer_port_bytes: int = 64,   # outer (L2) port, 512 bit/cycle
                 l2_burst_latency: int = 1):
        super().__init__(parent, name)

        self.set_component('pulp.dimc.dimc')

        # tcdm_burst_latency / l2_burst_latency are charged once per streamer burst and
        # do not scale with bandwidth.
        self.add_properties({
            "num_macros":        macros_per_block,
            "inner_port_bytes":  inner_port_bytes,
            "port_sync_cycles":  port_sync_cycles,
            "tcdm_burst_latency": tcdm_burst_latency,
            "nb_inner_blocks":   nb_inner_blocks,
            "outer_port_shared": outer_port_shared,
            "outer_port_bytes":  outer_port_bytes,
            "l2_burst_latency":  l2_burst_latency,
        })

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')

    # Standard HWPE completion interrupt (done_irq). Optional to bind: the model
    # guards irq.sync() with is_bound(), so leaving it unwired is harmless.
    def o_DONE_IRQ(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('done_irq', itf, signature='wire<bool>')
