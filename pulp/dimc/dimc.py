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
    # each reaching L1 through an inner port. Access latency belongs to the L1
    # bank and the crossbar, which is where magia_v2 puts RedMulE's, so this
    # model adds none of its own.
    # The three parameters that decide the geometry carry no default: the tile
    # that instantiates this model has to state them, the way
    # magia_v2/tile.py:199 states RedMulE's. A default here is a specification
    # nobody wrote down.
    def __init__(self,
                 parent: gvsoc.systree.Component,
                 name: str,
                 macros_per_block: int,
                 nb_inner_blocks: int,
                 inner_port_bytes: int,
                 ):
        super().__init__(parent, name)

        self.set_component('pulp.dimc.dimc')

        self.add_properties({
            "num_macros":        macros_per_block,
            "inner_port_bytes":  inner_port_bytes,
            "nb_inner_blocks":   nb_inner_blocks,
        })

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')

    # Standard HWPE completion interrupt (done_irq). Optional to bind: the model
    # guards irq.sync() with is_bound(), so leaving it unwired is harmless.
    def o_DONE_IRQ(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('done_irq', itf, signature='wire<bool>')
