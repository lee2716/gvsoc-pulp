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
import memory.memory as memory
import vp.clock_domain
import utils.loader.loader
import interco.router as router

from pulp.chips.democritos.democritos_A_tile import Democritos_A_Tile
from pulp.chips.raptor.raptor_arch import RaptorArch
from pulp.floonoc.floonoc import *
#from pulp.fractal_sync.fractal_sync import * # needed?
from pulp.chips.magia.kill_module.kill_module import *
from typing import List, Dict
import math

def n_fract_per_lvl(param: int) -> list[int]:
        max_power = int(math.log2(param))
        return [2**i for i in reversed(range(max_power))]

def calculate_north_south(n, tiling):
    """
    Calculates the north and south position of the fractals based on tiling parameter.
    """
    if n < tiling:
        north = n
    else:
        row = n // tiling
        column = n % tiling
        north = row * 2 * tiling + column
    
    south = north + tiling
    return north, south

class RaptorSoc (gvsoc.systree.Component):
    def __init__(self, parent, name, parser, binary):
        super().__init__(parent, name)

        # Bin loader
        loader = utils.loader.loader.ElfLoader(self, f'loader', binary=binary)

        # Simulation engine killer
        killer=KillModule(self,'kill-module',kill_addr_base=RaptorArch.TEST_END_ADDR_START,kill_addr_size=RaptorArch.TEST_END_SIZE,nb_cores_to_wait=RaptorArch.NB_TILES)

        # Single clock domain
        clock = vp.clock_domain.Clock_domain(self, 'tile-clock',
                                             frequency=RaptorArch.TILE_CLK_FREQ)
        clock.o_CLOCK(self.i_CLOCK())

        # Create Tiles
        cluster:List[Democritos_A_Tile] = []
        for id in range(0,RaptorArch.NB_TILES):
            cluster.append(Democritos_A_Tile(self, f'a-tile-{id}', parser, id))
        
        l2_mem = memory.Memory(self, f'L2-mem', size=RaptorArch.L2_SIZE, latency=1)

        # Create Tile matrix for IDs
        # --------------> X direction
        # | 0  1  2  3
        # | 4  5  6  7
        # | 8  9 10 11
        # |12 13 14 15
        # |
        # V
        # Y direction

        # Init matrix
        tile_matrix: List[List[int]] = [[0 for _ in range(RaptorArch.N_TILES_X)] for _ in range(RaptorArch.N_TILES_Y)]
        # Populate matrix
        id=0
        for y in range(0,RaptorArch.N_TILES_Y):
            for x in range(0,RaptorArch.N_TILES_X):
                tile_matrix[y][x] = id
                id = id + 1

        for row in tile_matrix:
            print(row)

        # FactalSync (TBD)

        # Connect NoC to tiles and L2
        noc = FlooNoc2dMeshNarrowWide(self,
                                    name='raptor-noc',
                                    narrow_width=4,
                                    wide_width=4,
                                    ni_outstanding_reqs=8,
                                    router_input_queue_size=4,
                                    dim_x=RaptorArch.N_TILES_X+1, dim_y=RaptorArch.N_TILES_Y)

        # Create routers
        for y in range(0,RaptorArch.N_TILES_Y):
            for x in range(1,RaptorArch.N_TILES_X+1):
                print(f"[NoC] Adding router and NI at position x={x} y={y}")
                noc.add_router(x,y)
                noc.add_network_interface(x,y)

        for y in range(0,RaptorArch.N_TILES_Y):
            print(f"[NoC] L2-NI at position x={x} y={y}")
            noc.add_network_interface(0,y)

        # Bind tiles to noc. E.g. for 4x4
        # {1.0}----{2.0}----{3.0}----{4.0}
        #   | 0      |  1     |  2     |  3
        #   |        |        |        |
        # {1.1}----{2.1}----{3.1}----{4.1}
        #   | 4      |  5     |  6     |  7
        #   |        |        |        |
        # {1.2}----{2.2}----{3.2}----{4.2}
        #   | 8      |  9     |  10    |  11
        #   |        |        |        |
        # {1.3}----{2.3}----{3.3}----{4.3}
        #     12        13       14       15

        id=0
        for y in range(0,RaptorArch.N_TILES_Y):
            for x in range(1,RaptorArch.N_TILES_X+1):
                print(f"[NoC] Adding tile {id} at position x={x} y={y}")
                cluster[id].o_KILLER_OUTPUT(killer.i_INPUT())
                cluster[id].o_NARROW_OUTPUT(noc.i_NARROW_INPUT(x,y))
                noc.o_NARROW_MAP(cluster[id].i_NARROW_INPUT(),name=f'tile-{id}-l1-mem',base=RaptorArch.L1_ADDR_START+(id*RaptorArch.L1_TILE_OFFSET),size=RaptorArch.L1_SIZE, x=x, y=y, rm_base=False)
                id += 1

        # Bind memory to noc
        # {0.0}----{1.0}----{2.0}----{3.0}----{4.0}
        #   | L2     | 0      |  1     |  2     |  3
        #   |        |        |        |        |
        # {0.1}----{1.1}----{2.1}----{3.1}----{4.1}
        #   | L2     | 4      |  5     |  6     |  7
        #   |        |        |        |        |
        # {0.2}----{1.2}----{2.2}----{3.2}----{4.2}
        #   | L2     | 8      |  9     |  10    |  11
        #   |        |        |        |        |
        # {0.3}----{1.3}----{2.3}----{3.3}----{4.3}
        #     L2       12        13       14       15

        for y in range(0,RaptorArch.N_TILES_Y):
            noc.o_NARROW_BIND(l2_mem.i_INPUT(), x=0, y=y)

        noc.o_MAP_DIR(base=RaptorArch.L2_ADDR_START,size=RaptorArch.L2_SIZE, dir=FlooNocDirection.LEFT,name=f'mem_left',rm_base=True)
        
        # Bind loader
        for id in range(0,RaptorArch.NB_TILES):
            if (id == 0):
                loader.o_OUT(cluster[id].i_LOADER()) #only cluster connected to the corner loads the elf
            loader.o_START(cluster[id].i_FETCHEN())
            loader.o_ENTRY(cluster[id].i_ENTRY())

                                    
