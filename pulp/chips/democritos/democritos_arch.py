#
# Copyright (C) 2025 ETH Zurich, University of Bologna and Fondazione ChipsIT
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

import os

from gvrun.attribute import Tree, Value

class DemocritosArch:
    # Single tile address map from magia_tile_pkg.sv
    IDMA_CTRL_ADDR_START    = 0x0000_0100
    IDMA_CTRL_SIZE          = 0x0000_03FF
    IDMA_CTRL_ADDR_END      = IDMA_CTRL_ADDR_START + IDMA_CTRL_SIZE
    FSYNC_CTRL_ADDR_START   = IDMA_CTRL_ADDR_END + 1
    FSYNC_CTRL_SIZE         = 0x0000_00FF
    FSYNC_CTRL_ADDR_END     = FSYNC_CTRL_ADDR_START + FSYNC_CTRL_SIZE
    EVENT_UNIT_ADDR_START   = FSYNC_CTRL_ADDR_END + 1
    EVENT_UNIT_SIZE         = 0x0000_0FFF
    EVENT_UNIT_ADDR_END     = EVENT_UNIT_ADDR_START + EVENT_UNIT_SIZE
    RESERVED_ADDR_START     = EVENT_UNIT_ADDR_END + 1
    RESERVED_SIZE           = 0x0000_E7FF
    RESERVED_ADDR_END       = RESERVED_ADDR_START + RESERVED_SIZE
    # Spatz control block. A literal, carved out of the reserved hole above:
    # chaining it after the event unit would move PCM/DIMC, whose address the HAL
    # repeats by hand.
    SPATZ_CTRL_START        = 0x0000_1700
    SPATZ_CTRL_SIZE         = 0x0000_00FF
    SPATZ_CTRL_END          = SPATZ_CTRL_START + SPATZ_CTRL_SIZE
    # A-tile-local HWPE address segment (PCM)
    PCM_START               = RESERVED_ADDR_END + 1
    PCM_SIZE                = 0x0000_00DF
    PCM_END                 = PCM_START + PCM_SIZE
    # D-tile-local HWPE address segment (DIMC) -- same tile-local position as PCM
    DIMC_START              = PCM_START
    DIMC_SIZE               = PCM_SIZE
    DIMC_END                = PCM_END
    STACK_ADDR_START        = PCM_END + 1
    STACK_SIZE              = 0x0000_FFFF
    STACK_ADDR_END          = STACK_ADDR_START + STACK_SIZE
    L1_ADDR_START           = STACK_ADDR_END + 1
    L1_SIZE                 = 0x000D_FFFF
    L1_ADDR_END             = L1_ADDR_START + L1_SIZE
    L1_TILE_OFFSET          = 0x0010_0000
    L2_ADDR_START           = 0xC000_0000
    # Last offset, not a byte count: TEST_END_ADDR_START below is L2_ADDR_END + 1,
    # and a-tile_test/kernel/crt0.S:76 hard-codes that address as 0xCCFF_0000.
    # So this value is pinned. Anything slicing L2 must use the span L2_SIZE + 1
    # and round each slice down to a page -- see DEMOCRITOS_L2_SLICE_BYTES.
    L2_SIZE                 = 0x0CFE_FFFF
    L2_ADDR_END             = L2_ADDR_START + L2_SIZE
    TEST_END_ADDR_START     = L2_ADDR_END + 1
    TEST_END_SIZE           = 0x400
    L2_ADDR_END             = L2_ADDR_START + L2_SIZE
    STDOUT_START            = 0xFFFF_0004
    STDOUT_SIZE             = 0x100

    # From magia_pkg.sv
    N_MEM_BANKS         = 32        # Number of TCDM banks
    N_WORDS_BANK        = 8192      # Number of words per TCDM bank

    # Extra
    BYTES_PER_WORD      = 4
    TILE_CLK_FREQ       = 50 * (10 ** 6)

    ENABLE_NOC          = True
    # Snitch+Spatz vector core, one component for both the scalar host and the
    # vector unit. It runs its own binary, entered from the boot rom. Only the
    # V-tile instantiates it.
    SPATZ_BOOTROM_ADDR  = 0x1000_0000
    SPATZ_BOOTROM_SIZE  = 0x100
    # Produced by `make bootrom` in democritos_tests/v-tile_test. The testbench
    # normally passes an explicit path (rom_file=) computed next to itself; this
    # is the fallback for a target that does not, and SPATZ_ROMFILE in the
    # environment overrides both. A bare name is searched on the systree path.
    SPATZ_ROMFILE       = os.environ.get('SPATZ_ROMFILE', 'spatz_init.bin')
    SPATZ_VLEN          = 256
    SPATZ_NB_LANES      = 4
    SPATZ_LANE_WIDTH    = 4
    SPATZ_NB_VLSU_PORTS = 4
    N_TILES_X           = 2 # 16
    N_TILES_Y           = 2 # 16
    NB_CLUSTERS         = N_TILES_X*N_TILES_Y # to be removed when we'll use the DemocritosTree properties instead of hardcoding the number of clusters in the components
    # Which accelerator each mesh position carries, one character per position
    # indexed by tile id: 'd' DIMC tile, 'a' A-tile, 'v' D-tile plus a
    # Snitch+Spatz vector core. Set DEMOCRITOS_TILE_TYPES to select a mesh
    # without editing this file, for example 'vvvv' for four Spatz tiles.
    # Keep NB_CLUSTERS a power of two: democritos_soc.py sizes the FractalSync
    # tree by int(log2(NB_CLUSTERS)) and under-provisions it silently otherwise.
    TILE_TYPES          = list(os.environ.get('DEMOCRITOS_TILE_TYPES',
                                              'd' * NB_CLUSTERS))

class DemocritosTree(Tree):
    def __init__(self, parent, name):
        super().__init__(parent, name)
        self.n_tiles_x = Value(self, 'n_tiles_x', DemocritosArch.N_TILES_X, cast=int,
            description='Number of tiles on X dimension')
        self.n_tiles_y = Value(self, 'n_tiles_y', DemocritosArch.N_TILES_Y, cast=int,
            description='Number of tiles on Y dimension')

        self.nb_clusters = self.n_tiles_x*self.n_tiles_y


class DemocritosDSE:
    SOC_L2_LATENCY              = 2
    TILE_ICACHE_REFILL_LATENCY  = 2
    TILE_TCDM_LATENCY           = 1
    TILE_AXI_XBAR_LATENCY       = 2
    TILE_AXI_XBAR_SYNC          = False
    TILE_OBI_XBAR_LATENCY       = 2
    TILE_OBI_XBAR_SYNC          = True
    TILE_IDMA0_BQUEUE_SIZE      = 8
    TILE_IDMA0_B_SIZE           = 32
    TILE_IDMA1_BQUEUE_SIZE      = 8
    TILE_IDMA1_B_SIZE           = 32
