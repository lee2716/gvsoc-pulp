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
import interco.router as router
import gdbserver.gdbserver
from pulp.stdout.stdout_v3 import Stdout

import pulp.cpu.iss.pulp_cores as iss
from pulp.cluster.l1_interleaver import L1_interleaver
from pulp.light_redmule.hwpe_interleaver import HWPEInterleaver
from pulp.snitch.snitch_cluster.dma_interleaver import DmaInterleaver
from pulp.chips.democritos.hierarchical_cache import Hierarchical_cache

from pulp.chips.democritos.democritos_arch import *
from pulp.chips.magia_v2.cv32.core import CV32CoreTest
from pulp.dimc.dimc import Dimc
from pulp.idma.snitch_dma import SnitchDma
from pulp.chips.magia_v2.fractal_sync_mm_ctrl.fractal_sync_mm_ctrl import FSync_mm_ctrl
from pulp.chips.magia_v2.idma_mm_ctrl.idma_mm_ctrl import iDMA_mm_ctrl
from pulp.event_unit.event_unit_v3 import Event_unit


# adapted from snitch cluster model
# interface i_INPUT -> interleaver -> banks
class Democritos_D_TileTcdm(gvsoc.systree.Component):
    def __init__(self, parent, name, parser):
        super().__init__(parent, name)

        # TODO: for early tests only. Move to json later
        nb_banks = DemocritosArch.N_MEM_BANKS
        bank_size = DemocritosArch.N_WORDS_BANK * DemocritosArch.BYTES_PER_WORD

        # 1 master: OBI, iDMA0, iDMA1
        L1_masters = 3
        interleaver = L1_interleaver(self, 'interleaver', nb_slaves=nb_banks, nb_masters=L1_masters, interleaving_bits=2)

        # OBI plus the two iDMAs. They share one DmaInterleaver input, the way
        # magia_v2/tile.py:59 does it; the count is how many composite ports the
        # tile exposes, not how many interleaver inputs exist.
        # 0: OBI xbar (core-issued L1 access through the DMA interleaver)
        # 1: iDMA0 TCDM side   2: iDMA1 TCDM side   3: NoC wide channel inbound
        dma_masters = 4
        dma_interleaver = DmaInterleaver(self, 'dma_interleaver', nb_master_ports=dma_masters, nb_banks=nb_banks, bank_width=DemocritosArch.BYTES_PER_WORD)

        # 1 master: DIMC HWPE
        dimc_hwpe_masters = 1
        dimc_interleaver = HWPEInterleaver(self, "dimc_interleaver", nb_master_ports=dimc_hwpe_masters, nb_banks=nb_banks, bank_width=DemocritosArch.BYTES_PER_WORD)

        banks = []
        for i in range(nb_banks):
            # Instantiate a new memory bank
            # atomics and truncate_size match magia_v2/tile.py:74. Without
            # atomics the banks reject RISC-V atomic instructions; truncate_size
            # masks an incoming address with (size - 1), so a bank sees an
            # in-range offset instead of running past its end.
            bank = memory.Memory(self, f'bank_{i}', atomics=True, size=bank_size,
                                 latency=DemocritosDSE.TILE_TCDM_LATENCY,
                                 truncate_size=bank_size)
            banks.append(bank)

            # Bind the new bank (slave) to the interleaver (master)
            self.bind(interleaver, f'out_{i}', bank, 'input')
            self.bind(dma_interleaver, f'out_{i}', bank, 'input')
            self.bind(dimc_interleaver, f'out_{i}', bank, 'input')

        # Bind the external porrts (input->[internal]output->interleaver)
        for i in range(L1_masters):
            self.bind(self, f'L1_input_{i}', interleaver, f'in_{i}')

        for i in range(dma_masters):
            self.bind(self, f'IDMA_input_{i}', dma_interleaver, f'input')

        # CHECK correct binding, i.e. name of the ports
        for i in range(dimc_hwpe_masters):
            self.bind(self, f'DIMC_HWPE_input', dimc_interleaver, f'input')

    # Input ports (port number as argument)
    def i_INPUT(self, id: int) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'L1_input_{id}', signature='io')

    def i_DMA_INPUT(self, id: int) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'IDMA_input_{id}', signature='io')

    def i_DIMC_HWPE_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, f'DIMC_HWPE_input', signature='io')

class Democritos_D_Tile(gvsoc.systree.Component):
    def __init__(self, parent, name, parser, tid: int=0):
        super().__init__(parent, name)

        # Core
        core_cv32 = CV32CoreTest(self, f'tile-{tid}-cv32-core', core_id=tid)

        # Instruction cache (from snitch cluster model)
        i_cache = Hierarchical_cache(self, f'tile-{tid}-i-cache', nb_cores=1, has_cc=0, l1_line_size_bits=7)

        # Data scratchpad
        l1_tcdm = Democritos_D_TileTcdm(self, f'tile-{tid}-l1-tcdm', parser)

       # AXI and OBI x-bars
        tile_xbar = router.Router(self, f'tile-{tid}-axi-xbar',bandwidth=4,latency=DemocritosDSE.TILE_AXI_XBAR_LATENCY,synchronous=DemocritosDSE.TILE_AXI_XBAR_SYNC)
        obi_xbar = router.Router(self, f'tile-{tid}-obi-xbar',bandwidth=4,latency=DemocritosDSE.TILE_OBI_XBAR_LATENCY,synchronous=DemocritosDSE.TILE_OBI_XBAR_SYNC)

        # iDMA controller
        idma_mm_ctrl= iDMA_mm_ctrl(self,f'tile-{tid}-idma-ctrl-mm')

        # iDMA
        idma0 = SnitchDma(self,f'tile-{tid}-idma0',loc_base=(tid*DemocritosArch.L1_TILE_OFFSET),loc_size=DemocritosArch.L1_SIZE,tcdm_width=32,transfer_queue_size=1,burst_queue_size=DemocritosDSE.TILE_IDMA0_BQUEUE_SIZE,burst_size=DemocritosDSE.TILE_IDMA0_B_SIZE)
        idma1 = SnitchDma(self,f'tile-{tid}-idma1',loc_base=(tid*DemocritosArch.L1_TILE_OFFSET),loc_size=DemocritosArch.L1_SIZE,tcdm_width=32,transfer_queue_size=1,burst_queue_size=DemocritosDSE.TILE_IDMA1_BQUEUE_SIZE,burst_size=DemocritosDSE.TILE_IDMA1_B_SIZE)

        # DIMC HWPE. One outer block = 2 inner blocks x 2 macros = 4 macros.
        # The macros of a block share one inner port and double-buffer against
        # each other; the rest of the architecture is fixed in Dimc().
        dimc = Dimc(self, 'dimc',
                    macros_per_block  = 2,    # macros sharing one inner port
                    nb_inner_blocks   = 2,    # 2 blocks x 2 macros = 4 macros
                    # Both port widths set every STARTING and STORING beat
                    # count, so the measured phase lengths rest on them.
                    inner_port_bytes  = 32,   # inner (L1) port, 256 bit/cycle
                    outer_port_bytes  = 64,   # outer (L2) port, 512 bit/cycle;
                                              # confirmed with the IP owners and
                                              # backed by an earlier measurement
                    # Both blocks arbitrate for one outer port. Since
                    # cut-through lets them stream together at 32 B/cycle each,
                    # the shared 64 B/cycle port is exactly saturated and
                    # splitting it in two measured no different.
                    outer_port_shared = 1,
                    cross_job_prefetch = 1,   # fill the queued job while this one computes
                    # A block starts streaming into its macros as soon as the
                    # outer port has bandwidth for the beat, rather than waiting
                    # for its whole working set. Set to 0 for the
                    # store-and-forward arm.
                    outer_cut_through = 1)
        self.dimc = dimc

        # Event unit (mirrors pulp/chips/magia_v2/tile.py). The address window is
        # already reserved in DemocritosArch; the HWPE raises done_irq into it so
        # SW can wait on an event instead of polling STATUS.
        self.add_properties({
            "event_unit": {
                "version": "4",
                "mapping": {
                    "base":          DemocritosArch.EVENT_UNIT_ADDR_START,
                    "size":          DemocritosArch.EVENT_UNIT_SIZE,
                    "remove_offset": DemocritosArch.EVENT_UNIT_ADDR_START,
                },
                "config": {
                    "nb_core": 1,                      # D-tile has one CV32 core
                    "properties": {
                        "dispatch":  {"size": 8},
                        "mutex":     {"nb_mutexes": 0},
                        "barriers":  {"nb_barriers": 0},
                        "soc_event": {"nb_fifo_events": 8, "fifo_event": 8},
                        "events":    {"dispatch": 8, "mutex": 0, "barrier": 0},
                    },
                },
            }
        })
        event_unit = Event_unit(self, f'tile-{tid}-event-unit',
                                self.get_property('event_unit/config'))

        # Fsync mm controller
        fsync_mm_ctrl = FSync_mm_ctrl(self,f'tile-{tid}-fs-ctrl-mm')

        # UART
        stdout = Stdout(self, f'tile-{tid}-stdout', max_cluster=DemocritosArch.NB_CLUSTERS, max_core_per_cluster=1, user_set_core_id=0, user_set_cluster_id=tid)

        # Bind: CV32 core data -> OBI interconnect
        core_cv32.o_DATA(obi_xbar.i_INPUT())
        core_cv32.o_DATA_DEBUG(obi_xbar.i_INPUT())

        # Bind: ELF loader -> OBI interconnect
        self.__o_LOADER(obi_xbar.i_INPUT())

        # Bind: CV32 core -> I$
        core_cv32.o_FETCH(i_cache.i_INPUT(0))
        core_cv32.o_FLUSH_CACHE(i_cache.i_FLUSH())
        i_cache.o_FLUSH_ACK(core_cv32.i_FLUSH_CACHE_ACK())

        # Bind: I$ -> tile interconnect
        i_cache.o_REFILL(tile_xbar.i_INPUT())

        # Bind OBI Xbar so that it can communicate with local reserved
        obi_xbar.o_MAP(l1_tcdm.i_INPUT(0), name='local-reserved',
                       base=DemocritosArch.RESERVED_ADDR_START,
                       size=DemocritosArch.RESERVED_SIZE, rm_base=False)

        # Bind OBI Xbar so that it can communicate with local stack
        obi_xbar.o_MAP(l1_tcdm.i_INPUT(0), name='local-stack',
                       base=DemocritosArch.STACK_ADDR_START,
                       size=DemocritosArch.STACK_SIZE, rm_base=False)

        # Bind OBI Xbar so that it can communicate with local L1
        obi_xbar.o_MAP(l1_tcdm.i_DMA_INPUT(0), name='local-l1-mem', #here we use the iDMA interleaver because an iDMA AXI request routed to OBI (e.g. local L1 to off-tile L1 data movement) does not handle the right bank interleaving
                       base=DemocritosArch.L1_ADDR_START,
                       size=DemocritosArch.L1_SIZE, rm_base=False, remove_offset=(tid*DemocritosArch.L1_TILE_OFFSET))

        # Bind OBI Xbar so that it can communicate with the tile Xbar to get access to remote tiles L1
        for tile_id in range(DemocritosArch.NB_CLUSTERS):
            if tile_id != tid: # skip yourself
                obi_xbar.o_MAP(tile_xbar.i_INPUT(), name=f'ob12axi-off-tile-{tile_id}-l1-mem',
                               base=DemocritosArch.L1_ADDR_START + (tile_id*DemocritosArch.L1_TILE_OFFSET),
                               size=DemocritosArch.L1_SIZE, rm_base=False)

        # Bind tile Xbar so that it can communicate with OBI Xbar L1 mem
        tile_xbar.o_MAP(obi_xbar.i_INPUT(), name='axi2obi-l1-mem',
                        base=DemocritosArch.L1_ADDR_START + (tile_id*DemocritosArch.L1_TILE_OFFSET),
                        size=DemocritosArch.L1_SIZE, rm_base=False)

        # Bind tile Xbar so that it can communicate with OBI Xbar reserved mem
        tile_xbar.o_MAP(obi_xbar.i_INPUT(), name='axi-to-obi-reserved-mem',
                        base=DemocritosArch.RESERVED_ADDR_START+(tid*DemocritosArch.L1_TILE_OFFSET),
                        size=DemocritosArch.RESERVED_SIZE, rm_base=False)

        # Mapping used by OBI Xbar to communicate with the tile Xbar
        obi_xbar.o_MAP(tile_xbar.i_INPUT(), name='obi2axi-off-tile-l2-mem',
                       base=DemocritosArch.L2_ADDR_START,
                       size=DemocritosArch.L2_SIZE, rm_base=False)

        # Bind tile Xbar so that it can write the L2 mem
        tile_xbar.o_MAP(self.__i_NARROW_OUTPUT(), name='axi-to-off-tile-l2-mem',
                        base=DemocritosArch.L2_ADDR_START,
                        size=DemocritosArch.L2_SIZE, rm_base=False)

        # Bind OBI Xbar so that it can write to UART
        obi_xbar.o_MAP(stdout.i_INPUT(), name='local-uart-mem',
                       base=DemocritosArch.STDOUT_START,
                       size=DemocritosArch.STDOUT_SIZE, rm_base=False)

        # Bind OBI Xbar so that it can communicate to DIMC HWPE
        obi_xbar.o_MAP(dimc.i_hwpe_slv(), name='local-dimc-hwpe',
                       base=DemocritosArch.DIMC_START,
                       size=DemocritosArch.DIMC_SIZE,
                       rm_base=True)

        # Event unit: map it on the OBI xbar and route the DIMC completion
        # interrupt into it (magia_v2 pattern: bind(<hwpe>, 'done_irq', ...)).
        obi_xbar.add_mapping('event_unit', **self.get_property('event_unit/mapping'))
        self.bind(obi_xbar, 'event_unit', event_unit, 'input')
        # Core side: clock + the irq request/ack handshake (magia_v2 does the same).
        self.bind(event_unit, 'clock_0', core_cv32, 'clock')
        self.bind(core_cv32, 'irq_ack', event_unit, 'irq_ack_0')
        self.bind(event_unit, 'irq_req_0', core_cv32, 'irq_req')
        # DIMC completion -> event 10 (same slot magia_v2 gives RedMule).
        self.bind(dimc, 'done_irq', event_unit, 'in_event_10_pe_0')
        # FractalSync barrier completion -> event 24 (same slot as magia_v2).
        self.bind(fsync_mm_ctrl, 'fsync_done_irq', event_unit, 'in_event_24_pe_0')
        # iDMA completion -> events 2 and 3, the slots magia_v2 uses. These are
        # master ports on iDMA_mm_ctrl and it drives them on every completion
        # (idma_mm_ctrl.cpp:240), so leaving them unbound is a null dereference
        # the moment software issues a transfer, not merely a missing feature.
        self.bind(idma_mm_ctrl, 'idma0_done_irq', event_unit, 'in_event_2_pe_0')
        self.bind(idma_mm_ctrl, 'idma1_done_irq', event_unit, 'in_event_3_pe_0')

        # FractalSync controller registers, so software can request a barrier.
        # Same address window magia_v2 uses (arch reserves FSYNC_CTRL_*).
        obi_xbar.o_MAP(fsync_mm_ctrl.i_INPUT(), name=f'fs-ctrl-mm-{tid}-mem',
                       base=DemocritosArch.FSYNC_CTRL_ADDR_START,
                       size=DemocritosArch.FSYNC_CTRL_SIZE, rm_base=True)

        # Bind obi xbar so that it can write to kill_module
        obi_xbar.o_MAP(self.__i_KILLER_OUTPUT(), name="Kill-sim-mem",
                base=DemocritosArch.TEST_END_ADDR_START,
                size=DemocritosArch.TEST_END_SIZE, rm_base=False)

        self.__o_NARROW_INPUT(tile_xbar.i_INPUT())

        # Wide channel inbound: a remote DMA writing this tile's L1 lands on the
        # DmaInterleaver, not the core-side one. magia_v2/tile.py:313 binds the
        # same way.
        self.__o_WIDE_INPUT(l1_tcdm.i_DMA_INPUT(3))

        # Bind CV32 core enable prots -> matching composite ports
        self.__o_ENTRY(core_cv32.i_ENTRY())
        self.__o_FETCHEN(core_cv32.i_FETCHEN())

        # Bind: iDMA controller
        obi_xbar.o_MAP(idma_mm_ctrl.i_INPUT(), name=f'iDMA-ctrl-mm-{tid}-mem', base=DemocritosArch.IDMA_CTRL_ADDR_START, size=DemocritosArch.IDMA_CTRL_SIZE, rm_base=True)

        # Bind iDMA0
        # Out the WIDE port, not tile_xbar. tile_xbar is 4 bytes/cycle and its
        # L2 route leaves through the 4-byte narrow NoC; measured on that path
        # the DMA ran at exactly 4.0 B/cycle and the weight transfers were 99%
        # of the psin runtime. The wide network is what exists for DMA traffic;
        # magia_v2/tile.py:342 binds it the same way.
        idma0.o_AXI(self.__i_WIDE_OUTPUT())
        # DmaInterleaver, not the core-side L1_interleaver: the latter is
        # interleaved every 4 bytes, so a DMA bound to it lands one eighth of
        # the bytes it was asked for. magia_v2/tile.py:343 binds the same way.
        idma0.o_TCDM(l1_tcdm.i_DMA_INPUT(1))
        idma_mm_ctrl.o_OFFLOAD_iDMA0_AXI2OBI(idma0.i_OFFLOAD())
        idma0.o_OFFLOAD_GRANT(idma_mm_ctrl.i_OFFLOAD_GRANT_iDMA0_AXI2OBI())

        # Bind iDMA1
        # Out the WIDE port, not tile_xbar. tile_xbar is 4 bytes/cycle and its
        # L2 route leaves through the 4-byte narrow NoC; measured on that path
        # the DMA ran at exactly 4.0 B/cycle and the weight transfers were 99%
        # of the psin runtime. The wide network is what exists for DMA traffic;
        # magia_v2/tile.py:342 binds it the same way.
        idma1.o_AXI(self.__i_WIDE_OUTPUT())
        idma1.o_TCDM(l1_tcdm.i_DMA_INPUT(2))
        idma_mm_ctrl.o_OFFLOAD_iDMA1_OBI2AXI(idma1.i_OFFLOAD())
        idma1.o_OFFLOAD_GRANT(idma_mm_ctrl.i_OFFLOAD_GRANT_iDMA1_OBI2AXI())

        # Bind DIMC
        dimc.o_stream_mst(l1_tcdm.i_DIMC_HWPE_INPUT())

        # Bind FractalSync ports
        fsync_mm_ctrl.o_XIF_2_FRACTAL_EAST_WEST(self.__o_SLAVE_EAST_WEST_FRACTAL())
        self.__i_SLAVE_EAST_WEST_FRACTAL(fsync_mm_ctrl.i_FRACTAL_2_XIF_EAST_WEST())

        fsync_mm_ctrl.o_XIF_2_FRACTAL_NORD_SUD(self.__o_SLAVE_NORTH_SOUTH_FRACTAL())
        self.__i_SLAVE_NORTH_SOUTH_FRACTAL(fsync_mm_ctrl.i_FRACTAL_2_XIF_NORD_SUD())

        fsync_mm_ctrl.o_XIF_2_NEIGHBOUR_FRACTAL_EAST_WEST(self.__o_SLAVE_EAST_WEST_NEIGHBOUR_FRACTAL())
        self.__i_SLAVE_EAST_WEST_NEIGHBOUR_FRACTAL(fsync_mm_ctrl.i_NEIGHBOUR_FRACTAL_2_XIF_EAST_WEST())

        fsync_mm_ctrl.o_XIF_2_NEIGHBOUR_FRACTAL_NORD_SUD(self.__o_SLAVE_NORTH_SOUTH_NEIGHBOUR_FRACTAL())
        self.__i_SLAVE_NORTH_SOUTH_NEIGHBOUR_FRACTAL(fsync_mm_ctrl.i_NEIGHBOUR_FRACTAL_2_XIF_NORD_SUD())

        # Enable debug
        gdbserver.gdbserver.Gdbserver(self, 'gdbserver')

    # East West port to FractalSync
    def __o_SLAVE_EAST_WEST_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'xif_2_east_west_fractal', signature='wire<PortReq<uint32_t>*>')

    def o_SLAVE_EAST_WEST_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('xif_2_east_west_fractal', itf, signature='wire<PortReq<uint32_t>*>')

    def __i_SLAVE_EAST_WEST_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('east_west_fractal_2_xif', itf, signature='wire<PortResp<uint32_t>*>', composite_bind=True)

    def i_SLAVE_EAST_WEST_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'east_west_fractal_2_xif', signature='wire<PortResp<uint32_t>*>')

    # North South port to FractalSync
    def __o_SLAVE_NORTH_SOUTH_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'xif_2_north_south_fractal', signature='wire<PortReq<uint32_t>*>')

    def o_SLAVE_NORTH_SOUTH_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('xif_2_north_south_fractal', itf, signature='wire<PortReq<uint32_t>*>')

    def __i_SLAVE_NORTH_SOUTH_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('north_south_fractal_2_xif', itf, signature='wire<PortResp<uint32_t>*>', composite_bind=True)

    def i_SLAVE_NORTH_SOUTH_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'north_south_fractal_2_xif', signature='wire<PortResp<uint32_t>*>')

    # East West port to neighbour FractalSync
    def __o_SLAVE_EAST_WEST_NEIGHBOUR_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'xif_2_east_west_neighbour_fractal', signature='wire<PortReq<uint32_t>*>')

    def o_SLAVE_EAST_WEST_NEIGHBOUR_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('xif_2_east_west_neighbour_fractal', itf, signature='wire<PortReq<uint32_t>*>')

    def __i_SLAVE_EAST_WEST_NEIGHBOUR_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('east_west_neighbour_fractal_2_xif', itf, signature='wire<PortResp<uint32_t>*>', composite_bind=True)

    def i_SLAVE_EAST_WEST_NEIGHBOUR_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'east_west_neighbour_fractal_2_xif', signature='wire<PortResp<uint32_t>*>')

    # North South port to neighbour FractalSync
    def __o_SLAVE_NORTH_SOUTH_NEIGHBOUR_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'xif_2_north_south_neighbour_fractal', signature='wire<PortReq<uint32_t>*>')

    def o_SLAVE_NORTH_SOUTH_NEIGHBOUR_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('xif_2_north_south_neighbour_fractal', itf, signature='wire<PortReq<uint32_t>*>')

    def __i_SLAVE_NORTH_SOUTH_NEIGHBOUR_FRACTAL(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('north_south_neighbour_fractal_2_xif', itf, signature='wire<PortResp<uint32_t>*>', composite_bind=True)

    def i_SLAVE_NORTH_SOUTH_NEIGHBOUR_FRACTAL(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'north_south_neighbour_fractal_2_xif', signature='wire<PortResp<uint32_t>*>')

    # Output (master) port to off-tile L2 memory
    def o_NARROW_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('narrow_output', itf, signature='io')

    def __i_NARROW_OUTPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'narrow_output', signature='io')

    def i_NARROW_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'narrow_input', signature='io')

    def __o_NARROW_INPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('narrow_input', itf, signature='io', composite_bind=True)

    # Output (master) wide port to off-tile L2 memory. Carries DMA traffic only;
    # core accesses keep using the narrow port.
    def o_WIDE_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('wide_output', itf, signature='io')

    def __i_WIDE_OUTPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'wide_output', signature='io')

    def i_WIDE_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'wide_input', signature='io')

    def __o_WIDE_INPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('wide_input', itf, signature='io', composite_bind=True)

    # Input port for the loader
    def i_LOADER(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'loader', signature='io')

    def __o_LOADER(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('loader', itf, signature='io', composite_bind=True)

    def i_FETCHEN(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'fetchen', signature='wire<bool>')

    def __o_FETCHEN(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('fetchen', itf, signature='wire<bool>', composite_bind=True)

    def i_ENTRY(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'entry', signature='wire<uint64_t>')

    def __o_ENTRY(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('entry', itf, signature='wire<uint64_t>', composite_bind=True)

    # Killer port
    def o_KILLER_OUTPUT(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('killer_output', itf, signature='io')

    def __i_KILLER_OUTPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'killer_output', signature='io')
