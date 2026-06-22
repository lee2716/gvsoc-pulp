import gvsoc.systree


class Dimc(gvsoc.systree.Component):

    def __init__(self,
                 parent: gvsoc.systree.Component,
                 name: str,
                 dimc_latency: int = 0,
                 num_macros: int = 2,
                 fifo_depth: int = 8):
        super().__init__(parent, name)

        self.set_component('pulp.dimc.dimc')

        self.add_properties({
            "dimc_latency": dimc_latency,
            "num_macros":   num_macros,
            "fifo_depth":   fifo_depth,
        })

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')
