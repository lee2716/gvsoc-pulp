import gvsoc.systree

class Pcm(gvsoc.systree.Component):

    def __init__(self,
                parent:gvsoc.systree.Component,
                name: str,
                latency=0,
                stim_file:str=None):
        super().__init__(parent, name)

        #self.add_sources(['pcm.hpp'])
        #self.add_sources(['pcm.cpp'])
        self.set_component('pulp.pcm.pcm')

        self.add_properties({
            "latency": latency,
            "stim_file": stim_file
        })

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')