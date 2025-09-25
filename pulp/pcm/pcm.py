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

    def i_CTRL_EXT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'ext_ctrl', signature='io')

    def i_EXT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'ext', signature='io')
