import gvsoc.systree


class Dimc(gvsoc.systree.Component):

    def __init__(self,
                 parent: gvsoc.systree.Component,
                 name: str,
                 num_macros: int = 2,
                 stream_chunk_bytes: int = 128,
                 stream_l1bw: int = 128,
                 stream_sync: int = 0,
                 stream_noc_lat: int = 1):
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
        })

    # Called from the tile/soc/board configure() chain so gvrun --param can set
    # these at launch time (mirrors PCM's set_stim_file / weights_path pattern).
    def set_stream_params(self, chunk_bytes=None, l1bw=None, sync=None, noc_lat=None):
        props = {}
        if chunk_bytes is not None: props["stream_chunk_bytes"] = int(chunk_bytes)
        if l1bw       is not None: props["stream_bank_bytes"]  = int(l1bw)
        if sync       is not None: props["stream_sync"]        = int(sync)
        if noc_lat    is not None: props["stream_noc_lat"]     = int(noc_lat)
        if props:
            self.add_properties(props)

    def i_hwpe_slv(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'hwpe_slv')

    def o_stream_mst(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('stream_mst', itf, signature='io')
