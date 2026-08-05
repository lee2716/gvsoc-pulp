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

import gvsoc.systree
import gvsoc.runner
from gvrun.parameter import TargetParameter

from pulp.chips.democritos.democritos_soc import DemocritosSoc

class DemocritosBoard(gvsoc.systree.Component):
    def __init__(self, parent, name:str, parser, options):
        super().__init__(parent, name, options=options)

        TargetParameter(
            self, name='binary', value=None,
            description='Binary to be loaded and started', cast=str
        )

        # Soc model
        self.soc = DemocritosSoc(self, 'democritos-soc', parser)

    def configure(self):
        binary = self.get_parameter('binary')
        if binary is not None:
            self.soc.loader.set_binary(binary)

    def handle_binary(self, binary):
        self.set_parameter('binary', binary)


class Target(gvsoc.runner.Target):
    def __init__(self, parser, options, name=None):
        super(Target, self).__init__(parser, options,
              model=DemocritosBoard, description="Democritos test board")
