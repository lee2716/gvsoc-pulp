# Copyright (C) 2026 University of Bologna

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



# Authors: Alessandro Nadalini (alessandro.nadalini3@unibo.it)

import gvsoc.runner
from pulp.chips.raptor.raptor_board import RaptorBoard

class Target(gvsoc.runner.Target):
    gapy_description="Raptor board"
    model = RaptorBoard

    def __init__(self,parser,options=None, name=None):
        super(Target,self).__init__(parser,options,model=RaptorBoard)