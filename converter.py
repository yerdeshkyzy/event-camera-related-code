# Copyright (c) Prophesee S.A.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.

"""
Metavision RAW to CSV python sample.
"""


from metavision_core.event_io import EventsIterator
import os
import pathlib
from math import floor

def main():
    """ Main """
    i=0
    current_file=pathlib.Path(__file__).parent.resolve()
    surfaces=['first','second','third','fourth','fifth']
    while(i<50):
        dirName=surfaces[floor(i/10)]
        trialNum=i%10
        path_to_file=(f"{dirName}/{trialNum}.raw")
        
        if not os.path.isfile(path_to_file):
            print('Fail to access RAW file ' + path_to_file)
            return
        
        if os.path.isfile(f"{dirName}/{trialNum}.csv"):
            print('file skipped because it already exists')
        else: 
            # Events iterator on Camera or RAW file
            mv_iterator = EventsIterator(input_path=path_to_file, delta_t=1000)

            with open(f"{dirName}/{trialNum}.csv", 'w') as csv_file:
                # Read formatted CD events from EventsIterator & write to CSV
                for evs in mv_iterator:
                    for (x, y, p, t) in evs:
                        csv_file.write("%d,%d,%d,%d\n" % (x, y, p, t))
        i=i+1
    return 0


if __name__ == "__main__":
    main()
