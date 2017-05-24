"""
Copyright 2015 Paul T. Grogan, Massachusetts Institute of Technology

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Operations class.
"""

import math
import itertools
import re
import sys

class Operations(object):
    def __init__(self):
        self.penaltyMemo = {}
        self.costISL = None
        self.costSGL = None
        self.groundSections = None
    
    def execute(self, controller, context):
        """
        Executes this operations model.
        @param controller: the controller for this operations model
        @type controller: L{Entity}
        @param context: the context of operations
        @type context: L{Context}
        """
        pass

    def finddeltatime(self, stationloc, elementloc):
        station_section = int(re.search(r'.+(\d)', stationloc).group(1))
        element_section = int(re.search(r'.+(\d)', elementloc).group(1))

        loc_diff = (station_section - element_section) if station_section>element_section else (6+station_section - element_section)

        loc_diff = 0
        if 'LE' in elementloc:
            if element_section % 2 != station_section % 2:
                return 0

        if element_section == station_section:
            return 0

        for i in range(element_section+1, station_section+1) if station_section>element_section else range(element_section+1, 7)+range(0, station_section+1):
                if 'LE' in elementloc:
                    if i%2 == element_section%2 and i in self.groundSections:
                        loc_diff += 1

                if 'ME' in elementloc and i in self.groundSections:
                    loc_diff += 1

        return loc_diff


    def getStoragePenalty(self, element, context, time, type = None):
        # if not element in self.penaltyMemo:
        #     demands = [e for e in context.events
        #                if e.isDemand()
        #                and element.couldSense(e.generateData())]
        #                #and (element.couldSense(e.generateData())
        #                #     or (any(m.isTransceiver() and m.isISL()
        #                #             for m in element.modules)))]
        #     values = [0]
        #     values.extend(map(lambda d: d.getValueAt(0), demands))
        #     values = list(set(values))
        #     values.sort()
        #     counts = map(lambda v: len([d for d in demands
        #                                 if d.getValueAt(0) == v]), values)
        #     counts[0] += len(context.events) - len(demands)
        #     print math.pow(sum(counts[0:values.index(values[-1])+1]), 1), math.pow(sum(counts[0:values.index(values[-1])]), 1)
        #     expValMax = reduce(lambda e, v:
        #                        e + v*(math.pow(sum(counts[0:values.index(v)+1]), 1)
        #                               - math.pow(sum(counts[0:values.index(v)]), 1))
        #                        / math.pow(sum(counts),1), values)
        #
        #     print values
        #     print counts
        #     print expValMax
        #     self.penaltyMemo[element] = -1*max(100, expValMax) # minimum penalty 100
        # print "time:", time
        myfederate = context.getElementOwner(element)
        federation = context.federations
        allfederates = [f.federates for f in context.federations][0]

        if type == 'independent':
            allstations = [e for e in myfederate.getElements() if e.isGround()]

        else:
            allstations = list(itertools.chain.from_iterable([[e for e in f.getElements() if e.isGround()] for f in allfederates]))
            if not self.groundSections:
                self.groundSections = [int(re.search(r'.+(\d)', s.getLocation()).group(1)) for s in allstations]
                print "Ground Sections:", self.groundSections

        mylocation = element.getLocation()
        # print time, element.getLocation(), mystationlocations, allstationlocations
        # print element.name, mylocation, otherstationlocations
        storage_opportunitycost = []
        counter, demand_value = element.getSensedDemands()
        # print "Counter and demand value: ", counter, demand_value

        for st in allstations:
            st_loc = st.getLocation()
            deltatime = self.finddeltatime(st_loc, mylocation)
            # print "Mylocation, stationlocation, allground locations:", mylocation, st_loc, deltatime

            prob = element.getDemandProb()
            # print element, " probability:", prob
            owner = context.getElementOwner(st)
            # print owner, myfederate
            costSGL = 0 if owner is myfederate else owner.getCost('oSGL')
            # print "counter:", counter
            newstorageopportunitycost = (demand_value*min(deltatime,4)-costSGL)*(1-(1-prob)**deltatime)
            # print "New storage opportunity cost:", demand_value, deltatime, prob, costSGL, newstorageopportunitycost
            storage_opportunitycost.append(newstorageopportunitycost)

        # print "Storage Opportunity cost:", storage_opportunitycost
        element.setStorageOpportunity(max(storage_opportunitycost)) #self.penaltyMemo[element]
        return max(storage_opportunitycost)

